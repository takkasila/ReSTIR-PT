/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/
import Params;
import Shift;
import Scene.HitInfo;
import PathReservoir;
import PathTracer;
import Scene.Scene;
import Utils.Debug.PixelDebug;
import Rendering.Utils.PixelStats;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Math.Ray;
import Utils.Math.MathHelpers;
import LoadShadingData;
import Rendering.Materials.MaterialShading;
import DebugPathDataType;

/** TODO.
*/
struct PathRetracePass
{
    static const uint kNeighborOffsetCount = NEIGHBOR_OFFSET_COUNT;
    static const uint kNeighborOffsetMask = kNeighborOffsetCount - 1;

    // Resources
    RestirPathTracerParams params;                        ///< Runtime parameters.

    Texture2D<PackedHitInfo> vbuffer;                     ///< Fullscreen V-buffer for the primary hits.
    Texture1D<float2> neighborOffsets;

    ByteAddressBuffer nRooksPattern;

    RWTexture2D<float4> outputColor;                      ///< Output resolved color.
    RWStructuredBuffer<PathReservoir> outputReservoirs;                  ///< New per-pixel paths.
    RWStructuredBuffer<PathReservoir> temporalReservoirs;
    RWStructuredBuffer<PixelReconnectionData> reconnectionDataBuffer;

    RWStructuredBuffer<DebugPathData> debugCentralResevoirPathDataBuffer;
    RWStructuredBuffer<DebugPathData> debugNeighborResevoirPathDataBuffer;

    int gSpatialRoundId;
    int  gNumSpatialRounds;
    uint gSpatialReusePattern;
    // for default pattern
    int gNeighborCount;
    float gGatherRadius;
    // for small window
    int gSmallWindowRadius;

    // for linear growth of M when merging reservoirs

    bool gFeatureBasedRejection;

    bool isValidPackedHitInfo(PackedHitInfo packed)
    {
        return packed.x != 0;
    }

    bool isValidScreenRegion(int2 pixel) { return all(pixel >= 0 && pixel < params.frameDim); }

    bool isValidGeometry(ShadingData centralSd, ShadingData neighborSd)
    {
        if (!gFeatureBasedRejection) return true;
        float3 camPos = gScene.camera.data.posW;
        float centralDist = distance(camPos, centralSd.posW);
        float neighborDist = distance(camPos, neighborSd.posW);
        return dot(centralSd.N, neighborSd.N) >= 0.5f && abs(centralDist - neighborDist) < 0.1f * centralDist;
    }

    ShadingData getPixelShadingData(int2 pixel, out PackedHitInfo PrimaryHitPacked)
    {
        ShadingData sd = {};
        Ray ray = gScene.camera.computeRayPinhole(pixel, params.frameDim);
        PrimaryHitPacked = vbuffer[pixel];
        if (isValidPackedHitInfo(PrimaryHitPacked))
        {
            HitInfo PrimaryHit; PrimaryHit.unpack(PrimaryHitPacked);

            TriangleHit th = PrimaryHit.getTriangleHit();
            sd = loadShadingData(PrimaryHit, -ray.dir, true);
        }
        return sd;
    }

    int getNeighborCount()
    {
        return gNeighborCount; // does not include self
    }

    int2 getNextNeighborPixel(const uint startIndex, int2 pixel, int i)
    {
        int2 neighborPixel = int2(0,0);

        if (SpatialReusePattern(gSpatialReusePattern) == SpatialReusePattern::Default)
        {
            uint neighborIndex = (startIndex + i) & kNeighborOffsetMask;
            neighborPixel = pixel + int2(neighborOffsets[neighborIndex] * gGatherRadius);
        }
        else if (SpatialReusePattern(gSpatialReusePattern) == SpatialReusePattern::SmallWindow)
        {
            int smallWindowDiameter = 2 * gSmallWindowRadius + 1;
            neighborPixel = pixel + int2(-gSmallWindowRadius + (i % smallWindowDiameter),
                -gSmallWindowRadius + (i / smallWindowDiameter));
            if (all(neighborPixel == pixel)) neighborPixel = int2(-1);
        }

        return neighborPixel;
    }

    void detectAndfillInAssumedSpecularVertex(PathReservoir reservoir, inout ReconnectionData reconnectionData)
    {
        bool isReservoirDSD = reservoir.pathFlags.decodeIsDSD();

        // Check if the case of need to fill in DSD data
        if (
            // Base path is DSD
            isReservoirDSD
            // Valid RC
            && any(reconnectionData.pathThroughput > 0.f)
            // Specular vertex has been assumed (rcPrevHit)
            && !reconnectionData.rcPrevHit.isValid()
            && reconnectionData.rcPrevPrevHit.isValid()
        )
        {
            print("- fill in assumed specular vertex", 0);

            reconnectionData.rcPrevHit = reservoir.rcPrevVertexHit.getHitInfo();

            // Compute rcPrevWo
            TriangleHit rcPrevHit = reservoir.rcPrevVertexHit.getHitInfo().getTriangleHit();
            float3 rcPrevVertexPos = gScene.getVertexData(rcPrevHit).posW;

            TriangleHit rcPrevPrevHit = reservoir.rcPrevPrevVertexHit.getHitInfo().getTriangleHit();
            float3 rcPrevPrevVertexPos = gScene.getVertexData(rcPrevPrevHit).posW;

            reconnectionData.rcPrevWo = normalize(rcPrevPrevVertexPos - rcPrevVertexPos);
        }
    }

    void fillDebugPathData(
        ReconnectionData reconnectionData
        , PathReservoir reservoir
        , inout DebugPathData debugPathData
        , in RWStructuredBuffer<DebugPathData> debugPathDataBuffer
    )
    {
        // Fill in RC data into debugData
        if (any(reconnectionData.pathThroughput > 0.f))
        {
            debugPathData.hasRcVertex = true;

            bool isReservoirDSD = reservoir.pathFlags.decodeIsDSD();
            bool isReservoirDD = reservoir.pathFlags.decodeIsDD();

            if (isReservoirDD)
            {
                // DD RC
                debugPathData.isRcDSD = false;
                debugPathData.vertexCount = reservoir.pathFlags.rcVertexLength() + 2;

                // RC
                TriangleHit rcVertexHit = reservoir.rcVertexHit.getHitInfo().getTriangleHit();
                VertexData rcVertex = gScene.getVertexData(rcVertexHit);

                debugPathData.vertices[debugPathData.vertexCount - 1] = float4(rcVertex.posW, 1);

                debugPathData.rcVertexIndex = debugPathData.vertexCount - 1;
            }
            else if (isReservoirDSD)
            {
                // DSD RC
                debugPathData.isRcDSD = true;
                debugPathData.vertexCount = reservoir.pathFlags.rcVertexLength() + 2;

                // RC
                TriangleHit rcVertexHit = reservoir.rcVertexHit.getHitInfo().getTriangleHit();
                VertexData rcVertex = gScene.getVertexData(rcVertexHit);

                debugPathData.vertices[debugPathData.vertexCount - 1] = float4(rcVertex.posW, 1);

                debugPathData.rcVertexIndex = debugPathData.vertexCount - 1;

                // RC Prev
                TriangleHit rcPrevVertexHit = reservoir.rcPrevVertexHit.getHitInfo().getTriangleHit();
                VertexData rcPrevVertex = gScene.getVertexData(rcPrevVertexHit);

                debugPathData.vertices[debugPathData.vertexCount - 2] = float4(rcPrevVertex.posW, 1);
            }
        }

        debugPathDataBuffer[0] = debugPathData;
    }

    void checkValidReconnectionData(PathReservoir reservoir, ReconnectionData reconnectionData)
    {
        bool rcHasThroughput = any(reconnectionData.pathThroughput > 0.f);

        // reservoir.weight == 0
        if (!any(reservoir.weight > 0.f))
        {
            if (rcHasThroughput)
            {
                print("Alert! ReconnectionData has throughput but reservoir is empty", 0);
            }
            return;
        }

        // rc.pathThrougput == 0
        if (!any(reconnectionData.pathThroughput > 0.f))
        {
            return;
        }

        bool isReservoirDD = reservoir.pathFlags.decodeIsDD();
        bool isReservoirDSD = reservoir.pathFlags.decodeIsDSD();

        bool isRcPrevHitValid = reconnectionData.rcPrevHit.isValid();
        bool isRcPrevPrevHitValid = reconnectionData.rcPrevPrevHit.isValid();

        if (!rcHasThroughput & (isRcPrevHitValid | isRcPrevPrevHitValid))
        {
            print("Alert! Doesn't have throughput but have valid hitinfo", 0);
        }

        if (isRcPrevHitValid)
        {
            if (isReservoirDD)
            {
                // ReconnectionData must also be for DD
                if (!isRcPrevPrevHitValid)
                {
                    if (!rcHasThroughput)
                    {
                        print("Alert! isRcPrevHitValid but has no throughput", 0);
                    }
                }
                else
                {
                    print("Alert! got unwanted rcPrevPrevHit", 0);
                }
            }
            else if (isReservoirDSD)
            {
                // ReconnectionData must also be for DSD
                if (isRcPrevPrevHitValid)
                {
                    if (!rcHasThroughput)
                    {
                        print("Alert! isRcPrevHitValid & isRcPrevPrevHitValid but has no throughput", 0);
                    }
                }
                else
                {
                    print("Alert! missing rcPrevPrevHit", 0);
                }
            }
        }
        else
        {
            print("Alert! missing rcPrevHit", 0);
        }
    }

    /*
    * Build a reservoir of paths whose suffix can be merged to a pixel's prefix.
    * This method is optimized for small windows such that we can run the entire
    * algorithm in one pass.
    */
    void ReSTIR(const uint2 pixel)
    {

        DebugPathData debugPathData1;
        DebugPathData debugPathData2;
        debugPathData1.init();
        debugPathData2.init();

        bool isDebug = all(getSelctedPixelLog() == pixel);
        if (isDebug)
        {
            debugCentralResevoirPathDataBuffer[0] = debugPathData1;
            debugNeighborResevoirPathDataBuffer[0] = debugPathData2;
        }

        // TODO: How to seed efficiently?
        var sg = TinyUniformSampleGenerator(pixel, (kCandidateSamples + 1 + gNumSpatialRounds) * params.seed + kCandidateSamples + 1 + gSpatialRoundId);

        // Compute offset into per-sample buffers. All samples are stored consecutively at this offset.
        const uint centralOffset = params.getReservoirOffset(pixel);
        PathReservoir centralReservoir = outputReservoirs[centralOffset];

        // Check if the location we are looking at has a surface
        PackedHitInfo centralPrimaryHitPacked;
        ShadingData centralPrimarySd = getPixelShadingData(pixel, centralPrimaryHitPacked);
        if (!isValidPackedHitInfo(centralPrimaryHitPacked))
            return;

        print("Central reservoir:", 0);
        print("- rcVertexHit.instanceID", centralReservoir.rcVertexHit.instanceID);
        print("- rcPrevVertexHit.instanceID", centralReservoir.rcPrevVertexHit.instanceID);
        print("- rcPrevPrevVertexHit.instanceID", centralReservoir.rcPrevPrevVertexHit.instanceID);
        print("- isDD", centralReservoir.pathFlags.decodeIsDD());
        print("- isDSD", centralReservoir.pathFlags.decodeIsDSD());
        print("- weight", centralReservoir.weight);

        if(!centralReservoir.pathFlags.decodeIsDD() & !centralReservoir.pathFlags.decodeIsDSD())
        {
            print("Alert! Central Reservoir is neither DD nor DSD", 0);
        }

        uint2 chosenPixel = pixel;
        int chosen_i = -1;
        float3 color = 0.f;
        int neighborCount = getNeighborCount();

        print("Neighbor count:", neighborCount);

        const uint startIndex = sampleNext1D(sg) * kNeighborOffsetCount;

        for (int i = 0; i < neighborCount; ++i)
        {
            print("- neighbor", i);

            int2 neighborPixel = getNextNeighborPixel(startIndex, pixel, i);

            // Check if not in screen
            if (!isValidScreenRegion(neighborPixel))
                continue;

            PathReservoir neighborReservoir = outputReservoirs[params.getReservoirOffset(neighborPixel)];

            print("- Neighbor reservoir:", 0);
            print("-- rcVertexHit.instanceID", neighborReservoir.rcVertexHit.instanceID);
            print("-- rcPrevVertexHit.instanceID", neighborReservoir.rcPrevVertexHit.instanceID);
            print("-- rcPrevPrevVertexHit.instanceID", neighborReservoir.rcPrevPrevVertexHit.instanceID);
            print("-- isDD", neighborReservoir.pathFlags.decodeIsDD());
            print("-- isDSD", neighborReservoir.pathFlags.decodeIsDSD());
            print("-- weight", neighborReservoir.weight);

            PackedHitInfo neighborPrimaryHitPacked;
            ShadingData neighborPrimarySd = getPixelShadingData(neighborPixel, neighborPrimaryHitPacked);
            // Check if not
            if (!isValidPackedHitInfo(neighborPrimaryHitPacked))
                continue;

            // Both surfaces are close and plausible to connect with each other
            if (!isValidGeometry(centralPrimarySd, neighborPrimarySd))
                continue;

            if(!neighborReservoir.pathFlags.decodeIsDD() & !neighborReservoir.pathFlags.decodeIsDSD())
            {
                print("Alert! Neighbor reservoir is neither DD nor DSD", 0);
            }

            ReconnectionData centralReservoirReconnectionData;
            ReconnectionData neighborReservoirReconnectionData;

            if (centralReservoir.rcVertexHit.getHitInfo().isValid())
            {
                print("- traceHybridShiftRays centralReservoir: begin", 0);

                traceHybridShiftRays(
                    params,
                    false,
                    neighborPrimaryHitPacked,
                    neighborPrimarySd,
                    centralReservoir,
                    centralReservoirReconnectionData,
                    debugPathData1
                );

                print("-- rcData.rcPrevHit", centralReservoirReconnectionData.rcPrevHit.isValid());
                print("-- rcData.rcPrevPrevHit", centralReservoirReconnectionData.rcPrevPrevHit.isValid());
                print("-- rcData.rcPrevWo", centralReservoirReconnectionData.rcPrevWo);
                print("-- rcData.rcPrevPrevWo", centralReservoirReconnectionData.rcPrevPrevWo);
                print("-- rcData.pathThroughput", centralReservoirReconnectionData.pathThroughput);

                // Check if the case of need to fill in DSD data
                detectAndfillInAssumedSpecularVertex(centralReservoir, centralReservoirReconnectionData);


                bool isRcPrevHitValid = centralReservoirReconnectionData.rcPrevHit.isValid();
                if (!isRcPrevHitValid)
                {
                    print("- doesn't have prevRC", 0);
                }

                // Set to buffer
                reconnectionDataBuffer[centralOffset].data[2 * i] = centralReservoirReconnectionData;

                if (isDebug)
                {
                    // Fill in RC data into debugData
                    fillDebugPathData(centralReservoirReconnectionData, centralReservoir, debugPathData1, debugCentralResevoirPathDataBuffer);
                }

                // Sanity check
                checkValidReconnectionData(centralReservoir, centralReservoirReconnectionData);

                print("- traceHybridShiftRays centralReservoir: end", 0);
            }

            if (neighborReservoir.rcVertexHit.getHitInfo().isValid())
            {
                print("- traceHybridShiftRays neighborReservoir: begin", 0);

                traceHybridShiftRays(
                    params,
                    false,
                    centralPrimaryHitPacked,
                    centralPrimarySd,
                    neighborReservoir,
                    neighborReservoirReconnectionData,
                    debugPathData2
                );

                print("-- rcData.rcPrevHit", neighborReservoirReconnectionData.rcPrevHit.isValid());
                print("-- rcData.rcPrevPrevHit", neighborReservoirReconnectionData.rcPrevPrevHit.isValid());
                print("-- rcData.rcPrevWo", neighborReservoirReconnectionData.rcPrevWo);
                print("-- rcData.rcPrevPrevWo", neighborReservoirReconnectionData.rcPrevPrevWo);
                print("-- rcData.pathThroughput", neighborReservoirReconnectionData.pathThroughput);

                bool isRcPrevHitValid = neighborReservoirReconnectionData.rcPrevHit.isValid();

                if (!isRcPrevHitValid)
                {
                    print("- doesn't have prevRC", 0);
                }

                // Check if the case of need to fill in DSD data
                detectAndfillInAssumedSpecularVertex(neighborReservoir, neighborReservoirReconnectionData);

                // Set to buffer
                reconnectionDataBuffer[centralOffset].data[2 * i + 1] = neighborReservoirReconnectionData;

                if (isDebug)
                {
                    // Fill in RC data into debugData
                    fillDebugPathData(neighborReservoirReconnectionData, neighborReservoir, debugPathData2, debugNeighborResevoirPathDataBuffer);
                }

                // Sanity check
                checkValidReconnectionData(neighborReservoir, neighborReservoirReconnectionData);

                print("- traceHybridShiftRays neighborReservoir: end", 0);
            }
        }

    }

    /** Entry point for merge pass.
        \param[in] pixel Pixel coordinates.
    */
    void execute(const uint2 _pixel)
    {
        const uint2 pixel = _pixel;

        if (any(pixel >= params.frameDim)) return;

        printSetPixel(pixel);
        logSetPixel(pixel);

        print("Spatial path retrace: begin", 0);
        ReSTIR(pixel);
        print("Spatial path retrace: end", 0);
    }
};

cbuffer CB
{
    PathRetracePass gPathRetracePass;
}


[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gPathRetracePass.execute(dispatchThreadId.xy);
}
