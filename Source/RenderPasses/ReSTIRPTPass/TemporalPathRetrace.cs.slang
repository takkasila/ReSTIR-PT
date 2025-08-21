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
    Texture2D<PackedHitInfo> temporalVbuffer;
    Texture2D<float2> motionVectors;

    ByteAddressBuffer nRooksPattern;

    RWTexture2D<float4> outputColor;                      ///< Output resolved color.
    RWStructuredBuffer<PathReservoir> outputReservoirs;
    RWStructuredBuffer<PathReservoir> temporalReservoirs;
    RWStructuredBuffer<PixelReconnectionData> reconnectionDataBuffer;

    RWStructuredBuffer<DebugPathData> debugCentralResevoirPathDataBuffer;
    RWStructuredBuffer<DebugPathData> debugTemporalResevoirPathDataBuffer;

    int  gNumSpatialRounds;
    bool gEnableTemporalReprojection;

    float gTemporalHistoryLength;
    bool gNoResamplingForTemporalReuse;

    bool isValidPackedHitInfo(PackedHitInfo packed)
    {
        return packed.x != 0;
    }

    bool isValidScreenRegion(int2 pixel) { return all(pixel >= 0 && pixel < params.frameDim); }

    ShadingData getPixelShadingData(int2 pixel, out PackedHitInfo PrimaryHitPacked)
    {
        ShadingData sd = {};
        Ray ray = gScene.camera.computeRayPinhole(pixel, params.frameDim);
        PrimaryHitPacked = vbuffer[pixel];
        if (isValidPackedHitInfo(PrimaryHitPacked))
        {
            HitInfo PrimaryHit; PrimaryHit.unpack(PrimaryHitPacked);
            sd = loadShadingData(PrimaryHit, -ray.dir, true);
        }
        return sd;
    }

    ShadingData getPixelTemporalShadingData(int2 pixel, out PackedHitInfo PrimaryHitPacked)
    {
        ShadingData sd = {};
        Ray ray = gScene.camera.computeRayPinholePrevFrame(pixel, params.frameDim);
        PrimaryHitPacked = temporalVbuffer[pixel];
        if (isValidPackedHitInfo(PrimaryHitPacked))
        {
            HitInfo PrimaryHit; PrimaryHit.unpack(PrimaryHitPacked);
            sd = loadShadingData(PrimaryHit, -ray.dir, true);
        }
        return sd;
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
        bool isReservoirDD = reservoir.pathFlags.decodeIsDD();
        bool isReservoirDSD = reservoir.pathFlags.decodeIsDSD();
        bool isReservoirWeight0 = reservoir.weight == 0.f;

        bool isRcPrevValid = reconnectionData.rcPrevHit.isValid();
        bool isRcPrevWo0 = all(reconnectionData.rcPrevWo == 0.f);
        bool isRcPrevPrevValid = reconnectionData.rcPrevPrevHit.isValid();
        bool isRcPrevPrevWo0 = all(reconnectionData.rcPrevPrevWo == 0.f);
        bool isThroughput0 = all(reconnectionData.pathThroughput == 0.f);

        //
        // Case 1: Base path has no RC and weight == 0
        //
        if (!isReservoirDD & !isReservoirDSD & isReservoirWeight0)
        {
            if (!isRcPrevValid & !isRcPrevPrevValid & isRcPrevWo0 & isRcPrevPrevWo0 & isThroughput0)
            {
                // Valid
                return;
            }
            else
            {
                print("Alert! Invalid ReconnectionData case", 1);
            }
        }
        //
        // Case 2: Base path has no RC, but weight != 0
        //
        else if (!isReservoirDD & !isReservoirDSD & !isReservoirWeight0)
        {
            if (!isRcPrevValid & !isRcPrevPrevValid & isRcPrevWo0 & isRcPrevPrevWo0)
            {
                // In this case, RC's throughput can be any value
                // Valid
                return;
            }
            else
            {
                print("Alert! Invalid ReconnectionData case", 2);
            }
        }
        //
        // Case 3: Base path has DD RC
        //
        else if (isReservoirDD)
        {
            // Keep in mind that if reservoir rcVertexLength == 1, we skip retrace and use dummyRC.
            // Case 3.1: Offset path is invertible and has an RC
            if (isRcPrevValid & !isRcPrevPrevValid & !isRcPrevWo0 & isRcPrevPrevWo0 & !isThroughput0)
            {
                // Valid
                return;
            }
            // Case 3.2: Offset path couldn't find an RC or is not invertible.
            else if (!isRcPrevValid & !isRcPrevPrevValid & isRcPrevWo0 & isRcPrevPrevWo0 & isThroughput0)
            {
                // Valid
                return;
            }
            else
            {
                print("Alert! Invalid ReconnectionData case", 3);
            }
        }
        //
        // Case 4: Base path has DSD RC
        //
        else if (isReservoirDSD)
        {
            // Case 4.1: Offset path is invertible and has an RC
            if (isRcPrevValid & isRcPrevPrevValid & !isRcPrevWo0 & !isRcPrevPrevWo0 & !isThroughput0)
            {
                // Valid
                return;
            }
            // Case 4.2: Offset path couldn't find an RC or is not invertible.
            else if (!isRcPrevValid & !isRcPrevPrevValid & isRcPrevWo0 & isRcPrevPrevWo0 & isThroughput0)
            {
                // Valid
                return;
            }
            else
            {
                print("Alert! Invalid ReconnectionData case", 3);
            }
        }

        print("The ReconnectionData in question:", 0);
        print("- rcData.rcPrevHit", reconnectionData.rcPrevHit.isValid());
        print("- rcData.rcPrevPrevHit", reconnectionData.rcPrevPrevHit.isValid());
        print("- rcData.rcPrevWo", reconnectionData.rcPrevWo);
        print("- rcData.rcPrevPrevWo", reconnectionData.rcPrevPrevWo);
        print("- rcData.pathThroughput", reconnectionData.pathThroughput);
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
            debugTemporalResevoirPathDataBuffer[0] = debugPathData2;
        }

        // TODO: How to seed efficiently?
        var sg = TinyUniformSampleGenerator(pixel, (kCandidateSamples + 1 + gNumSpatialRounds) * params.seed + kCandidateSamples);

        // Compute offset into per-sample buffers. All samples are stored consecutively at this offset.
        const uint centralOffset = params.getReservoirOffset(pixel);
        PathReservoir centralReservoir = outputReservoirs[centralOffset];

        // X_0 = camera, X_1 = primary hit
        PackedHitInfo centralPrimaryHitPacked;
        ShadingData centralPrimarySd = getPixelShadingData(pixel, centralPrimaryHitPacked);
        if (!isValidPackedHitInfo(centralPrimaryHitPacked))
            return;

        // color averaging test

        // fetch temporal reservoir
        float2 motionVector = motionVectors[pixel];
        int2 prevPixel = pixel;

        // False by default
        if (gEnableTemporalReprojection)
        {
            prevPixel = pixel + motionVector * params.frameDim + (sampleNext2D(sg) * 1.f - 0.f);
        }

        if (!isValidScreenRegion(prevPixel))
            return;

        // Check if the location we are looking at has a surface
        PackedHitInfo temporalPrimaryHitPacked;
        ShadingData temporalPrimarySd = getPixelTemporalShadingData(prevPixel, temporalPrimaryHitPacked);
        if (!isValidPackedHitInfo(temporalPrimaryHitPacked))
            return;

        PathReservoir temporalReservoir = temporalReservoirs[params.getReservoirOffset(prevPixel)];

        print("Central reservoir:", 0);
        print("- rcVertexHit.instanceID", centralReservoir.rcVertexHit.instanceID);
        print("- rcPrevVertexHit.instanceID", centralReservoir.rcPrevVertexHit.instanceID);
        print("- rcPrevPrevVertexHit.instanceID", centralReservoir.rcPrevPrevVertexHit.instanceID);
        print("- isDD", centralReservoir.pathFlags.decodeIsDD());
        print("- isDSD", centralReservoir.pathFlags.decodeIsDSD());

        print("Temporal reservoir:", 0);
        print("- rcVertexHit.instanceID", temporalReservoir.rcVertexHit.instanceID);
        print("- rcPrevVertexHit.instanceID", temporalReservoir.rcPrevVertexHit.instanceID);
        print("- rcPrevPrevVertexHit.instanceID", temporalReservoir.rcPrevPrevVertexHit.instanceID);
        print("- isDD", temporalReservoir.pathFlags.decodeIsDD());
        print("- isDSD", temporalReservoir.pathFlags.decodeIsDSD());

        // talbot MIS
        // compute mis weight for current pixel

        ReconnectionData centralReservoirReconnectionData;
        ReconnectionData temporalReservoirReconnectionData;

        if (centralReservoir.pathFlags.rcVertexLength() > 1)
        {
            print("- traceHybridShiftRays centralReservoir: begin", 0);

            //  Use previous frame data
            traceHybridShiftRays(
                params,
                true,
                temporalPrimaryHitPacked,
                temporalPrimarySd,
                centralReservoir,
                centralReservoirReconnectionData,
                debugPathData1
            );

            print("- Output Central ReconnectionData", 0);
            print("-- rcData.rcPrevHit", centralReservoirReconnectionData.rcPrevHit.isValid());
            print("-- rcData.rcPrevPrevHit", centralReservoirReconnectionData.rcPrevPrevHit.isValid());
            print("-- rcData.rcPrevWo", centralReservoirReconnectionData.rcPrevWo);
            print("-- rcData.rcPrevPrevWo", centralReservoirReconnectionData.rcPrevPrevWo);
            print("-- rcData.pathThroughput", centralReservoirReconnectionData.pathThroughput);

            // Check if the case of need to fill in DSD data
            detectAndfillInAssumedSpecularVertex(centralReservoir, centralReservoirReconnectionData);

            // Set to buffer
            reconnectionDataBuffer[centralOffset].data[0] = centralReservoirReconnectionData;

            if (isDebug)
            {
                // Fill in RC data into debugData
                fillDebugPathData(centralReservoirReconnectionData, centralReservoir, debugPathData1, debugCentralResevoirPathDataBuffer);
            }

            // Sanity check
            checkValidReconnectionData(centralReservoir, centralReservoirReconnectionData);

            print("- traceHybridShiftRays centralReservoir: end", 0);
        }
        else
        {
            print("- centralReservoir doesn't have RC", 0);

            // Set ReconnectionData throughput to 0 and move on
            centralReservoirReconnectionData.pathThroughput = float3(0.f);
            reconnectionDataBuffer[centralOffset].data[0] = centralReservoirReconnectionData;
        }

        if (temporalReservoir.pathFlags.rcVertexLength() > 1)
        {
            print("- traceHybridShiftRays temporalReservoir: begin", 0);

            traceHybridShiftRays(
                params,
                false,
                centralPrimaryHitPacked,
                centralPrimarySd,
                temporalReservoir,
                temporalReservoirReconnectionData,
                debugPathData2
            );

            print("- Output Temporal ReconnectionData", 0);
            print("-- rcData.rcPrevHit", temporalReservoirReconnectionData.rcPrevHit.isValid());
            print("-- rcData.rcPrevPrevHit", temporalReservoirReconnectionData.rcPrevPrevHit.isValid());
            print("-- rcData.rcPrevWo", temporalReservoirReconnectionData.rcPrevWo);
            print("-- rcData.rcPrevPrevWo", temporalReservoirReconnectionData.rcPrevPrevWo);
            print("-- rcData.pathThroughput", temporalReservoirReconnectionData.pathThroughput);

            // Check if the case of need to fill in DSD data
            detectAndfillInAssumedSpecularVertex(temporalReservoir, temporalReservoirReconnectionData);

            // Set to buffer
            reconnectionDataBuffer[centralOffset].data[1] = temporalReservoirReconnectionData;

            if (isDebug)
            {
                // Fill in RC data into debugData
                fillDebugPathData(temporalReservoirReconnectionData, temporalReservoir, debugPathData2, debugTemporalResevoirPathDataBuffer);
            }

            // Sanity check
            checkValidReconnectionData(temporalReservoir, temporalReservoirReconnectionData);

            print("- traceHybridShiftRays temporalReservoir: end", 0);

        }
        else
        {
            print("- temporalReservoir doesn't have RC", 0);

            // Set ReconnectionData throughput to 0 and move on
            temporalReservoirReconnectionData.pathThroughput = float3(0.f);
            reconnectionDataBuffer[centralOffset].data[1] = temporalReservoirReconnectionData;
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

        print("Temporal path retrace: begin", 0);
        ReSTIR(pixel);
        print("Temporal path retrace: end", 0);
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
