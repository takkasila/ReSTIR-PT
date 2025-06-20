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

    /*
    * Build a reservoir of paths whose suffix can be merged to a pixel's prefix.
    * This method is optimized for small windows such that we can run the entire
    * algorithm in one pass.
    */
    void ReSTIR(const uint2 pixel)
    {
        print("Temporal path retrace: begin", 0);

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
        if (!isValidPackedHitInfo(centralPrimaryHitPacked)) return;

        // color averaging test

        // fetch temporal reservoir
        float2 motionVector = motionVectors[pixel];
        int2 prevPixel = pixel;

        bool foundTemporalSurface = true;

        // False by default
        if (gEnableTemporalReprojection)
        {
            prevPixel = pixel + motionVector * params.frameDim + (sampleNext2D(sg) * 1.f - 0.f);
        }

        if (!isValidScreenRegion(prevPixel) || !foundTemporalSurface) return;

        PackedHitInfo temporalPrimaryHitPacked;
        ShadingData temporalPrimarySd = getPixelTemporalShadingData(prevPixel, temporalPrimaryHitPacked);
        if (!isValidPackedHitInfo(temporalPrimaryHitPacked)) return;

        PathReservoir temporalReservoir = temporalReservoirs[params.getReservoirOffset(prevPixel)];

        print("Central reservoir:", 0);
        print("- rcVertexHit.instanceID", centralReservoir.rcVertexHit.instanceID);
        print("- rcPrevVertexHit.instanceID", centralReservoir.rcPrevVertexHit.instanceID);
        print("- rcPrevPrevVertexHit.instanceID", centralReservoir.rcPrevPrevVertexHit.instanceID);

        print("temporalReservoir reservoir:", 0);
        print("- rcVertexHit.instanceID", temporalReservoir.rcVertexHit.instanceID);
        print("- rcPrevVertexHit.instanceID", temporalReservoir.rcPrevVertexHit.instanceID);
        print("- rcPrevPrevVertexHit.instanceID", temporalReservoir.rcPrevPrevVertexHit.instanceID);

        // talbot MIS
        // compute mis weight for current pixel

        ReconnectionData centralReservoirReconnectionData;
        ReconnectionData temporalReservoirReconnectionData;

        // write to memory
        if (centralReservoir.pathFlags.rcVertexLength() > 1)
        {
            print("- traceHybridShiftRays centralReservoir", 0);

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

            reconnectionDataBuffer[centralOffset].data[0] = centralReservoirReconnectionData;

            if (isDebug)
            {
                print("- debugPathData1.vertexCount", debugPathData1.vertexCount);
                print("- rcVertexLength", centralReservoir.pathFlags.rcVertexLength());
                print("- pathLength", centralReservoir.pathFlags.pathLength());

                bool isValidRCD = (
                    any(centralReservoirReconnectionData.pathThroughput > 0.f)
                    && (centralReservoirReconnectionData.rcPrevHit.isValid())
                );

                if (isValidRCD)
                {
                    // Add a RC vertex at the end
                    TriangleHit centralReservoirRCVertexHit = centralReservoir.rcVertexHit.getHitInfo().getTriangleHit();
                    VertexData rcVertex = gScene.getVertexData(centralReservoirRCVertexHit);
                    debugPathData1.vertices[debugPathData1.vertexCount] = float4(rcVertex.posW, 1);
                    debugPathData1.vertexCount += 1;
                }

                debugCentralResevoirPathDataBuffer[0] = debugPathData1;
            }
        }

        if (temporalReservoir.pathFlags.rcVertexLength() > 1)
        {
            print("- traceHybridShiftRays temporalReservoir", 0);

            traceHybridShiftRays(
                params,
                false,
                centralPrimaryHitPacked,
                centralPrimarySd,
                temporalReservoir,
                temporalReservoirReconnectionData,
                debugPathData2
            );

            reconnectionDataBuffer[centralOffset].data[1] = temporalReservoirReconnectionData;

            if (isDebug)
            {
                bool isValidRCD = (
                    any(temporalReservoirReconnectionData.pathThroughput > 0.f)
                    && (temporalReservoirReconnectionData.rcPrevHit.isValid())
                );

                if(isValidRCD)
                {
                    // Add RC vertex at the end
                    TriangleHit temporalReservoirRCVertexHit = temporalReservoir.rcVertexHit.getHitInfo().getTriangleHit();
                    VertexData rcVertex = gScene.getVertexData(temporalReservoirRCVertexHit);
                    debugPathData2.vertices[debugPathData2.vertexCount] = float4(rcVertex.posW, 1);
                    debugPathData2.vertexCount += 1;
                }

                debugTemporalResevoirPathDataBuffer[0] = debugPathData2;

            }
        }
        print("Temporal path retrace: end", 0);
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

        ReSTIR(pixel);
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
