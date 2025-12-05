/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/
import Scene.Raytracing;
import Utils.Debug.PixelDebug;
import Rendering.Utils.PixelStats;
import PathTracer;
import PathState;
import Params;
import PathReservoir;
import DebugPathDataType;
import LoadShadingData;

ParameterBlock<PathTracer> gPathTracer;

RWStructuredBuffer<DebugPathData> pixelDebugPathDataBuffer;

cbuffer CB
{
    int gSampleId;
}

void tracePath(uint pathID, inout PathReservoir giReservoir, uint sampleIdx)
{
    PathState path;
    gPathTracer.generatePath(pathID, path);

    uint2 pixel = path.getPixel();

    printSetPixel(pixel);
    logSetPixel(pixel);

    uint2 selectedPixel = getSelctedPixelLog();

    bool isAtDebugPixel = all(selectedPixel == pixel);

    uint vertexCount = 0;
    uint loopCount = 0;

    print("{tracePath", 0);

    print("pixel", pixel);
    print("pathID", pathID);

    print("{traceLoop", 0);
    while (path.isActive())
    {
        print("{i", loopCount);
        print("vertex pos", path.origin);
        print("path.length", path.length);

        //  vertex 0 at camera
        if (path.length == 0 && isAtDebugPixel)
        {
            pixelDebugPathDataBuffer[0].vertices[0] = float4(path.origin, 1);
            vertexCount += 1;
        }

        if (path.isHit())
        {
            DebugVertexState debugVertexState;
            debugVertexState.isSampledLight = false;

            gPathTracer.handleHit(path, debugVertexState);

            //  Record path data for visualization
            if (isAtDebugPixel)
            {
                pixelDebugPathDataBuffer[0].vertices[vertexCount] = float4(debugVertexState.pos, 1);
                pixelDebugPathDataBuffer[0].isSampledLight[vertexCount] = debugVertexState.isSampledLight;
                pixelDebugPathDataBuffer[0].sampledLightPosition[vertexCount] = float4(debugVertexState.sampledLightPosition, 1);
                pixelDebugPathDataBuffer[0].sampledLightRadiance[vertexCount] = float4(debugVertexState.sampledLightRadiance, 1);
                vertexCount += 1;
            }

            // After handle
            if (path.isTerminated())
            {
                print("terminated", 0);
                print("}i", loopCount);
                break;
            }

            //  Trace a new ray.
            //      This also increases path.length
            gPathTracer.nextVertex(path);
        }
        else
        {
            print("miss", 0);
            gPathTracer.handleMiss(path);
        }

        print("}i", loopCount);
        loopCount += 1;
    }
    print("}traceLoop", 0);

    print("Path Radiance", path.L);

    gPathTracer.finalize(path);

    gPathTracer.writeOutput(path, giReservoir, sampleIdx);
    // giReservoir is just a temporary middle man.
    // The final output is in gPathTracer.outputReservoirs[reservoirIdx]

    const uint reservoirIdx = gPathTracer.params.getReservoirOffset(pixel);

    const PathReservoir outputReservoirs = gPathTracer.outputReservoirs[reservoirIdx];
    const float4 outputColor = gPathTracer.outputColor[reservoirIdx];

    print("Output reservoir", 0);
    print("- rcVertexHit id:", outputReservoirs.rcVertexHit.instanceID);
    print("- rcPrevVertexHit id:", outputReservoirs.rcPrevVertexHit.instanceID);
    print("- rcPrevPrevVertexHit id:", outputReservoirs.rcPrevPrevVertexHit.instanceID);
    print("- isDD", outputReservoirs.pathFlags.decodeIsDD());
    print("- isDSD", outputReservoirs.pathFlags.decodeIsDSD());
    print("- rcVertexLength", outputReservoirs.pathFlags.rcVertexLength());
    print("- pathLength", outputReservoirs.pathFlags.pathLength());
    print("- weight", outputReservoirs.weight);
    print("- rcVertexIrradiance", outputReservoirs.rcVertexIrradiance[0]);
    print("- rcPrevPrevVertexWo", outputReservoirs.rcPrevPrevVertexWo);

    if (isAtDebugPixel)
    {
        // Check Reservoir validity
        checkReservoirValidity(outputReservoirs);

        // Fill in debutPathData
        pixelDebugPathDataBuffer[0].vertexCount = vertexCount;

        pixelDebugPathDataBuffer[0].L = float4(path.L, 1);
    }

    print("}tracePath", 0);
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    uint2 pixel = dispatchThreadId.xy;
    if (all(pixel >= gPathTracer.params.frameDim)) return;

    // Skip pixel if there is no hit in the vbuffer.
    HitInfo hit = HitInfo(gPathTracer.vbuffer[pixel]);
    if (!hit.isValid()) return;

    PathReservoir giReservoir;

    static const uint itersPerShaderPass = PathSamplingMode(kPathSamplingMode) == PathSamplingMode::PathTracing ? kSamplesPerPixel : kCandidateSamples;

    if (itersPerShaderPass == 1)
    {
        // Handle fixed 1 spp case.
        uint pathID = pixel.x | (pixel.y << 12) | (gSampleId << 24);
        tracePath(pathID, giReservoir, 0);
    }
    else//(itersPerShaderPass > 1)
    {
        // Handle fixed multiple spp case.
        for (uint sampleIdx = 0; sampleIdx < itersPerShaderPass; ++sampleIdx)
        {
            uint pathID = pixel.x | (pixel.y << 12) | (sampleIdx << 24);
            tracePath(pathID, giReservoir, sampleIdx);
        }
    }
}
