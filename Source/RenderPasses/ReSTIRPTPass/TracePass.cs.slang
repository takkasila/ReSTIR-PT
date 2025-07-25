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
    uint vertexCount = 0;

    print("pixel", pixel);
    print("pathID", pathID);

    while (path.isActive())
    {
        print("vertex", path.origin);
        print("path.length", path.length);

        //  vertex 0 at camera
        if (path.length == 0 && all(selectedPixel == pixel))
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
            if (all(selectedPixel == pixel))
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
                print("Terminated", 0);
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

        print("in loop path.pathReservoir.rcVertexHit.instanceID", path.pathReservoir.rcVertexHit.instanceID);

        print("", 0);   // empty line
    }

    print("Path Radiance", path.L);

    if (all(selectedPixel == pixel))
    {
        pixelDebugPathDataBuffer[0].vertexCount = vertexCount;

        pixelDebugPathDataBuffer[0].L = float4(path.L, 1);

        //  Check if have RC vertex
        if (path.pathBuilder.rcVertexHit.instanceID == 0xffffffff)
        {
            print("Doesn't have RC vertex", 0);
            pixelDebugPathDataBuffer[0].hasRcVertex = false;
        }
        else
        {
            print("Have RC vertex", 0);
            pixelDebugPathDataBuffer[0].hasRcVertex = true;
            pixelDebugPathDataBuffer[0].rcVertexIndex = path.pathBuilder.rcVertexLength + 1;

            bool isPrevRcVertexSpecular = path.pathBuilder.pathFlags.decodeIsSpecularBounce(true);

            if (isPrevRcVertexSpecular)
            {
                // We have RC for HybridShiftSMS
                print("- RC is DSD for HybridShift", 0);
                pixelDebugPathDataBuffer[0].isRcDSD = true;
            }
        }

    }

    gPathTracer.finalize(path);

    const uint reservoirIdx = gPathTracer.params.getReservoirOffset(pixel);

    gPathTracer.writeOutput(path, giReservoir, sampleIdx);

    print("Output reservoir:", 0);
    print("- rcVertexHit id:", gPathTracer.outputReservoirs[reservoirIdx].rcVertexHit.instanceID);
    print("- rcPrevVertexHit id:", gPathTracer.outputReservoirs[reservoirIdx].rcPrevVertexHit.instanceID);
    print("- rcPrevPrevVertexHit id:", gPathTracer.outputReservoirs[reservoirIdx].rcPrevPrevVertexHit.instanceID);

    bool isDelta1 = giReservoir.pathFlags.decodeIsDeltaEvent(true);
    bool isDelta2 = giReservoir.pathFlags.decodeIsDeltaEvent(false);

    if(isDelta1 || isDelta2)
    {
        print("- base path does have some delta", (isDelta1 ? 1 : 0) | (isDelta2 ? 2 : 0));
    }
    else
    {
        print("- base path does not have delta", 0);
    }
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
