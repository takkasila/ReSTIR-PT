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

    printSetPixel(path.getPixel());
    logSetPixel(path.getPixel());

    uint2 selectedPixel = getSelctedPixelLog();

    print("pixel", path.getPixel());
    print("pathID", pathID);
    print("", 0);

    while (path.isActive())
    {
        print("vertex", path.origin);

        if (all(selectedPixel == path.getPixel()))
        {
            pixelDebugPathDataBuffer[0].vertices[(int)path.length] = path.origin;
        }

        if (path.isHit())
        {
            gPathTracer.handleHit(path);

            if (path.isTerminated())
            {
                print("Terminated", 0);
                break;
            }
            gPathTracer.nextVertex(path);
        }
        else
        {
            print("miss", 0);
            gPathTracer.handleMiss(path);
        }

        print("", 0);
    }

    print("Path Radiance", path.L);


    if(all(selectedPixel == path.getPixel()))
    {
        if (path.pathBuilder.rcVertexHit.instanceID == 0xffffffff)
        {
            print("Doesn't have RC vertex", 0);
            pixelDebugPathDataBuffer[0].hasRCVertex = false;
        }
        else
        {
            print("Have RC vertex", 0);
            pixelDebugPathDataBuffer[0].hasRCVertex = true;
            pixelDebugPathDataBuffer[0].rcVertexIndex = path.pathBuilder.rcVertexLength + 1;
        }

        pixelDebugPathDataBuffer[0].length = path.length + 1;

        pixelDebugPathDataBuffer[0].L = path.L;
    }

    gPathTracer.finalize(path);
    gPathTracer.writeOutput(path, giReservoir, sampleIdx);
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
