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

void checkReservoirValidity(PathReservoir reservoir)
{
    bool isDD = reservoir.pathFlags.decodeIsDD();
    bool isDSD = reservoir.pathFlags.decodeIsDSD();
    bool isWeight0 = reservoir.weight == 0.f;

    bool isRcValid = reservoir.rcVertexHit.getHitInfo().isValid();
    bool isRcPrevValid = reservoir.rcPrevVertexHit.getHitInfo().isValid();
    bool isRcPrevPrevValid = reservoir.rcPrevPrevVertexHit.getHitInfo().isValid();
    bool isRcPrevPrevWo0 = all(reservoir.rcPrevPrevVertexWo == 0.f);

    int pathLength = reservoir.pathFlags.pathLength();
    int rcVertexLength = reservoir.pathFlags.rcVertexLength();

    //
    // Case 1: no RC
    //
    if (!isDD && !isDSD)
    {
        if (!isRcValid && !isRcPrevValid && !isRcPrevPrevValid && (rcVertexLength == 15))
        {
            // Case 1.1: Miss
            if (pathLength == 0)
            {
                if (isWeight0)
                {
                    return;
                }
                else
                {
                    print("Alert! Invalid reservoir case", 1);
                }
            }
            // Case 1.2: Hit but no RC
            else if(pathLength > 0)
            {
                return;
            }
            else
            {
                print("Alert! Invalid reservoir case", 2);
            }
        }
        else
        {
            print("Alert! Invalid reservoir case", 3);
        }
    }
    //
    // Case 2: DD RC
    //
    else if (isDD && !isDSD)
    {
        if (isRcValid && !isRcPrevValid && !isRcPrevPrevValid && (rcVertexLength > 0) && (pathLength > 0) && !isWeight0)
        {
            return;
        }
        else
        {
            print("Alert! Invalid reservoir case", 3);
        }
    }
    //
    // Case 3: DSD RC
    //
    else if (!isDD && isDSD)
    {
        if (isRcValid && isRcPrevValid && isRcPrevPrevValid && (rcVertexLength > 0) && (pathLength > 0) && !isWeight0)
        {
            return;
        }
        else
        {
            print("Alert! Invalid reservoir case", 4);
        }
    }
    //
    // Case 4: RC is both DD and DSD -- incorrect
    //
    else
    {
        print("Alert! Invalid reservoir case", 5);
    }

    // Check suspiciously high weight
    if (reservoir.weight > 10.f)
    {
        print("Alert! Reservoir heigh weight", 0);
    }

    print("The reservoir in question", 0);
    print("- rcVertexHit id:", reservoir.rcVertexHit.instanceID);
    print("- rcPrevVertexHit id:", reservoir.rcPrevVertexHit.instanceID);
    print("- rcPrevPrevVertexHit id:", reservoir.rcPrevPrevVertexHit.instanceID);
    print("- isDD", reservoir.pathFlags.decodeIsDD());
    print("- isDSD", reservoir.pathFlags.decodeIsDSD());
    print("- rcVertexLength", reservoir.pathFlags.rcVertexLength());
    print("- pathLength", reservoir.pathFlags.pathLength());
    print("- weight", reservoir.weight);
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

    print("{tracePath", 0);

    print("pixel", pixel);
    print("pathID", pathID);

    print("{traceLoop", 0);
    while (path.isActive())
    {
        print("{loop", 0);
        print("vertex", path.origin);
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
                print("}loop", 0);
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

        print("}loop", 0);
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
