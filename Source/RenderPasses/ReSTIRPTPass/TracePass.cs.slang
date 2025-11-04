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
        // Case 1.1: Miss
        if (isWeight0)
        {
            if (!isRcValid && isRcPrevValid && isRcPrevPrevValid && (rcVertexLength == 15) && (pathLength == 0))
            {
                return;
            }
            else
            {
                print("Alert! Invalid reservoir case", 1);
            }
        }
        // Case 1.2: Hit but no RC
        else
        {
            if (!isRcValid && !isRcPrevValid && !isRcPrevPrevValid && (rcVertexLength == 15) && (pathLength > 0))
            {
                return;
            }
            else
            {
                print("Alert! Invalid reservoir case", 2);
            }
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

    print("The reservoir in question:", 0);
    print("- rcVertexHit id:", reservoir.rcVertexHit.instanceID);
    print("- rcPrevVertexHit id:", reservoir.rcPrevVertexHit.instanceID);
    print("- rcPrevPrevVertexHit id:", reservoir.rcPrevPrevVertexHit.instanceID);
    print("- isDD", reservoir.pathFlags.decodeIsDD());
    print("- isDSD", reservoir.pathFlags.decodeIsDSD());
    print("- rcVertexLength", reservoir.pathFlags.rcVertexLength());
    print("- pathLength", reservoir.pathFlags.pathLength());
    print("- weight", reservoir.weight);
}

void computeNormalDerivatives(HitInfo hit, float3 viewDir, out float3 dndu, out float3 dndv)
{
    TriangleHit triHit = hit.getTriangleHit();

    float3 baryCentricWeights = triHit.getBarycentricWeights();
    print("triHit.instanceID.index", triHit.instanceID.index);
    print("triHit.primitiveIndex", triHit.primitiveIndex);
    print("triHit.baryCentricWeights", baryCentricWeights);

    float curveSphereRadius = {};
    uint materialID = {};

    VertexData v = loadVertexData(hit, viewDir, materialID, curveSphereRadius);

    print("v.posW", v.posW);
    print("v.normalW", v.normalW);
    print("v.tangentW", v.tangentW);
    print("v.faceNormalW", v.faceNormalW);
    print("v.texC", v.texC);
    print("v.coneTexLODValue", v.coneTexLODValue);

    // Get 3 corner vertices of the triangle
    const uint3 vtxIndices = gScene.getIndices(triHit.instanceID, triHit.primitiveIndex);
    StaticVertexData vertices[3] = {
        gScene.getVertex(vtxIndices[0]),
        gScene.getVertex(vtxIndices[1]),
        gScene.getVertex(vtxIndices[2])
    };

    const float4x4 worldMat = gScene.getWorldMatrix(triHit.instanceID);
    const float3x3 worldInvTransposeMat = gScene.getInverseTransposeWorldMatrix(triHit.instanceID);
    const bool isFrontFaceCW = gScene.isObjectFrontFaceCW(triHit.instanceID);

    print("vertices[0].position", vertices[0].position);
    print("vertices[0].normal", vertices[0].normal);
    print("vertices[0].tangent", vertices[0].tangent);
    print("vertices[0].texCrd", vertices[0].texCrd);
    print("", 0);

    print("vertices[1].position", vertices[1].position);
    print("vertices[1].normal", vertices[1].normal);
    print("vertices[1].tangent", vertices[1].tangent);
    print("vertices[1].texCrd", vertices[1].texCrd);
    print("", 1);

    print("vertices[2].position", vertices[2].position);
    print("vertices[2].normal", vertices[2].normal);
    print("vertices[2].tangent", vertices[2].tangent);
    print("vertices[2].texCrd", vertices[2].texCrd);
    print("", 2);

    float3 N_u = vertices[1].normal - vertices[0].normal;
    float3 N_v = vertices[2].normal - vertices[0].normal;

    float3x3 I3 = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
    };

    matrix<float, 3, 1>  n = { v.normalW.x, v.normalW.y, v.normalW.z };
    matrix<float, 1, 3>  nT = { v.normalW.x, v.normalW.y, v.normalW.z };

    float3x3 projection = I3 - mul(n, nT);

    print("p", projection[0]);
    print("p", projection[1]);
    print("p", projection[2]);

    float3 n_u = mul(projection, N_u);
    float3 n_v = mul(projection, N_v);

    print("n_u", n_u);
    print("n_v", n_v);

    dndu = n_u;
    dndv = n_v;

    // Or do we actually want dndt, dndb?
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

    print("pixel", pixel);
    print("pathID", pathID);

    if (path.isActive() && isAtDebugPixel)
    {
        float3 dndu, dndv;
        computeNormalDerivatives(path.hit, path.dir, dndu, dndv);
    }

    while (path.isActive())
    {
        print("vertex", path.origin);
        print("path.length", path.length);
        print("path.isSpecularBounce", path.isSpecularBounce());

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

        print("- in loop path.pathReservoir.rcVertexHit.instanceID", path.pathReservoir.rcVertexHit.instanceID);

        print("", 0);   // empty line
    }

    print("Path Radiance", path.L);

    gPathTracer.finalize(path);

    gPathTracer.writeOutput(path, giReservoir, sampleIdx);
    // giReservoir is just a temporary middle man.
    // The final output is in gPathTracer.outputReservoirs[reservoirIdx]

    const uint reservoirIdx = gPathTracer.params.getReservoirOffset(pixel);

    const PathReservoir outputReservoirs = gPathTracer.outputReservoirs[reservoirIdx];
    const float4 outputColor = gPathTracer.outputColor[reservoirIdx];

    print("Output color", outputColor);

    print("Output reservoir:", 0);
    print("- rcVertexHit id:", outputReservoirs.rcVertexHit.instanceID);
    print("- rcPrevVertexHit id:", outputReservoirs.rcPrevVertexHit.instanceID);
    print("- rcPrevPrevVertexHit id:", outputReservoirs.rcPrevPrevVertexHit.instanceID);
    print("- isDD", outputReservoirs.pathFlags.decodeIsDD());
    print("- isDSD", outputReservoirs.pathFlags.decodeIsDSD());
    print("- rcVertexLength", outputReservoirs.pathFlags.rcVertexLength());
    print("- pathLength", outputReservoirs.pathFlags.pathLength());
    print("- weight", outputReservoirs.weight);
    print("- RcSpecularBounce", outputReservoirs.pathFlags.decodeIsSpecularBounce(false));
    print("- prevRcSpecularBounce", outputReservoirs.pathFlags.decodeIsSpecularBounce(true));

    if (isAtDebugPixel)
    {
        // Check Reservoir validity
        checkReservoirValidity(outputReservoirs);

        // Fill in debutPathData
        pixelDebugPathDataBuffer[0].vertexCount = vertexCount;

        pixelDebugPathDataBuffer[0].L = float4(path.L, 1);

        //  Check if have RC vertex
        HitInfo rcVertexHitInfo = outputReservoirs.rcVertexHit.getHitInfo();
        HitInfo rcPrevVertexHitInfo = outputReservoirs.rcPrevVertexHit.getHitInfo();
        HitInfo rcPrevPrevVertexHitInfo = outputReservoirs.rcPrevPrevVertexHit.getHitInfo();

        print("rcVertexHitInfo", rcVertexHitInfo.isValid());
        print("rcPrevVertexHitInfo", rcPrevVertexHitInfo.isValid());
        print("rcPrevPrevVertexHitInfo", rcPrevPrevVertexHitInfo.isValid());

        if (!rcVertexHitInfo.isValid())
        {
            print("Doesn't have RC vertex", 0);
            pixelDebugPathDataBuffer[0].hasRcVertex = false;
        }
        else
        {
            bool isDD = outputReservoirs.pathFlags.decodeIsDD();
            bool isDSD = outputReservoirs.pathFlags.decodeIsDSD();

            if(!rcPrevVertexHitInfo.isValid() && !rcPrevPrevVertexHitInfo.isValid() && isDD)
            {
                print("RC is DD", 0);
                pixelDebugPathDataBuffer[0].hasRcVertex = true;
                pixelDebugPathDataBuffer[0].isRcDSD = false;
                pixelDebugPathDataBuffer[0].rcVertexIndex = outputReservoirs.pathFlags.rcVertexLength() + 1;
            }
            else if(rcPrevVertexHitInfo.isValid() && rcPrevPrevVertexHitInfo.isValid() && isDSD)
            {
                print("RC is DSD", 0);
                pixelDebugPathDataBuffer[0].hasRcVertex = true;
                pixelDebugPathDataBuffer[0].isRcDSD = true;
                pixelDebugPathDataBuffer[0].rcVertexIndex = outputReservoirs.pathFlags.rcVertexLength() + 1;
            }
            else
            {
                print("Something is wrong in TracePass!", 0);
            }
        }
    }

    bool isDelta1 = outputReservoirs.pathFlags.decodeIsDeltaEvent(true);
    bool isDelta2 = outputReservoirs.pathFlags.decodeIsDeltaEvent(false);

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
