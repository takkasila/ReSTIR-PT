#pragma once
#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

#ifdef HOST_CODE
    // for memcpy
    #include <cstring>

    #define ALIGNAS(x) alignas(x)

#else
    #define ALIGNAS(x)

#endif

#define kMaxPathLength 15
#define kMaxVertices (kMaxPathLength+1)
#define kMaxManifoldIteration 20

struct DebugPathData
{
    uint vertexCount = 0;
    //  boolean padding:
    //      - CPU: 1 B
    //      - GPU: 4 B
    //  We format CPU's to match with GPU's so that we can interpret them in CPU correctly.
    ALIGNAS(4) bool hasRcVertex = false;
    ALIGNAS(4) bool isRcDSD = false;
    uint rcVertexIndex = 0;
    float4 L = float4(0, 0, 0, 0);
    float4 vertices[kMaxVertices];
    bool isSampledLight[kMaxVertices];
    float4 sampledLightPosition[kMaxVertices];
    float4 sampledLightRadiance[kMaxVertices];

#ifdef HOST_CODE
    void init()
    {
        vertexCount = 0;
        hasRcVertex = false;
        isRcDSD = false;
        rcVertexIndex = 0;
        L = float4(0, 0, 0, 0);

        float4 val = float4(0, 0, 0, 0);
        for (uint i = 0; i < 16; i++)
        {
            vertices[i] = val;
            isSampledLight[i] = false;
            sampledLightPosition[i] = val;
            sampledLightRadiance[i] = val;
        }
    }

    void deepCopy(DebugPathData srcDebugPathData)
    {
        vertexCount = srcDebugPathData.vertexCount;
        hasRcVertex = srcDebugPathData.hasRcVertex;
        isRcDSD = srcDebugPathData.isRcDSD;
        rcVertexIndex = srcDebugPathData.rcVertexIndex;
        L = srcDebugPathData.L;
        std::size_t floatArrSize = kMaxVertices * sizeof(float4);
        std::memcpy(vertices, srcDebugPathData.vertices, floatArrSize);
        std::memcpy(isSampledLight, srcDebugPathData.isSampledLight, kMaxVertices * sizeof(bool));
        std::memcpy(sampledLightPosition, srcDebugPathData.sampledLightPosition, floatArrSize);
        std::memcpy(sampledLightRadiance, srcDebugPathData.sampledLightRadiance, floatArrSize);
    }

#else
    [mutating] void init()
    {
        vertexCount = 0;
        hasRcVertex = false;
        isRcDSD = false;
        rcVertexIndex = 0;
        L = float4(0, 0, 0, 0);

        float4 val = float4(0, 0, 0, 0);
        for (uint i = 0; i < 16; i++)
        {
            vertices[i] = val;
            isSampledLight[i] = false;
            sampledLightPosition[i] = val;
            sampledLightRadiance[i] = val;
        }
    }
#endif

};

struct DebugVertexState
{
    float3 pos = float3(0, 0, 0);
    bool isSampledLight = false;
    float3 sampledLightPosition = float3(0, 0, 0);
    float3 sampledLightRadiance = float3(0, 0, 0);
};

struct DebugManifoldWalk
{
    float4 x1, x2, x3, target;
    float4 x2_i[kMaxManifoldIteration];
    float4 x3_i[kMaxManifoldIteration];
    int numIter = -1;   // Indicates that it has not been written into

#ifdef HOST_CODE
    void init()
    {
        float4 zero = float4(0, 0, 0, 0);
        x1 = x2 = x3 = target = zero;
        numIter = -1;

        for (uint i = 0; i < kMaxManifoldIteration; i++)
        {
            x2_i[i] = zero;
            x3_i[i] = zero;
        }
    }

    void deepCopy(DebugManifoldWalk srcDebugManifoldWalk)
    {
        x1 = srcDebugManifoldWalk.x1;
        x2 = srcDebugManifoldWalk.x2;
        x3 = srcDebugManifoldWalk.x3;
        target = srcDebugManifoldWalk.target;
        std::memcpy(x2_i, srcDebugManifoldWalk.x2_i, kMaxManifoldIteration * sizeof(float4));
        std::memcpy(x3_i, srcDebugManifoldWalk.x3_i, kMaxManifoldIteration * sizeof(float4));
        numIter = srcDebugManifoldWalk.numIter;
    }

#else
    [mutating] void init()
    {
        float4 zero = float4(0, 0, 0, 0);
        x1 = x2 = x3 = target = zero;
        numIter = -1;

        for (uint i = 0; i < kMaxManifoldIteration; i++)
        {
            x2_i[i] = zero;
            x3_i[i] = zero;
        }
    }
#endif
};

END_NAMESPACE_FALCOR
