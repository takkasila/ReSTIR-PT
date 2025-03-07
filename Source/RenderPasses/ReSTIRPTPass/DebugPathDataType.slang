#pragma once
#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

#ifdef HOST_CODE
    // for memcpy
    #include <cstring>
#endif

#define kMaxPathLength 15
#define kMaxVertices (kMaxPathLength+1)

struct DebugPathData
{
    uint vertexCount = 0;
    bool hasRCVertex = false;
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
        hasRCVertex = false;
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
        hasRCVertex = srcDebugPathData.hasRCVertex;
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
        hasRCVertex = false;
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
    bool isSampledLight = false;
    float3 sampledLightPosition = float3(0, 0, 0);
    float3 sampledLightRadiance = float3(0, 0, 0);
};

END_NAMESPACE_FALCOR
