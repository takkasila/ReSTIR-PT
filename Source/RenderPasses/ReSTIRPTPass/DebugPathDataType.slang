#pragma once
#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

#define kMaxPathLength 15
#define kMaxVertices (kMaxPathLength+1)

struct DebugPathData
{
    uint vertexCount = 0;
    bool hasRCVertex = false;
    uint rcVertexIndex = 0;
    float4 L;
    float4 vertices[kMaxVertices];
    bool isSampledLight[kMaxVertices];
    float4 sampledLightPosition[kMaxVertices];
    float4 sampledLightRadiance[kMaxVertices];

};

struct DebugVertexState
{
    bool isSampledLight = false;
    float3 sampledLightPosition = float3(0, 0, 0);
    float3 sampledLightRadiance = float3(0, 0, 0);
};

END_NAMESPACE_FALCOR
