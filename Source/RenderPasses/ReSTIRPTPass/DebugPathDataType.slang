#pragma once
#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

struct DebugPathData
{
    uint length = 0;
    bool hasRCVertex = false;
    uint rcVertexIndex = 0;
    float3 L;
    float3 vertices[15];
    bool isSampledLight[15];
    float3 sampledLightPosition[15];

};

struct DebugVertexState
{
    bool isSampledLight = false;
    float3 sampledLightPosition = float3(0,0,0);
};

END_NAMESPACE_FALCOR
