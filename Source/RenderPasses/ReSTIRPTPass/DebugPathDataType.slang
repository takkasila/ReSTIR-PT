#pragma once
#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

struct DebugPathData
{
    uint length = 0;
    bool hasRCVertex = false;
    uint rcVertexIndex = 0;
    float3 vertices[15];
};

END_NAMESPACE_FALCOR
