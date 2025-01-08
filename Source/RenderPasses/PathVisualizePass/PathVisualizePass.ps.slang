/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/

import Scene.Scene;
import Scene.HitInfo;
import Utils.Debug.PixelDebug;
import Scene.Camera.Camera;
#include "Utils/Math/MathConstants.slangh"

SamplerState gPointSampler : register(s0);

cbuffer PerFrameCB : register(b0)
{
    uint2 gResolution;
    Camera gCamera;
    uint2 gSelectedPixel;
    uint gPathLenght;
}

Texture2D gInputImgTex;
Texture2D gDepthTex;
Texture2D<PackedHitInfo> gVBuffer;
Texture1D<float3> gPathVerticesTex;

static const uint MAX_PATH_VERTICES = 100;

// Todo: send in
//  - path verticies

// https://iquilezles.org/articles/
// https://www.shadertoy.com/view/wdXGDr
float sdCylinder(float3 p, float3 a, float3 b, float r)
{
    float3 ba = b - a;
    float3 pa = p - a;
    float baba = dot(ba, ba);
    float paba = dot(pa, ba);
    float x = length(pa * baba - ba * paba) - r * baba;
    float y = abs(paba - baba * 0.5) - baba * 0.5;
    float x2 = x * x;
    float y2 = y * y * baba;

    float d = (max(x, y) < 0.0) ? -min(x2, y2) : (((x > 0.0) ? x2 : 0.0) + ((y > 0.0) ? y2 : 0.0));

    return sign(d) * sqrt(abs(d)) / baba;
}

float map(in float3 pos)
{
    float tMin = FLT_MAX;
    float t = FLT_MAX;
    float radius = 0.005;
    float3 currVert;
    float3 nextVert;

    for (int i = 0; i < MAX_PATH_VERTICES; i++)
    {
        if (i > gPathLenght - 1)
        {
            break;
        }

        // Retrieve current path's vertex and the next one
        currVert = gPathVerticesTex[i];
        nextVert = gPathVerticesTex[i+1];

        t = sdCylinder(pos, currVert, nextVert, radius);

        if (tMin >= t)
            tMin = t;

    }
    return tMin;
}

float3 calcNormal(in float3 pos)
{
    float2 e = float2(1.0, -1.0) * 0.5773;
    const float eps = 0.0005;
    return normalize(e.xyy * map(pos + e.xyy * eps) +
                     e.yyx * map(pos + e.yyx * eps) +
                     e.yxy * map(pos + e.yxy * eps) +
                     e.xxx * map(pos + e.xxx * eps));
}

float4 main(float2 texC : TEXCOORD) : SV_TARGET0
{
    // Todo: figure out why printSetPixel didn't work
    // printSetPixel(gSelectedPixel);
    // printSetPixel(uint2(959, 782));
    // print("hello path viz", gSelectedPixel);
    // print("gResolution", gResolution);
    // print("Camera position", gCamera.getPosition());
    // print("scene light count", gScene.getLightCount());

    float4 color = gInputImgTex.Sample(gPointSampler, texC);
    float3 finalColor = color.rgb;
    // finalColor.g = 0;
    // finalColor.b = 1;

    // Compute view ray
    uint2 pixelCoord = uint2(texC * gResolution);
    const Ray viewRay = gCamera.computeRayPinhole(pixelCoord, gResolution);

    // Raymarch
    float t = 0.0f;
    // float tMax = 10.0f;
    for (int i = 0; i < 256; i++)
    {
        float3 pos = viewRay.origin + viewRay.tMin + viewRay.dir * t;
        float h = map(pos);

        if (h < 0.0001 || t > viewRay.tMax)
            break;

        t += h;
    }

    if (t < viewRay.tMax)
    {
        // Compute shading color
        float3 pos = viewRay.origin + viewRay.tMin + viewRay.dir * t;
        float3 nor = calcNormal(pos);
        float diffuse = clamp(dot(nor, float3(0.57703)), 0.0, 1.0);
        float ambient = 0.5 + 0.5 * dot(nor, float3(0.0, 1.0, 0.0));
        float3 shadeColor = float3(0.1, 0.1, 0.1) * ambient + float3(0.3, 0.3, 0.5) * diffuse;

        const HitInfo hit = HitInfo(gVBuffer[pixelCoord]);

        if (hit.isValid())
        {
            // Compare distace with scene depth
            float distToT = distance(pos, gCamera.getPosition());
            float distToSceneVert = viewRay.tMax + 1.0;

            HitType hitType = hit.getType();

            if (hitType == HitType::Triangle)
            {
                // Bug: using sceneVertex causes access violation error. Check if gVBuffer is handled correctly.

                const TriangleHit triangleHit = hit.getTriangleHit();
                VertexData sceneVertex = gScene.getVertexData(triangleHit);
                distToSceneVert = distance(sceneVertex.posW, gCamera.getPosition());
            }

            if (distToT < distToSceneVert)
            {
                finalColor = shadeColor;
            }
        }
        else
        {
            // Scene miss, no occlusion
            finalColor = shadeColor;
        }

    }


    return float4(finalColor, color.a);
}
