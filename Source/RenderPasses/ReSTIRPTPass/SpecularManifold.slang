#include "Utils/Math/MathConstants.slangh"
import Params;
import PathState;
import PathReservoir;
import LoadShadingData;
import Utils.Debug.PixelDebug;
import Utils.Math.MathHelpers;
import Scene.Scene;
import Scene.RaytracingInline;
import Rendering.Utils.PixelStats;
import Rendering.Materials.MaterialShading;

void computeNormalDerivatives(HitInfo hit, float3 viewDir, out float3 dndu, out float3 dndv)
{
    TriangleHit triHit = hit.getTriangleHit();

    float curveSphereRadius = {};
    uint materialID = {};
    VertexData v = loadVertexData(hit, viewDir, materialID, curveSphereRadius);
    // print("------ v.normalW", v.normalW);

    // Get 3 corner vertices of the triangle
    const uint3 vtxIndices = gScene.getIndices(triHit.instanceID, triHit.primitiveIndex);
    StaticVertexData vertices[3] = {
        gScene.getVertex(vtxIndices[0]),
        gScene.getVertex(vtxIndices[1]),
        gScene.getVertex(vtxIndices[2])
    };

    // print("------ vertices[0].normal", vertices[0].normal);
    // print("------ vertices[1].normal", vertices[1].normal);
    // print("------ vertices[2].normal", vertices[2].normal);

    float N_dot_N = dot(v.normalW, v.normalW);

    float3x3 I3 = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
    };

    matrix<float, 3, 1> n = { v.normalW.x, v.normalW.y, v.normalW.z };
    matrix<float, 1, 3> nT = { v.normalW.x, v.normalW.y, v.normalW.z };

    float3x3 term = (I3 - (mul(n, nT) / N_dot_N)) / sqrt(N_dot_N);

    // print("------ Matrix:", 0);
    // print("------- ", term[0]);
    // print("------- ", term[1]);
    // print("------- ", term[2]);

    float3 N_u = vertices[1].normal - vertices[0].normal;
    float3 N_v = vertices[2].normal - vertices[0].normal;

    dndu = mul(term, N_u);
    dndv = mul(term, N_v);
}


// x1 closer to light source
// x3 closer to eye
bool computeSpecularConstraintDerivatives(ShadingData x1, ShadingData x2, ShadingData x3, float3 dndu, float3 dndv, out float2x2 gradC[3])
{
    // Compute relevant directions and a few useful projections
    // Actually, is wi and wo swapped in this case?
    // Yes, just swap the name when declaring
    float3 wi = x1.posW - x2.posW;
    float3 wo = x3.posW - x2.posW;
    float lwi = length(wi);
    float ili = 1 / lwi;
    float ilo = 1 / length(wo);
    wi *= ili;
    wo *= ilo;

    // Half vector
    float3 H = wi + x2.eta * wo;
    float ilh = 1 / length(H);
    H *= ilh;

    float dot_H_n = dot(x2.N, H);
    float dot_H_dndu = dot(dndu, H);
    float dot_H_dndv = dot(dndv, H);
    float dot_u_n = dot(x2.T, x2.N);
    float dot_v_n = dot(x2.B, x2.N);

    // Local shading tangent frame
    float3 s = x2.T - dot_u_n * x2.N;
    float3 t = x2.B - dot_v_n * x2.N;

    ilo *= x2.eta * ilh;
    ili *= ilh;

    // Derivatives of C with respect to x1
    float3 dH_du = (x1.T - wi * dot(wi, x1.T)) * ili;
    float3 dH_dv = (x1.B - wi * dot(wi, x1.B)) * ili;

    dH_du -= H * dot(dH_du, H);
    dH_dv -= H * dot(dH_dv, H);

    float2x2 B1 = float2x2(
        dot(dH_du, s), dot(dH_dv, s),
        dot(dH_du, t), dot(dH_dv, t)
    );

    print("-------- B1:", 0);
    print("--------- ", B1[0]);
    print("--------- ", B1[1]);

    // Derivatives of C with respect to x2
    dH_du = -x2.T * (ili + ilo)
            + wi * (dot(wi, x2.T) * ili)
            + wo * (dot(wo, x2.T) * ilo);
    dH_dv = -x2.B * (ili + ilo)
            + wi * (dot(wi, x2.B) * ili)
            + wo * (dot(wo, x2.B) * ilo);

    dH_du -= H * dot(dH_du, H);
    dH_dv -= H * dot(dH_dv, H);

    float2x2 B2 = float2x2(
        dot(dH_du, s) - dot(x2.T, dndu) * dot_H_n - dot_u_n * dot_H_dndu,
        dot(dH_dv, s) - dot(x2.T, dndv) * dot_H_n - dot_u_n * dot_H_dndv,
        dot(dH_du, t) - dot(x2.B, dndu) * dot_H_n - dot_v_n * dot_H_dndu,
        dot(dH_dv, t) - dot(x2.B, dndv) * dot_H_n - dot_v_n * dot_H_dndv
    );

    print("-------- B2, A:", 0);
    print("--------- ", B2[0]);
    print("--------- ", B2[1]);

    // Derivatives of C with respect to x3
    dH_du = (x3.T - wo * dot(wo, x3.T)) * ilo;
    dH_dv = (x3.B - wo * dot(wo, x3.B)) * ilo;

    dH_du -= H * dot(dH_du, H);
    dH_dv -= H * dot(dH_dv, H);

    float2x2 B3 = float2x2(
        dot(dH_du, s), dot(dH_dv, s),
        dot(dH_du, t), dot(dH_dv, t)
    );

    print("-------- B3:", 0);
    print("--------- ", B3[0]);
    print("--------- ", B3[1]);

    gradC[0] = B1;
    gradC[1] = B2;
    gradC[2] = B3;

    // Check validity
    bool isValid = any(isnan(gradC[0])) || any(isnan(gradC[0])) || any(isnan(gradC[0]));
    return !isValid;
}

// x1 closer to light source
// x3 closer to eye
float computeDSDShiftJacobian(ShadingData x1, ShadingData x2, ShadingData x3, float3 dndu, float3 dndv)
{
    // Compute Gradient C
    float2x2 gradC[3];
    if (!computeSpecularConstraintDerivatives(x1, x2, x3, dndu, dndv, gradC))
    {
        print("-------- Invalid gradC", 0);
        return 0;
    }
    // NOTE: gradC[0] is respective toward x3

    // Compute G'(x2 <-> x3)
    float3 wi = x1.posW - x2.posW;
    float3 wo = x3.posW - x2.posW;
    float lwi = length(wi);
    float ili = 1 / lwi;
    float lwo = length(wo);
    float ilo = 1 / lwo;
    wi *= ili;
    wo *= ilo;

    float g_x1x2 = dot(x2.N, wo) / (lwo * lwo);

    // Compute Jacobian as G'(x2 <-> x3) * |B2^-1 * B1|
    float detB2Inv = determinant(inverse(gradC[1]));
    float detB1 = determinant(gradC[0]);

    print("-------- g_x1x2", g_x1x2);
    print("-------- detB2Inv", detB2Inv);
    print("-------- detB1", detB1);

    float jacobian = g_x1x2 * detB2Inv * detB1;
    return jacobian;
}

// x1 closer to light source
// x3 closer to eye
float computeReconnectionDSDShiftJacobian(
    ShadingData srcX1,
    ShadingData srcX2,
    ShadingData srcX3,
    float3 srcX2DnDu,
    float3 srcX2DnDv,
    ShadingData dstX1,
    ShadingData dstX2,
    ShadingData dstX3,
    float3 dstX2DnDu,
    float3 dstX2DnDv,
)
{
    float dstJacobian = computeDSDShiftJacobian(dstX1, dstX2, dstX3, dstX2DnDu, dstX2DnDv);

    float srcJacobian = computeDSDShiftJacobian(srcX1, srcX2, srcX3, srcX2DnDu, srcX2DnDv);

    float Jacobian = dstJacobian / srcJacobian;

    print("------- Jacobian DSD shift dst:", dstJacobian);
    print("------- Jacobian DSD shift src:", srcJacobian);
    print("------- Jacobian Reconnection DSD shift:", Jacobian);

    return Jacobian;
}

// x1 closer to light source
// x3 closer to eye
bool walkManifoldDSD(
    const ShadingData x1_in,
    const HitInfo x2Hit_in,
    const ShadingData x3_in,
    const ShadingData target,
    out ShadingData x2_new
)
{
    print("----- walkManifoldDSD: begin()", 0);

    float beta = 1.f;
    // threshold = self.eps_threshold()
    float3 x3ToTarget;
    float endPointDistance;

    //  TODO: Get SCENE_LENGTH from somewhere?
    const float SCENE_LENGTH = 1.f;
    const uint MAX_ITER = 20;
    const float EPS = 1e-05F; // or FLT_EPSILON

    float2x2 gradC[3];

    HitInfo x2Hit = x2Hit_in;
    ShadingData x2 = loadShadingDataWithPrevVertexPosition(x2Hit, x3_in.posW, false);
    float3 x2_dndu;
    float3 x2_dndv;

    HitInfo x3Hit = {};
    ShadingData x3 = x3_in;

    for (uint i = 0; i < MAX_ITER; i++)
    {
        print("----- Round", i);

        // Check if close enough
        x3ToTarget = target.posW - x3.posW;
        endPointDistance = length(x3ToTarget);

        print("------ End point distance", endPointDistance);

        if (endPointDistance <= SCENE_LENGTH * EPS)
        {
            print("------ stop: close enough", 0);
            break;
        }

        // Prep ingredients
        computeNormalDerivatives(x2Hit, x2.V, x2_dndu, x2_dndv);
        if (!computeSpecularConstraintDerivatives(x1_in, x2, x3, x2_dndu, x2_dndv, gradC))
        {
            print("------ invalid gradC", 0);
            return false;
        }

        print("------ x2.posW", x2.posW);

        // Contruct tangent plane at X3
        matrix<float, 2, 3> TX3_transpose = {
            x3.T.x, x3.T.y, x3.T.z,
            x3.B.x, x3.B.y, x3.B.z,
        };

        // Project distancement to tangent plane
        float2 projectedDisplacement = mul(TX3_transpose, x3ToTarget);

        print("------ projectedDisplacement", projectedDisplacement);

        // Displacement of x2 is A^-1 * B3
        float2x2 displacementX3ToX2 = inverse(gradC[1]) * gradC[2];
        float2 displacementX2 = mul(displacementX3ToX2, projectedDisplacement);

        print("------ displacementX2", displacementX2);

        // P2
        float2x2 P2 = {
            1.0f, 0,
            0, 1.0f
        };
        P2 = -P2; // Or is it?

        // Project the displacement onto second vertex tangent space
        float2 displacementX2ProjectedX2 = mul(P2, displacementX2);
        print("------ displacementX2ProjectedX2", displacementX2ProjectedX2);

        // Construct tangent plane at X2
        matrix<float, 3, 2> TX2 = {
            x2.T.x, x2.B.x,
            x2.T.y, x2.B.y,
            x2.T.z, x2.B.z,
        };

        // Transform displacement on tangent plane to world space
        float3 displacementWorld = mul(TX2, displacementX2ProjectedX2);
        print("------ displacementWorld", displacementWorld);

        // Apply the step size
        displacementWorld *= beta;
        print("------ displacementWorld with step size", displacementWorld);

        // Extrapolate position
        float3 x2_new = x2.posW - displacementWorld;
        print("------ x2_new", x2_new);

        // Trace ray to update new positions
        {
            // Trace ray x1 -> x2
            float3 rayDir = normalize(x2_new - x1_in.posW);
            RayDesc ray = { x1_in.posW, 0.f, rayDir, kRayTMax };

            float hitT;
            SceneRayQuery<kUseAlphaTest> sceneRayQuery;
            logTraceRay(PixelStatsRayType::ClosestHit);

            // Trace ray
            if (!sceneRayQuery.traceRay(ray, x2Hit, hitT, RAY_FLAG_NONE, 0xff))
            {
                print("------ Unsuccessful trace x1 -> x2", 0);
                return false;
            }

            // Valid hit
            print("------ Successful trace x1 -> x2", 0);

            // Trace ray x2 -> x3
            x2 = loadShadingDataWithPrevVertexPosition(x2Hit, x1_in.posW, false);
            print("Updated x2", x2.posW);

            rayDir = normalize(x3.posW - x2.posW);
            ray = { x2.posW, 0.f, rayDir, kRayTMax };

            logTraceRay(PixelStatsRayType::ClosestHit);

            // Trace ray
            if (!sceneRayQuery.traceRay(ray, x3Hit, hitT, RAY_FLAG_NONE, 0xff))
            {
                print("------ Unsuccessful trace x2 -> x3", 0);
                return false;
            }

            // Valid hit
            print("------ Successful trace x2 -> x3", 0);

            x3 = loadShadingDataWithPrevVertexPosition(x3Hit, x2.posW, false);
            print("Updated x3", x3.posW);
        }

    }

    print("----- walkManifoldDSD: end()", 0);
    return true;
}
