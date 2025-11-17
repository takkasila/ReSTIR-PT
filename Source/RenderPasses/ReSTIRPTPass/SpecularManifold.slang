#include "Utils/Math/MathConstants.slangh"
import PathReservoir;
import LoadShadingData;
import Params;
import Utils.Debug.PixelDebug;
import Utils.Math.MathHelpers;
import Scene.Scene;


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


void computeSpecularConstraintDerivatives(ShadingData x1, ShadingData x2, ShadingData x3, float3 dndu, float3 dndv, out float2x2 gradC[3])
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
}

float computeDSDShiftJacobian(ShadingData x1, ShadingData x2, ShadingData x3, float3 dndu, float3 dndv)
{
    // Compute Gradient C
    float2x2 gradC[3];

    computeSpecularConstraintDerivatives(x1, x2, x3, dndu, dndv, gradC);

    // Compute G'(x1 <-> x2)
    float3 wi = x1.posW - x2.posW;
    float3 wo = x3.posW - x2.posW;
    float lwi = length(wi);
    float ili = 1 / lwi;
    float ilo = 1 / length(wo);
    wi *= ili;
    wo *= ilo;

    float cosThetaX2 = dot(x2.N, wi);
    float g_x1x2 = cosThetaX2 / (lwi * lwi);

    // Compute Jacobian as G'(x1 <-> x2) * |A^-1 * B3|
    float detAInv = determinant(inverse(gradC[1]));
    float detB3 = determinant(gradC[2]);

    print("-------- g_x1x2", g_x1x2);
    print("-------- detAInv", detAInv);
    print("-------- detB3", detB3);

    float jacobian = g_x1x2 * detAInv * detB3;
    return jacobian;
}


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

void walkManifoldDSD(
    ShadingData x1,
    ShadingData x2,
    ShadingData x3,
    ShadingData target,
    float3 x2_dndu,
    float3 x2_dndv,
    out ShadingData x2_new
)
{
    print("----- walkManifoldDSD: begin()", 0);
    // TODO:

    print("----- walkManifoldDSD: end()", 0);
}