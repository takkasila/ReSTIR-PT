/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/

#include "Utils/Math/MathConstants.slangh"

import PathReservoir;
import Utils.Sampling.SampleGenerator;
import LoadShadingData;
import Params;
import Utils.Debug.PixelDebug;
import Rendering.Materials.MaterialShading;
import Rendering.Utils.PixelStats;
import Scene.RaytracingInline;
import Scene.Scene;
import PathTracer;
import RenderPasses.Shared.Denoising.NRDData;
import DebugPathDataType;

ParameterBlock<PathTracer> gPathTracer;

float3 computeShiftedIntegrandExtraOutput(
    const RestirPathTracerParams params, inout float dstJacobian, inout float output2, const PackedHitInfo dstPrimaryHitPacked, const ShadingData dstPrimarySd,
    const ShadingData srcPrimarySd, const PathReservoir srcReservoir, ReconnectionData rcData, bool evalVisibility, bool usePrev = false, bool temporalUpdateForDynamicScene = false)
{
    PathReservoir tempReservoir = srcReservoir;
    float3 res = computeShiftedIntegrand_(params, dstJacobian, dstPrimaryHitPacked, dstPrimarySd,
        srcPrimarySd, tempReservoir, rcData, evalVisibility, usePrev, temporalUpdateForDynamicScene);
    if (PathSamplingMode(kPathSamplingMode) == PathSamplingMode::PathReuse) // for precomputing MIS weights
    {
        dstJacobian = tempReservoir.cachedJacobian[0];
        output2 = tempReservoir.cachedJacobian[1];
    }
    return res;
}

float3 computeShiftedIntegrand(
    const RestirPathTracerParams params, inout float dstJacobian, const PackedHitInfo dstPrimaryHitPacked, const ShadingData dstPrimarySd,
    const ShadingData srcPrimarySd, const PathReservoir srcReservoir, ReconnectionData rcData, bool evalVisibility, bool usePrev = false, bool temporalUpdateForDynamicScene = false)
{
    PathReservoir tempReservoir = srcReservoir;
    float3 res = computeShiftedIntegrand_(params, dstJacobian, dstPrimaryHitPacked, dstPrimarySd,
        srcPrimarySd, tempReservoir, rcData, evalVisibility, usePrev, temporalUpdateForDynamicScene);
    if (PathSamplingMode(kPathSamplingMode) == PathSamplingMode::PathReuse) // for precomputing MIS weights
    {
        dstJacobian = tempReservoir.cachedJacobian[0];
    }
    return res;
}

float3 computeShiftedIntegrand_(const RestirPathTracerParams params, inout float dstJacobian, const PackedHitInfo dstPrimaryHitPacked, const ShadingData dstPrimarySd,
    const ShadingData srcPrimarySd, inout PathReservoir srcReservoir, ReconnectionData rcData, bool evalVisibility, bool usePrev = false, bool temporalUpdateForDynamicScene = false)
{
    dstJacobian = 0.f;

    if (srcReservoir.weight == 0.f) return 0.f;

    if (ShiftMapping(kShiftStrategy) == ShiftMapping::Reconnection)
    {
#if BPR
        float3 res = computeShiftedIntegrandReconnectionPathTree(params, dstJacobian, dstPrimarySd, srcPrimarySd, srcReservoir, evalVisibility);
        if (PathSamplingMode(kPathSamplingMode) == PathSamplingMode::PathReuse && all(res == 0)) // Nan, inf
        {
            srcReservoir.cachedJacobian[0] = 0;
            srcReservoir.cachedJacobian[1] = 0;
        }
        return res;
#else

        if (temporalUpdateForDynamicScene && kTemporalUpdateForDynamicScene)
        {
            gPathTracer.traceTemporalUpdate(dstPrimarySd, srcReservoir);
        }

        return computeShiftedIntegrandReconnection(params, dstJacobian, dstPrimarySd, srcPrimarySd, srcReservoir, evalVisibility);
#endif
    }
    else if (ShiftMapping(kShiftStrategy) == ShiftMapping::RandomReplay)
    {
        return computeShiftedIntegrandRandomReplay(params, usePrev, dstJacobian, dstPrimaryHitPacked, dstPrimarySd, srcPrimarySd, srcReservoir);
    }
    else if (ShiftMapping(kShiftStrategy) == ShiftMapping::Hybrid)
    {
        //  dst: dst
        //  src: src
        print("--- rcData.rcPrevHit.isValid()", rcData.rcPrevHit.isValid());
        print("--- rcData.pathThroughput", rcData.pathThroughput);

        float3 tmp = computeShiftedIntegrandHybrid(params, usePrev, temporalUpdateForDynamicScene, dstJacobian, dstPrimaryHitPacked, dstPrimarySd, srcPrimarySd, srcReservoir, rcData, evalVisibility);

        print("--- integrand", tmp);

        return tmp;
    }
    else if (ShiftMapping(kShiftStrategy) == ShiftMapping::HybridShiftSMS)
    {

        print("--- rcData.rcPrevPrevHit.isValid()", rcData.rcPrevPrevHit.isValid());
        print("--- rcData.rcPrevHit.isValid()", rcData.rcPrevHit.isValid());
        print("--- rcData.pathThroughput", rcData.pathThroughput);

        bool isDD = srcReservoir.pathFlags.decodeIsDD();
        bool isDSD = srcReservoir.pathFlags.decodeIsDSD();

        float3 tmp = 0.f;

        if(isDSD)
        {
            print("--- Doing DSD RC in HybridShiftSMS: UNDERCONSTRUCTION", 0);
            // TODO:
        }
        else
        {
            print("--- Doing DD RC in HybridShiftSMS", 0);

            // DD reconnection
            tmp = computeShiftedIntegrandHybrid(params, usePrev, temporalUpdateForDynamicScene, dstJacobian, dstPrimaryHitPacked, dstPrimarySd, srcPrimarySd, srcReservoir, rcData, evalVisibility);

            print("--- integrand", tmp);

        }

        return tmp;
    }

    return 1.f;
}


// Doing basic random replay
float3 computeShiftedIntegrandRandomReplay(const RestirPathTracerParams params, bool usePrev, inout float dstJacobian, const PackedHitInfo dstPrimaryHitPacked, ShadingData dstPrimarySd, ShadingData srcPrimarySd,
    inout PathReservoir tempPathReservoir)
{
    dstJacobian = 0.f;

    if (tempPathReservoir.weight == 0.f) return 0.f;

    dstJacobian = 1.f;
    PathReservoir srcReservoir = tempPathReservoir;

    HitInfo dstPrimaryHit; dstPrimaryHit.unpack(dstPrimaryHitPacked);
    Ray ray = Ray(gScene.camera.getPosition(usePrev), -dstPrimarySd.V, 0, kRayTMax);

    float3 L = gPathTracer.traceRandomReplayPath(dstPrimaryHit, dstPrimarySd, srcPrimarySd, ray, tempPathReservoir);

    // these members are overriden in the random replay, fill them back
    tempPathReservoir.M = srcReservoir.M;
    tempPathReservoir.weight = srcReservoir.weight;
    tempPathReservoir.rcRandomSeed = srcReservoir.rcRandomSeed;
    tempPathReservoir.initRandomSeed = srcReservoir.initRandomSeed;

    if (isnan(dstJacobian) || isinf(dstJacobian)) dstJacobian = 0.f;

    return L;
}

float3 computeShiftedIntegrandHybrid(const RestirPathTracerParams params, bool usePrev, bool temporalUpdateForDynamicScene, inout float dstJacobian, const PackedHitInfo dstPrimaryHitPacked, const ShadingData dstPrimarySd, const ShadingData srcPrimarySd,
    inout PathReservoir tempPathReservoir, ReconnectionData rcData, bool evalVisibility)
{
    dstJacobian = 0.f;

    if (tempPathReservoir.weight == 0.f)
    {
        print("---- empty reservoir", 0);
        return 0.f;
    }

    dstJacobian = 1.f;

    HitInfo dstPrimaryHit;
    dstPrimaryHit.unpack(dstPrimaryHitPacked);
    Ray ray = Ray(gScene.camera.getPosition(usePrev), -dstPrimarySd.V, 0, kRayTMax);
    bool isRcVertexEscapedVertex = tempPathReservoir.pathFlags.pathLength() + 1 == tempPathReservoir.pathFlags.rcVertexLength();

    PathReservoir srcReservoir = tempPathReservoir;

    HitInfo dstRcPrevVertexHit;
    float3 dstRcPrevVertexWo;
    float3 Tp;

    if (srcReservoir.pathFlags.rcVertexLength() == 1)
    {
        print("---- case RC == v2", 0);

        // Dummy RC comes here

        Tp = 1.f;
        dstRcPrevVertexHit.setValid();
        dstRcPrevVertexWo = 1.f; // this value doesn't matter, as long as it is not all 0

        if (params.localStrategyType & (uint)LocalStrategy::RoughnessCondition)
        {
            bool isSpecularBounce = srcReservoir.pathFlags.decodeIsSpecularBounce(true);

            bool isLastVertexClassifiedAsRough = kSeparatePathBSDF ?
                (isSpecularBounce ? dstPrimarySd.linearRoughness > params.specularRoughnessThreshold : hasRoughComponent(dstPrimarySd, 1.f)) :
                classifyAsRough(dstPrimarySd, params.specularRoughnessThreshold);

            if (!isLastVertexClassifiedAsRough)
            {
                // RcPrevVertex is specular. Invalidate
                print("----- Invalidate shift", 0);
                Tp = 0.f;
            }
        }
    }
    else
    {
        print("---- case RC >= v3", 0);

        dstRcPrevVertexHit = rcData.rcPrevHit;
        dstRcPrevVertexWo = rcData.rcPrevWo;
        Tp = rcData.pathThroughput;

        if (all(Tp == float3(1.0f)))
        {
            print("----- Something is wrong", 0);
            print("----- dstRcPrevVertexHit", dstRcPrevVertexHit.isValid());
            print("----- dstRcPrevVertexWo", dstRcPrevVertexWo);
            print("----- Tp", Tp);
        }
    }

    float3 rcTp = 1.f;

    // the reconnection vertex exists
    if (
        any(Tp > 0.f)
        && (
            dstRcPrevVertexHit.isValid()
            && (srcReservoir.pathFlags.rcVertexLength() <= srcReservoir.pathFlags.pathLength() || isRcVertexEscapedVertex)
        )
    )
    {

        if (all(dstRcPrevVertexWo == 0.f))
        {
            print("---- Exit because dstRcPrevVertexWo is 0.f (dummyRC)", 0);
            // Invalid shift
            return 0.f;
        }
        else
        {
            // Test
            // return 0.f;

            print("---- Computing shifted integrand hybrid", 0);


            // Valid shift. Compute shifted integrand
            ShadingData dstRcPrevVertexSd = dstPrimarySd;
            ShadingData srcRcPrevVertexSd = srcPrimarySd;

            if (srcReservoir.pathFlags.rcVertexLength() > 1)
            {
                dstRcPrevVertexSd = loadShadingData(dstRcPrevVertexHit, dstRcPrevVertexWo, srcReservoir.pathFlags.rcVertexLength() == 1);
            }

            float reconnectionJacobian = 1.f;

            if (temporalUpdateForDynamicScene && kTemporalUpdateForDynamicScene && !isRcVertexEscapedVertex)
            {
                static const int id = 0;
                gPathTracer.traceTemporalUpdate(dstRcPrevVertexSd, tempPathReservoir);
                srcReservoir.rcVertexIrradiance[id] = tempPathReservoir.rcVertexIrradiance[id];
                srcReservoir.lightPdf = tempPathReservoir.lightPdf;
                srcReservoir.pathFlags.insertLightType(tempPathReservoir.pathFlags.lightType());
                srcReservoir.rcVertexWi[id] = tempPathReservoir.rcVertexWi[id];
            }

            print("----- dstRcPrevVertexSd", dstRcPrevVertexSd.posW);
            print("----- srcRcPrevVertexSd", srcRcPrevVertexSd.posW);

            // Why is rcTp always 0 for HybridSMS?
            rcTp = computeShiftedIntegrandReconnection(params, reconnectionJacobian, dstRcPrevVertexSd, srcRcPrevVertexSd, srcReservoir,
                evalVisibility, true, srcReservoir.pathFlags.rcVertexLength() > 1);

            dstJacobian *= reconnectionJacobian;

            tempPathReservoir.cachedJacobian = srcReservoir.cachedJacobian;
        }
    }
    else
    {
        print("----- Reconnection vertex doesn't exists", 0);
    }

    print("---- Tp", Tp);
    print("---- rcTp", rcTp);

    return Tp * rcTp;
}

void traceHybridShiftRays(
    const RestirPathTracerParams params,
    bool usePrev,
    const PackedHitInfo dstPrimaryHitPacked,
    const ShadingData dstPrimarySd,
    PathReservoir srcReservoir,
    inout ReconnectionData reconnectionData,    // dst
    inout DebugPathData debugPathData
)
{
    print("-- srcReservoir.weight", srcReservoir.weight);

    if (srcReservoir.weight == 0.f)
    {
        // reservoir has no contribution; basically a miss
        print("-- return because 0 weight", 0);
        reconnectionData.pathThroughput = float3(0.f);
        return;
    }

    // usePrev means use previous frame data
    HitInfo dstPrimaryHit;

    dstPrimaryHit.unpack(dstPrimaryHitPacked);
    Ray ray = Ray(gScene.camera.getPosition(usePrev), -dstPrimarySd.V, 0, kRayTMax); // this ray can be arbitrary, does not affect results

    gPathTracer.traceRandomReplayPathHybridSimple(
        dstPrimaryHit,
        dstPrimarySd,
        ray,
        srcReservoir.pathFlags,
        srcReservoir.initRandomSeed,
        reconnectionData,
        debugPathData
    );
}


void traceDenoiserData(const RestirPathTracerParams params, const PackedHitInfo dstPrimaryHitPacked, const ShadingData dstPrimarySd,
    ReSTIRPathFlags pathFlags, uint initRandomSeed, inout float hitDist, inout NRDPathType pathType, inout float3 sampleReflectance)
{
    HitInfo dstPrimaryHit; dstPrimaryHit.unpack(dstPrimaryHitPacked);
    Ray ray = Ray(gScene.camera.getPosition(), -dstPrimarySd.V, 0, kRayTMax); // this ray can be arbitrary, does not affect results
    gPathTracer.traceRandomReplayGetDenoiserData(dstPrimaryHit, dstPrimarySd, ray, pathFlags,
        initRandomSeed, hitDist, pathType, sampleReflectance);
}


bool isJacobianInvalid(float Jacobian)
{
    return Jacobian <= 0.f || isnan(Jacobian) || isinf(Jacobian);
}

#if BPR
float3 computeShiftedIntegrandReconnectionPathTree(const RestirPathTracerParams params, inout float dstJacobian, ShadingData dstPrimarySd,
    ShadingData srcPrimarySd, inout PathReservoir srcReservoir, bool evalVisibility)
{
    dstJacobian = 0.f;

    int rcVertexLength = 1;
    HitInfo rcVertexHit = srcReservoir.rcVertexHit.getHitInfo();
    float3 rcVertexIrradiance = srcReservoir.rcVertexIrradiance[0];

    float3 rcVertexBSDFLightSamplingIrradiance = srcReservoir.rcVertexBSDFLightSamplingIrradiance;
    float3 rcVertexWi = srcReservoir.rcVertexWi[0];

    bool rcVertexHitExists = rcVertexHit.isValid();
    if (!rcVertexHitExists)
    {
        return 0.f;
    }
    bool isTransmission = srcReservoir.pathFlags.decodeIsTransmissionEvent(true, false);

    srcPrimarySd.posW = srcPrimarySd.computeNewRayOrigin(!isTransmission);
    dstPrimarySd.posW = dstPrimarySd.computeNewRayOrigin(!isTransmission);

    bool isDelta1 = srcReservoir.pathFlags.decodeIsDeltaEvent(true);
    bool isDelta2 = srcReservoir.pathFlags.decodeIsDeltaEvent(false);

    // delta bounce before/after rcVertex (if isRcVertexNEE, deltaAfterRc won't be set)
    if (isDelta1 || isDelta2)
        if (any(srcPrimarySd.posW != dstPrimarySd.posW))
            return 0.f;

    ShadingData rcVertexSd = loadShadingDataWithPrevVertexPosition(rcVertexHit, dstPrimarySd.posW, false);

    // need to evaluate source PDF of BSDF sampling
    float3 dstConnectionV = -rcVertexSd.V; // direction point from dst primary hit point to reconnection vertex
    float3 srcConnectionV = normalize(rcVertexSd.posW - srcPrimarySd.posW);

    float3 shiftedDisp = rcVertexSd.posW - dstPrimarySd.posW;
    float shifted_dist2 = dot(shiftedDisp, shiftedDisp);
    float shifted_cosine = abs(dot(rcVertexSd.faceN, -dstConnectionV));

    float3 originalDisp = rcVertexSd.posW - srcPrimarySd.posW;
    float original_dist2 = dot(originalDisp, originalDisp);
    float original_cosine = abs(dot(rcVertexSd.faceN, -srcConnectionV));
    float Jacobian = shifted_cosine / shifted_dist2 * original_dist2 / original_cosine;
    if (isJacobianInvalid(Jacobian)) return 0.f;

    // assuming BSDF sampling
    assert(kUseBSDFSampling);

    uint allowedSampledTypes1 = getAllowedBSDFFlags(srcReservoir.pathFlags.decodeIsSpecularBounce(true));

    // assuming bsdf sampling
    float dstPDF1 = evalPdfBSDF(dstPrimarySd, dstConnectionV, allowedSampledTypes1, isDelta1);
    float srcPDF1 = evalPdfBSDF(srcPrimarySd, srcConnectionV, allowedSampledTypes1, isDelta1); //

    Jacobian *= dstPDF1 / srcPDF1; // this is the only jacobian we need for computing MIS weight for path reuse NEE

    if (isJacobianInvalid(Jacobian)) return 0.f;

    float3 dstF1 = evalBSDFCosine(dstPrimarySd, dstConnectionV, allowedSampledTypes1, isDelta1);
    float3 dstIntegrand = dstF1 / dstPDF1;

    float3 dstIntegrandNEE;

    float NEEJacobian = Jacobian;
    float BSDFSamplingJacobian = 0.f;

    dstIntegrandNEE = 0.f;
    if (kUseNEE && any(srcReservoir.rcVertexIrradiance[1] > 0.f))
    {
        float3 dstF2NEE = evalBSDFCosine(rcVertexSd, srcReservoir.rcVertexWi[1]);
        float dstPDF2NEE = srcReservoir.rcLightPdf;
        float dstRcVertexNEEScatterPdf = evalPdfBSDF(rcVertexSd, srcReservoir.rcVertexWi[1]);

        dstIntegrandNEE = dstF2NEE / dstPDF2NEE * srcReservoir.rcVertexIrradiance[1];
        float misWeightNEE = 1.f;

        if (kUseMIS && srcReservoir.rcLightType() != (uint)PathTracer::LightSampleType::Analytic)
        {
            float lightPdf = dstPDF2NEE;
            misWeightNEE *= PathTracer::evalMIS(1, lightPdf, 1, dstRcVertexNEEScatterPdf);
        }
        dstIntegrandNEE *= misWeightNEE;
    }

    uint allowedSampledTypes2 = getAllowedBSDFFlags(srcReservoir.pathFlags.decodeIsSpecularBounce(false));

    float3 dstIntegrandBSDFSampling = 0;

    if (any(rcVertexIrradiance > 0) || any(rcVertexBSDFLightSamplingIrradiance > 0))
    {
        float dstRcVertexScatterPdfAll = 0.f;

        // assuming bsdf sampling
        float dstRcVertexScatterPdf = evalPdfBSDF(rcVertexSd, rcVertexWi, dstRcVertexScatterPdfAll, allowedSampledTypes2, isDelta2);
        float srcRcVertexScatterPdf = evalPdfBSDFWithV(rcVertexSd, -srcConnectionV, rcVertexWi, allowedSampledTypes2, isDelta2);
        float dstPDF2 = dstRcVertexScatterPdf;

        float3 dstF2 = evalBSDFCosine(rcVertexSd, rcVertexWi, allowedSampledTypes2, isDelta2);

        float lightPdfBSDFSampling = srcReservoir.lightPdf;

        dstIntegrandBSDFSampling = dstF2 / dstPDF2 * rcVertexIrradiance;

        // include MIS
        if (kUseMIS || !kUseNEE)
        {
            if (any(rcVertexBSDFLightSamplingIrradiance > 0))
            {
                float misWeightBSDFSampling = PathTracer::evalMIS(1, dstRcVertexScatterPdfAll, 1, lightPdfBSDFSampling);
                dstIntegrandBSDFSampling += dstF2 / dstPDF2 * rcVertexBSDFLightSamplingIrradiance * misWeightBSDFSampling;
            }
        }

        Jacobian *= dstRcVertexScatterPdf / srcRcVertexScatterPdf;

        BSDFSamplingJacobian = Jacobian;
    }

    if (isJacobianInvalid(Jacobian)) return 0.f;

    // Evaluate visibility: vertex 1 <-> vertex 2 (reconnection vertex).
    if (evalVisibility)
    {
        bool isVisible = evalSegmentVisibility(dstPrimarySd.posW, rcVertexSd.posW);
        if (!isVisible)
        {
            return 0.f;
        }
    }

    // non-invertible case
    srcReservoir.rcVertexIrradiance[0] = dstIntegrand * dstIntegrandBSDFSampling;
    if (all(srcReservoir.rcVertexIrradiance[0] == 0.f)) BSDFSamplingJacobian = 0.f;

    srcReservoir.rcVertexIrradiance[1] = dstIntegrand * dstIntegrandNEE;
    if (all(srcReservoir.rcVertexIrradiance[1] == 0.f)) NEEJacobian = 0.f;

    dstIntegrand = dstIntegrand * (dstIntegrandBSDFSampling + dstIntegrandNEE);

    if (any(isnan(dstIntegrand) || isinf(dstIntegrand))) return 0.f;

    srcReservoir.cachedJacobian[0] = BSDFSamplingJacobian;
    srcReservoir.cachedJacobian[1] = NEEJacobian;

    dstJacobian = Jacobian; // product jacobian

    return dstIntegrand;
}
#endif

// dstPdf * dstJacobian transforms pdf in dst space to src space
// srcPdf / dstJacobian transforms pdf in src space to dst space
float3 computeShiftedIntegrandReconnection(const RestirPathTracerParams params, inout float dstJacobian,  ShadingData dstPrimarySd,
                                ShadingData srcPrimarySd, inout PathReservoir srcReservoir, bool evalVisibility,
                                bool useHybridShift=false, bool useCachedJacobian = false)
{
    float3 dstCachedJacobian;
    dstJacobian = 0.f;

    int rcVertexLength = !useHybridShift ? 1 : srcReservoir.pathFlags.rcVertexLength();

    HitInfo rcVertexHit = srcReservoir.rcVertexHit.getHitInfo();
    float3 rcVertexIrradiance = srcReservoir.rcVertexIrradiance[0];
    float3 rcVertexWi = srcReservoir.rcVertexWi[0];
    bool rcVertexHitExists = rcVertexHit.isValid();

    bool isTransmission = srcReservoir.pathFlags.decodeIsTransmissionEvent(true);
    uint allowedSampledTypes1 = getAllowedBSDFFlags(srcReservoir.pathFlags.decodeIsSpecularBounce(true));

    srcPrimarySd.posW = srcPrimarySd.computeNewRayOrigin(!isTransmission);
    dstPrimarySd.posW = dstPrimarySd.computeNewRayOrigin(!isTransmission);

    if (!rcVertexHitExists)
    {

        print("------ rcVertex does not exists", 0);
        float3 dstIntegrand = 0.f;
        // are we having a infinite light as rcVertex?
        if (kUseMIS && srcReservoir.pathFlags.lightType() == (uint)PathTracer::LightSampleType::EnvMap && srcReservoir.pathFlags.pathLength() + 1 == rcVertexLength && !srcReservoir.pathFlags.lastVertexNEE())
        {
            float3 wi = rcVertexWi;
            bool isVisible = evalSegmentVisibility(dstPrimarySd.posW, wi, true); // test along a direction
            if (isVisible)
            {
                float srcPDF1 = useCachedJacobian ? srcReservoir.cachedJacobian.x : evalPdfBSDF(srcPrimarySd, wi);
                float dstPDF1All;
                float dstPDF1 = evalPdfBSDF(dstPrimarySd, wi, dstPDF1All, allowedSampledTypes1);
                dstCachedJacobian.x = dstPDF1;
                float3 dstF1 = evalBSDFCosine(dstPrimarySd, wi, allowedSampledTypes1);
                float misWeight = PathTracer::evalMIS(1, dstPDF1All, 1, srcReservoir.lightPdf);//   dstPDF1 / (dstPDF1 + srcReservoir.lightPdf);
                dstIntegrand = dstF1 / dstPDF1 * misWeight * rcVertexIrradiance;
                dstJacobian = dstPDF1 / srcPDF1;
            }
        }

        if (useCachedJacobian)
            srcReservoir.cachedJacobian = dstCachedJacobian;

        print("------ rcVertex does not exists", 0);

        // fill in rcVertex0 information
        if (isJacobianInvalid(dstJacobian)) dstJacobian = 0.f;
        if (any(isnan(dstIntegrand) || isinf(dstIntegrand))) return 0.f;

        return dstIntegrand;
    }

    print("------ rcVertex does exists", 0);

    bool isRcVertexFinal = srcReservoir.pathFlags.pathLength() == rcVertexLength;
    bool isRcVertexEscapedVertex = srcReservoir.pathFlags.pathLength() + 1 == rcVertexLength && !srcReservoir.pathFlags.lastVertexNEE();
    bool isRcVertexNEE = isRcVertexFinal && srcReservoir.pathFlags.lastVertexNEE();

    bool isDelta1 = srcReservoir.pathFlags.decodeIsDeltaEvent(true);
    bool isDelta2 = srcReservoir.pathFlags.decodeIsDeltaEvent(false);

    // delta bounce before/after rcVertex (if isRcVertexNEE, deltaAfterRc won't be set)
    print("------ path flags ", srcReservoir.pathFlags.flags);
    if (isDelta1 || isDelta2)
    {
        print("------ is delta", (isDelta1 ? 1 : 0) | (isDelta2 ? 2 : 0));
        return 0.f;
    }

    print("------ is not delta", 0);

    ShadingData rcVertexSd = loadShadingDataWithPrevVertexPosition(rcVertexHit, dstPrimarySd.posW, false);

    // need to evaluate source PDF of BSDF sampling
    float3 dstConnectionV = -rcVertexSd.V; // direction point from dst primary hit point to reconnection vertex
    float3 srcConnectionV = normalize(rcVertexSd.posW - srcPrimarySd.posW);

    float3 shiftedDisp = rcVertexSd.posW - dstPrimarySd.posW;
    float shifted_dist2 = dot(shiftedDisp, shiftedDisp);
    float shifted_cosine = abs(dot(rcVertexSd.faceN, -dstConnectionV));

    if ((params.localStrategyType & (uint)LocalStrategy::DistanceCondition) && useHybridShift)
    {
        bool isFarField = sqrt(shifted_dist2) >= params.nearFieldDistance;
        if (!isFarField)
        {
            // Only trigger for Spaitial reuse
            print("------ Reject for being too close", 0);
            return 0.f;
        }
    }

    print("------ is far field", 0);

    dstCachedJacobian.z = shifted_cosine / shifted_dist2;
    float Jacobian;
    if (useCachedJacobian) Jacobian = dstCachedJacobian.z / srcReservoir.cachedJacobian.z;
    else
    {
        float3 originalDisp = rcVertexSd.posW - srcPrimarySd.posW;
        float original_dist2 = dot(originalDisp, originalDisp);
        float original_cosine = abs(dot(rcVertexSd.faceN, -srcConnectionV));
        Jacobian = dstCachedJacobian.z* original_dist2 / original_cosine;
    }
    if (isJacobianInvalid(Jacobian)) return 0.f;

    print("------ valid jacobian 1", 0);

    // assuming BSDF sampling
    assert(kUseBSDFSampling);

    // assuming bsdf sampling
    float dstPDF1All = 0.f;
    float dstPDF1 = evalPdfBSDF(dstPrimarySd, dstConnectionV, dstPDF1All, allowedSampledTypes1);
    dstCachedJacobian.x = dstPDF1;
    float srcPDF1 = useCachedJacobian ? srcReservoir.cachedJacobian.x : evalPdfBSDF(srcPrimarySd, srcConnectionV, allowedSampledTypes1); //

    Jacobian *= dstPDF1 / srcPDF1;

    if (isJacobianInvalid(Jacobian)) return 0.f;

    print("------ valid jacobian 2", 0);

    float3 dstF1 = evalBSDFCosine(dstPrimarySd, dstConnectionV, allowedSampledTypes1);

    float dstRcVertexScatterPdfAll = 0.f;
    float dstPDF2 = 1.f;
    float dstRcVertexScatterPdf = 1.f;
    float srcRcVertexScatterPdf = 1.f;

    uint allowedSampledTypes2 = isRcVertexNEE ? -1 : getAllowedBSDFFlags(srcReservoir.pathFlags.decodeIsSpecularBounce(false));

    if (!isRcVertexEscapedVertex)
    {
        // assuming bsdf sampling
        dstRcVertexScatterPdf = evalPdfBSDF(rcVertexSd, rcVertexWi, dstRcVertexScatterPdfAll, allowedSampledTypes2);
        dstCachedJacobian.y = dstRcVertexScatterPdf;
        srcRcVertexScatterPdf = useCachedJacobian ? srcReservoir.cachedJacobian.y : evalPdfBSDFWithV(rcVertexSd, -srcConnectionV, rcVertexWi, allowedSampledTypes2);

        if (!isRcVertexNEE) dstPDF2 = dstRcVertexScatterPdf;
        else dstPDF2 = srcReservoir.lightPdf;
    }

    float3 dstF2 = 1.f;

    if (!isRcVertexEscapedVertex)
        dstF2 = evalBSDFCosine(rcVertexSd, rcVertexWi, allowedSampledTypes2);

    // connection point behind surface
    if (all(dstF1 == 0.f) || all(dstF2 == 0.f)) return 0.f;

    print("------ valid connection point", 0);

    //////
    float3 dstIntegrandNoF1 = dstF2 / dstPDF2 * rcVertexIrradiance;
    float3 dstIntegrand = dstF1 / dstPDF1 * dstIntegrandNoF1; // TODO: might need to reevaluate Le for changing emissive lights

    if (isRcVertexEscapedVertex)
    {
        float misWeight = PathTracer::evalMIS(1, dstPDF1All, 1, srcReservoir.lightPdf);// dstPDF1 / (srcReservoir.lightPdf + dstPDF1);
        dstIntegrand *= misWeight;
    }

    // MIS weight
    if (isRcVertexFinal && kUseMIS)
    {
        if (srcReservoir.pathFlags.lightType() != (uint)PathTracer::LightSampleType::Analytic) // TODO: optimize way this check
        {
            float lightPdf = srcReservoir.lightPdf;
            float misWeight = PathTracer::evalMIS(1, isRcVertexNEE ? lightPdf : dstRcVertexScatterPdfAll, 1, isRcVertexNEE ? dstRcVertexScatterPdfAll : lightPdf);
            dstIntegrand = dstIntegrand * misWeight;
            dstIntegrandNoF1 = dstIntegrandNoF1 * misWeight;
            if (!isRcVertexNEE)
                Jacobian *= dstRcVertexScatterPdf / srcRcVertexScatterPdf;
        }
    }

    // need to account for non-identity jacobian due to BSDF sampling
    if (!isRcVertexFinal && !isRcVertexEscapedVertex)
    {
        Jacobian *= dstRcVertexScatterPdf / srcRcVertexScatterPdf;
    }

    if (isJacobianInvalid(Jacobian)) return 0.f;

    print("------ valid jacobian 3", 0);


    // Evaluate visibility: vertex 1 <-> vertex 2 (reconnection vertex).
    if (evalVisibility)
    {
        bool isVisible = evalSegmentVisibility(dstPrimarySd.posW, rcVertexSd.posW);
        if (!isVisible)
            return 0.f;
    }

    print("------ visible", 0);

    if (any(isnan(dstIntegrand) || isinf(dstIntegrand))) return 0.f;

    print("------ dstIntegrand is valid", 0);

    if (params.rejectShiftBasedOnJacobian)
    {
        if (Jacobian > 0.f && (max(Jacobian, 1 / Jacobian) > 1 + params.jacobianRejectionThreshold))
        {
            // discard based on Jacobian (unbiased)
            Jacobian = 0.f;
            dstIntegrand = 0.f;
        }
    }

    dstJacobian = Jacobian;

    if (useCachedJacobian)
    {
        srcReservoir.cachedJacobian = dstCachedJacobian;
        print("------ use cache jacobian", dstCachedJacobian);
    }

    print("------ dstIntegrand", dstIntegrand);

    return dstIntegrand;
}


bool shiftAndMergeReservoir(const RestirPathTracerParams params, bool temporalUpdateForDynamicScene, inout float dstJacobian, const PackedHitInfo dstPrimaryHitPacked, const ShadingData dstPrimarySd, inout PathReservoir dstReservoir,
                                    const ShadingData srcPrimarySd, const PathReservoir srcReservoir, ReconnectionData rcData, bool evalVisibility, inout SampleGenerator sg, bool isSpatialReuse, float misWeight = 1.f, bool forceMerge = false)
{
    PathReservoir tempPathReservoir = srcReservoir;
    // dst: dst
    // src: src
    float3 dstIntegrand = computeShiftedIntegrand_(params, dstJacobian, dstPrimaryHitPacked, dstPrimarySd, srcPrimarySd,
                                                            tempPathReservoir, rcData, evalVisibility, false, temporalUpdateForDynamicScene);

    if (PathSamplingMode(kPathSamplingMode) == PathSamplingMode::PathReuse)
    {
        dstReservoir.F = dstIntegrand;
        dstReservoir.pathFlags = tempPathReservoir.pathFlags;
        dstReservoir.rcRandomSeed = tempPathReservoir.rcRandomSeed;
        dstReservoir.initRandomSeed = tempPathReservoir.initRandomSeed;
        dstReservoir.cachedJacobian = tempPathReservoir.cachedJacobian;
        dstReservoir.lightPdf = tempPathReservoir.lightPdf;
        dstReservoir.rcVertexWi[0] = tempPathReservoir.rcVertexWi[0];
        dstReservoir.rcVertexHit = tempPathReservoir.rcVertexHit;
        dstReservoir.rcVertexIrradiance[0] = tempPathReservoir.rcVertexIrradiance[0];
        dstReservoir.rcVertexWi[1] = tempPathReservoir.rcVertexWi[1];
        dstReservoir.rcVertexIrradiance[1] = tempPathReservoir.rcVertexIrradiance[1];
        dstReservoir.weight = tempPathReservoir.weight;

        return any(dstReservoir.F > 0);
    }
    else
    {
        bool selected = dstReservoir.merge(dstIntegrand, dstJacobian, tempPathReservoir, sg, misWeight, forceMerge);

        if (forceMerge)
        {
            if (!selected) dstReservoir.F = 0.f;
            dstReservoir.M = srcReservoir.M;
            dstReservoir.weight = srcReservoir.weight;
        }

        return selected;
    }
}

bool mergeReservoirNoResampling(const RestirPathTracerParams params, inout PathReservoir dstReservoir, const PathReservoir srcReservoir, inout SampleGenerator sg)
{
    bool selected = dstReservoir.merge(srcReservoir.F, 1.f, srcReservoir, sg, 1.f);
    return selected;
}

bool mergeReservoirWithResamplingMIS(const RestirPathTracerParams params, float3 dstIntegrand, float dstJacobian, inout PathReservoir dstReservoir, const PathReservoir tempDstReservoir, const PathReservoir srcReservoir, inout SampleGenerator sg, bool isSpatialReuse, float misWeight = 1.f)
{
    bool selected = dstReservoir.mergeWithResamplingMIS(dstIntegrand, dstJacobian, tempDstReservoir, sg, misWeight);
    return selected;
}

bool mergeReservoir(const RestirPathTracerParams params, float3 dstIntegrand, float dstJacobian, inout PathReservoir dstReservoir, const PathReservoir tempDstReservoir, const PathReservoir srcReservoir, inout SampleGenerator sg, bool isSpatialReuse, float misWeight = 1.f)
{
    bool selected = dstReservoir.merge(dstIntegrand, dstJacobian, tempDstReservoir, sg, misWeight);
    return selected;
}

/// Evaluates the visibility term to the reconnection vertex from the source vertex.
bool evalSegmentVisibility(const float3 sourceVertex, const float3 targetVertex, bool isDirection = false)
{
    float3 dir;
    float tMax;
    if (isDirection)
    {
        dir = targetVertex;
        tMax = kRayTMax;
    }
    else
    {
        float3 edge = targetVertex - sourceVertex;
        tMax = length(edge);
        dir = edge / tMax;
        tMax *= 0.999f; // prevent intersection with another surface
    }

    RayDesc ray = { sourceVertex, 0.f, dir, tMax }; // TODO: use offset position instead

    SceneRayQuery<kUseAlphaTest> sceneRayQuery;
    return sceneRayQuery.traceVisibilityRay(ray, RAY_FLAG_NONE, 0xff);
}

bool traceVisibilityRay(const Ray ray)
{
    SceneRayQuery<kUseAlphaTest> sceneRayQuery;
    return sceneRayQuery.traceVisibilityRay(ray.toRayDesc(), RAY_FLAG_NONE, 0xff);
}

