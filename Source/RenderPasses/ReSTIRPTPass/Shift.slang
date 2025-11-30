/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/

#include "Utils/Math/MathConstants.slangh"

import PathReservoir;
import Utils.Sampling.SampleGenerator;
import LoadShadingData;
import Params;
import Utils.Debug.PixelDebug;
import Utils.Math.MathHelpers;
import Rendering.Materials.MaterialShading;
import Rendering.Utils.PixelStats;
import Scene.RaytracingInline;
import Scene.Scene;
import PathTracer;
import RenderPasses.Shared.Denoising.NRDData;
import DebugPathDataType;
import SpecularManifold;

ParameterBlock<PathTracer> gPathTracer;

float3 computeShiftedIntegrandExtraOutput(
    const RestirPathTracerParams params, inout float dstJacobian, inout float output2, const PackedHitInfo dstPrimaryHitPacked, const ShadingData dstPrimarySd,
    const ShadingData srcPrimarySd, const PathReservoir srcReservoir, ReconnectionData rcData, SampleGenerator sg, bool evalVisibility, bool usePrev = false, bool temporalUpdateForDynamicScene = false)
{
    PathReservoir tempReservoir = srcReservoir;
    float3 res = computeShiftedIntegrand_(params, dstJacobian, dstPrimaryHitPacked, dstPrimarySd,
        srcPrimarySd, tempReservoir, rcData, sg, evalVisibility, usePrev, temporalUpdateForDynamicScene);
    if (PathSamplingMode(kPathSamplingMode) == PathSamplingMode::PathReuse) // for precomputing MIS weights
    {
        dstJacobian = tempReservoir.cachedJacobian[0];
        output2 = tempReservoir.cachedJacobian[1];
    }
    return res;
}

float3 computeShiftedIntegrand(
    const RestirPathTracerParams params, inout float dstJacobian, const PackedHitInfo dstPrimaryHitPacked, const ShadingData dstPrimarySd,
    const ShadingData srcPrimarySd, const PathReservoir srcReservoir, ReconnectionData rcData, SampleGenerator sg, bool evalVisibility, bool usePrev = false, bool temporalUpdateForDynamicScene = false)
{
    PathReservoir tempReservoir = srcReservoir;
    float3 res = computeShiftedIntegrand_(params, dstJacobian, dstPrimaryHitPacked, dstPrimarySd,
        srcPrimarySd, tempReservoir, rcData, sg, evalVisibility, usePrev, temporalUpdateForDynamicScene);
    if (PathSamplingMode(kPathSamplingMode) == PathSamplingMode::PathReuse) // for precomputing MIS weights
    {
        dstJacobian = tempReservoir.cachedJacobian[0];
    }
    return res;
}

float3 computeShiftedIntegrand_(
    const RestirPathTracerParams params,
    inout float dstJacobian,
    const PackedHitInfo dstPrimaryHitPacked,
    const ShadingData dstPrimarySd,
    const ShadingData srcPrimarySd,
    inout PathReservoir srcReservoir,
    ReconnectionData rcData,
    SampleGenerator sg,
    bool evalVisibility,
    bool usePrev = false,
    bool temporalUpdateForDynamicScene = false
)
{
    dstJacobian = 0.f;

    // Filter out empty reservoir case
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
            print("--- Doing DSD RC in HybridShiftSMS", 0);
            // DSD reconnection
            tmp = computeShiftedIntegrandHybridShiftSMS(params, usePrev, temporalUpdateForDynamicScene, dstJacobian, dstPrimaryHitPacked, dstPrimarySd, srcPrimarySd, srcReservoir, rcData, evalVisibility, sg);

            print("--- integrand", tmp);
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

            // kSeparatePathBSDF = true
            bool isLastVertexClassifiedAsRough = kSeparatePathBSDF ?
                (isSpecularBounce ? dstPrimarySd.linearRoughness > params.specularRoughnessThreshold : hasRoughComponent(dstPrimarySd, 1.f)) :
                classifyAsRough(dstPrimarySd, params.specularRoughnessThreshold);

            print("----- isLastVertexClassifiedAsRough", isLastVertexClassifiedAsRough);

            if (!isLastVertexClassifiedAsRough)
            {
                // RcPrevVertex is specular. Invalidate
                print("----- Invalidate shift because prevRc is specular", 0);
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

// dstPdf * dstJacobian transforms pdf in dst space to src space
// srcPdf / dstJacobian transforms pdf in src space to dst space
float3 computeShiftedIntegrandDSD(
    const RestirPathTracerParams params,
    ShadingData dstPrevRcSd,
    ShadingData dstPrevPrevRcSd,
    float3 dstRcPrevDnDu,
    float3 dstRcPrevDnDv,
    ShadingData srcPrevRcSd,
    ShadingData srcPrevPrevRcSd,
    float3 srcRcPrevDnDu,
    float3 srcRcPrevDnDv,
    inout PathReservoir srcReservoir,
    inout float dstJacobian,
    bool evalVisibility
)
{
    print("------ computeShiftedIntegrandDSD: begin()", 0);

    bool useCachedJacobian = false;
    bool useHybridShift = true;

    float3 dstCachedJacobian;
    dstJacobian = 0.f;

    int rcVertexLength = srcReservoir.pathFlags.rcVertexLength();

    HitInfo rcVertexHit = srcReservoir.rcVertexHit.getHitInfo();
    float3 rcVertexIrradiance = srcReservoir.rcVertexIrradiance[0];
    float3 rcVertexWi = srcReservoir.rcVertexWi[0];
    bool rcVertexHitExists = rcVertexHit.isValid();

    bool isPrevRcTransmission = srcReservoir.pathFlags.decodeIsTransmissionEvent(true);
    srcPrevRcSd.posW = srcPrevRcSd.computeNewRayOrigin(!isPrevRcTransmission);
    dstPrevRcSd.posW = dstPrevRcSd.computeNewRayOrigin(!isPrevRcTransmission);

    // Caution: Keep in mind that rcVertexSd.V will be from dstPrevRc, NOT srcPrevRc
    ShadingData rcVertexSd = loadShadingDataWithPrevVertexPosition(rcVertexHit, dstPrevRcSd.posW, false);

    // need to evaluate source PDF of BSDF sampling

    // Check distance condition between rc <-> prevRc, prevRc <-> prevPrevRc
    //  TODO: Because we are not handling EnvMap now, move this to be caller's responsibility
    if (params.localStrategyType & (uint)LocalStrategy::DistanceCondition)
    {
        float lenRcToPrevRc = length(rcVertexSd.posW - dstPrevRcSd.posW);
        float lenPrevRcToPrevPrevRc = length(dstPrevRcSd.posW - dstPrevPrevRcSd.posW);
        if (lenRcToPrevRc < params.nearFieldDistance || lenPrevRcToPrevPrevRc < params.nearFieldDistance)
        {
            print("------ Rejected for being too close", 0);
            return 0.f;
        }
    }

    //
    // Compute Jacobian of the shift
    //
    print("------ Compute Jacobian", 0);
    float Jacobian = computeReconnectionDSDShiftJacobian(
        rcVertexSd,
        srcPrevRcSd,
        srcPrevPrevRcSd,
        srcRcPrevDnDu,
        srcRcPrevDnDv,
        rcVertexSd,
        dstPrevRcSd,
        dstPrevPrevRcSd,
        dstRcPrevDnDu,
        dstRcPrevDnDv
    );

    if (isJacobianInvalid(Jacobian))
    {
        print("------ Invalid Jacobian DSD shift", 0);
        return 0.f;
    }

    //
    // Compute Jacobian: Random Replay of choosing same RC [GRIS, Eq. 54]
    //
    // assuming BSDF sampling
    assert(kUseBSDFSampling);

    //  Get allowed sampled types at srcPrevRc and srcPrevPrevRc vertices
    uint allowedSampledTypesPrevPrevRc = getAllowedBSDFFlags(false);
    uint allowedSampledTypesPrevRc = getAllowedBSDFFlags(true);

    //  DST PDF : pdf of dstPrevPrevRc -> dstPrevRc -> rc ; given dstPrevPrevRc.V
    float dstPDFPrevPrevRcAll = 0.f;
    float dstPDFPrevPrevRc = evalPdfBSDF(dstPrevPrevRcSd, -dstPrevRcSd.V, dstPDFPrevPrevRcAll, allowedSampledTypesPrevPrevRc);
    float dstPDFPrevRcAll = 0.f;
    float dstPDFPrevRc = evalPdfBSDF(dstPrevRcSd, -rcVertexSd.V, dstPDFPrevRcAll, allowedSampledTypesPrevRc, true);

    print("------- dstPDFPrevPrevRc", dstPDFPrevPrevRc);
    print("------- dstPDFPrevRc", dstPDFPrevRc);

    // SRC PDF : pdf of srcPrevPrevRc -> srcPrevRc -> rc ; given srcPrevPrevRc.V
    float srcPDFPrevPrevRc = evalPdfBSDF(srcPrevPrevRcSd, -srcPrevRcSd.V, allowedSampledTypesPrevPrevRc);
    // norm(srcPrev -> rc)
    float3 srcDirPrevToRc = normalize(rcVertexSd.posW - srcPrevRcSd.posW);
    float srcPDFPrevRc = evalPdfBSDF(srcPrevRcSd, srcDirPrevToRc, allowedSampledTypesPrevRc, true);

    print("------- srcPDFPrevPrevRc", srcPDFPrevPrevRc);
    print("------- srcPDFPrevRc", srcPDFPrevRc);

    // Adding in Random Replay Jacobian Shift in Primary Sample Sapce[GRIS Eq. 54]
    //  In DSD case, we may not need PDF1_rc because we assume pure specular so pdfPrevRc should always be 1
    float JacobianRandomReplay = (dstPDFPrevPrevRc * dstPDFPrevRc) / (srcPDFPrevPrevRc * srcPDFPrevRc);
    Jacobian *= JacobianRandomReplay;
    dstCachedJacobian.x = dstPDFPrevPrevRc;

    print("------- Jacobian Random Replay:", JacobianRandomReplay);

    if (isJacobianInvalid(Jacobian))
    {
        print("------ Invalid Jacobian Reconnection shift", 0);
        return 0.f;
    }

    print("------- total Jacobian det:", Jacobian);

    print("------ Compute reconnection integrand", 0);

    //
    // Compute the integrand at dstPrevPrevRc
    //      Intg(dstPrevPrev) = f(dstPrevPrev) * f(dstPrev) * f(RC) * rcVertexIrradiance
    //                          / ( pdf(dstPrevPrev.wi | dstPrevPrev.wo) *  pdf(dstPrev.wi | dstPrev.wo) * pdf(RC.wi | RC.wo to dsPrevRc))

    //      dstPrevPrev
    float3 dstFPrevPrevRc = evalBSDFCosine(dstPrevPrevRcSd, -dstPrevRcSd.V, allowedSampledTypesPrevPrevRc);

    //      dstPrev
    float3 dstFPrevRc = evalBSDFCosine(dstPrevRcSd, -rcVertexSd.V, allowedSampledTypesPrevRc, true);

    //      RC
    float3 dstFRc = 1.f;
    float dstPDFRc = 1.f;
    float dstRcVertexScatterPdf = 1.f;
    float dstRcVertexScatterPdfAll = 0.f;
    float srcRcVertexScatterPdf = 1.f;

    int srcPathLength = srcReservoir.pathFlags.pathLength();
    bool isRcVertexFinal = srcPathLength == rcVertexLength;
    bool isRcVertexNEE = isRcVertexFinal && srcReservoir.pathFlags.lastVertexNEE();
    bool isRcVertexEscapedVertex = srcPathLength + 1 == rcVertexLength && !srcReservoir.pathFlags.lastVertexNEE();

    if (!isRcVertexEscapedVertex)
    {
        print("------- RC is not escaped", 0);

        uint allowedSampledTypesRc = isRcVertexNEE ? -1 : getAllowedBSDFFlags(srcReservoir.pathFlags.decodeIsSpecularBounce(false));

        // pdf of sampling rc -> srcRcWi ; given dstPrev -> rc
        dstRcVertexScatterPdf = evalPdfBSDF(rcVertexSd, rcVertexWi, dstRcVertexScatterPdfAll, allowedSampledTypesRc);

        // pdf of sampling rc -> RcWi ; with prior from srcPrevRc
        srcRcVertexScatterPdf = evalPdfBSDFWithV(rcVertexSd, -srcDirPrevToRc, rcVertexWi, allowedSampledTypesRc);

        if (isRcVertexNEE)
        {
            dstPDFRc = srcReservoir.lightPdf;
            print("------- Set dstPDFRc to lightPdf", dstPDFRc);
        }
        else
        {
            dstPDFRc = dstRcVertexScatterPdf;
            print("------- Set dstPDFRc to dstRcVertexScatterPdf", dstPDFRc);
        }

        dstFRc = evalBSDFCosine(rcVertexSd, rcVertexWi, allowedSampledTypesRc);
    }

    print("------ dstFPrevPrevRc", dstFPrevPrevRc);
    print("------ dstFPrevRc", dstFPrevRc);
    print("------ dstFRc", dstFRc);
    print("------ dstPDFPrevPrevRc", dstPDFPrevPrevRc);
    print("------ dstPDFPrevRc", dstPDFPrevRc);
    print("------ dstPDFRc", dstPDFRc);

    //      Check valid BSDFs
    if (all(dstFPrevPrevRc == 0.f) || all(dstFPrevRc == 0.f) || all(dstFRc == 0.f))
    {
        print("------ Invalid BSDFs", 0);
        return 0.f;
    }

    print("------ rcVertexIrradiance", rcVertexIrradiance);

    //      Finally compute Intg(dstPrevPrev)
    float3 dstIntgPrevPrevRc = dstFPrevPrevRc * dstFPrevRc * dstFRc * rcVertexIrradiance
                                / (dstPDFPrevPrevRc * dstPDFPrevRc * dstPDFRc);

    print("------ dstIntgPrevPrevRc", dstIntgPrevPrevRc);

    if (isRcVertexEscapedVertex)
    {
        // MIS between getting to RC via BSDF sampling and NEE
        // dstPDF1 / (srcReservoir.lightPdf + dstPDF1);
        float misWeight = PathTracer::evalMIS(1, dstPDFPrevRcAll, 1, srcReservoir.lightPdf);
        dstIntgPrevPrevRc *= misWeight;
        print("------ RC is escaped, added in MIS PrevRc and intg is now", dstIntgPrevPrevRc);
    }

    // If path ends at RC and base path sampled the light by EnvMap or Emissive means, then add in MIS weight
    if (isRcVertexFinal
        && kUseMIS
        && srcReservoir.pathFlags.lightType() != (uint)PathTracer::LightSampleType::Analytic
    )
    {
        float misWeight = PathTracer::evalMIS(1, dstRcVertexScatterPdfAll, 1, srcReservoir.lightPdf);

        dstIntgPrevPrevRc *= misWeight;

        // If at the ending RC does not sampled direct light then it
        //  go to sample EnvMap therefore adding in Jacobian of getting this sampling direction in offset path compare to in base path
        if (!isRcVertexNEE)
            Jacobian *= dstRcVertexScatterPdf / srcRcVertexScatterPdf;

        print("------ RC is final, added in MIS RC and is now", dstIntgPrevPrevRc);
    }

    // Check validity
    if (any(isnan(dstIntgPrevPrevRc) || isinf(dstIntgPrevPrevRc)))
    {
        print("------ Invalid dstIntgPrevPrevRc", dstIntgPrevPrevRc);
        return 0.f;
    }


    // Handle shift of scatter at RC from src to dst in case RC is neither final nor escaped.
    if (!isRcVertexFinal && !isRcVertexEscapedVertex)
    {
        // prob of sampling continuing direction on offset path based on base's
        Jacobian *= dstRcVertexScatterPdf / srcRcVertexScatterPdf;
    }

    print("------ Updated Jacobian at RC scatter:", Jacobian);

    if (isJacobianInvalid(Jacobian))
    {
        print("------ Invalid Jacobian", 0);
        return 0.f;
    }

    // Evaluate visibility: dstPrevPrevRc <-> dstPrevRc <-> dstRc
    if (evalVisibility)
    {
        bool isVisible = evalSegmentVisibility(dstPrevPrevRcSd.posW, dstPrevRcSd.posW);
        if (!isVisible)
        {
            print("------ dstPrevPrevRc <-> dstPrevRc is not visible", 0);
            return 0.f;
        }

        isVisible = evalSegmentVisibility(dstPrevRcSd.posW, rcVertexSd.posW);
        if (!isVisible)
        {
            print("------ dstPrevRc <-> dstRc is not visible", 0);
            return 0.f;
        }
    }

    // Reject Shift based on Jacobian
    if ((bool)params.rejectShiftBasedOnJacobian)
    {
        if (Jacobian > 0.f && (max(Jacobian, 1 / Jacobian) > 1 + params.jacobianRejectionThreshold))
        {
            print("------- Reject shift based on Jacobian", 0);
            // discard based on Jacobian (unbiased)
            Jacobian = 0.f;
            dstIntgPrevPrevRc = 0.f;
        }
    }

    dstJacobian = Jacobian;

    print("------ dstIntgPrevPrevRc", dstIntgPrevPrevRc);
    print("------ computeShiftedIntegrandDSD: end()", 0);

    return dstIntgPrevPrevRc;
}

// SRC: Base path
// DST: Offset path
float3 computeShiftedIntegrandHybridShiftSMS(
    const RestirPathTracerParams params,
    bool usePrev,
    bool temporalUpdateForDynamicScene,
    inout float dstJacobian,
    const PackedHitInfo dstPrimaryHitPacked,
    const ShadingData dstPrimarySd,
    const ShadingData srcPrimarySd,
    inout PathReservoir tempPathReservoir,
    ReconnectionData rcData,
    bool evalVisibility,
    SampleGenerator sg
)
{
    dstJacobian = 0.f;

    if (tempPathReservoir.weight == 0.f)
    {
        print("---- empty reservoir", 0);
        return 0.f;
    }

    dstJacobian = 1.f;

    // Create a copy
    PathReservoir srcReservoir = tempPathReservoir;
    int rcVertexLength = srcReservoir.pathFlags.rcVertexLength();
    int srcPathLength = srcReservoir.pathFlags.pathLength();
    // The scatter dir from RC hit nothing, the RC is considered the last vertex but idk why path.length receeds by 1.
    bool isRcVertexEscapedVertex = rcVertexLength == srcPathLength + 1;
    HitInfo srcRcPrevVertexHit = srcReservoir.rcPrevVertexHit.getHitInfo();
    HitInfo srcRcPrevPrevVertexHit = srcReservoir.rcPrevPrevVertexHit.getHitInfo();
    float3 srcRcPrevPrevVertexWo = srcReservoir.rcPrevPrevVertexWo;


    // Contrary to computeShiftedIntegrandHybrid(), we are guaranteed to not have dummyRC i.e. always actual ReconnectionData

    // Extract Reconnection Data
    HitInfo dstRcPrevVertexHit = rcData.rcPrevHit;
    HitInfo dstRcPrevPrevVertexHit = rcData.rcPrevPrevHit;
    float3 dstRcPrevVertexWo = rcData.rcPrevWo;
    float3 dstRcprevPrevVertexWo = rcData.rcPrevPrevWo;
    float3 Tp = rcData.pathThroughput;
    Tp = rcData.pathThroughput;

    if (all(Tp == float3(1.0f)))
    {
        print("----- Something is wrong", 0);
        print("----- dstRcPrevVertexHit", dstRcPrevVertexHit.isValid());
        print("----- dstRcPrevVertexWo", dstRcPrevVertexWo);
        print("----- Tp", Tp);
    }

    float3 rcTp = 1.f;

    // Check if the ReconnectionData is valid
    bool conditions = dstRcPrevVertexHit.isValid()
        && dstRcPrevPrevVertexHit.isValid()
        && any(Tp > 0.f)
        && (
            rcVertexLength <= srcPathLength
            || isRcVertexEscapedVertex
        );

    if (conditions)
    {
        print("---- Prep for Manifold Exploration", 0);
        // Prepare ingredients for Manifold Exploration

        // Load shading data: src and dst rcPrev, rcPrevPrev
        ShadingData dstRcPrevVertexSd = loadShadingData(dstRcPrevVertexHit, dstRcPrevVertexWo, false);
        ShadingData dstRcPrevPrevVertexSd = dstPrimarySd;
        ShadingData srcRcPrevPrevVertexSd = srcPrimarySd;

        if (rcVertexLength > 2)
        {
            dstRcPrevPrevVertexSd = loadShadingData(dstRcPrevPrevVertexHit, dstRcprevPrevVertexWo, false);
            srcRcPrevPrevVertexSd = loadShadingData(srcRcPrevPrevVertexHit, srcRcPrevPrevVertexWo, false);
        }

        ShadingData srcRcPrevVertexSd = loadShadingDataWithPrevVertexPosition(srcRcPrevVertexHit, srcRcPrevPrevVertexSd.posW, false);

        VertexData rcVertex = gScene.getVertexData(srcReservoir.rcVertexHit.getHitInfo().getTriangleHit());
        ShadingData rcVertexSd = loadShadingDataWithPrevVertexPosition(srcReservoir.rcVertexHit.getHitInfo(), srcRcPrevVertexSd.posW, false);

        // Check if need to trace temporal
        if (temporalUpdateForDynamicScene
            && kTemporalUpdateForDynamicScene
            && !isRcVertexEscapedVertex
        )
        {
            print("----- Trace temporal update", 0);
            static const int id = 0;
            gPathTracer.traceTemporalUpdate(dstRcPrevVertexSd, tempPathReservoir);
            // Copy the partially-updated reservoir data
            srcReservoir.rcVertexIrradiance[id] = tempPathReservoir.rcVertexIrradiance[id];
            srcReservoir.lightPdf = tempPathReservoir.lightPdf;
            srcReservoir.pathFlags.insertLightType(tempPathReservoir.pathFlags.lightType());
            srcReservoir.rcVertexWi[id] = tempPathReservoir.rcVertexWi[id];
        }

        // Compute normal derivatives
        float3 srcRcPrevDnDu, srcRcPrevDnDv;
        computeNormalDerivatives(srcRcPrevVertexHit, srcRcPrevVertexSd.V, srcRcPrevDnDu, srcRcPrevDnDv);

        float3 dstRcPrevDnDu, dstRcPrevDnDv;
        computeNormalDerivatives(dstRcPrevVertexHit, dstRcPrevVertexSd.V, dstRcPrevDnDu, dstRcPrevDnDv);


        print("----- Input to Manifold Exploration:", 0);
        print("------ rcVertexSd.posW", rcVertexSd.posW);
        print("------ srcRcPrevVertexSd.posW", srcRcPrevVertexSd.posW);
        print("------ srcRcPrevPrevVertexSd.posW", srcRcPrevPrevVertexSd.posW);

        print("\n------ dstRcPrevVertexSd.posW", dstRcPrevVertexSd.posW);
        print("------ dstRcPrevPrevVertexSd.posW", dstRcPrevPrevVertexSd.posW);

        print("\n------ dstRcPrevDnDu", dstRcPrevDnDu);
        print("------ dstRcPrevDnDv", dstRcPrevDnDv);
        print("------ srcRcPrevDnDu", srcRcPrevDnDu);
        print("------ srcRcPrevDnDv", srcRcPrevDnDv);

        // Manifold Exploration to find the correct x2
        HitInfo x2_new;
        bool isManifoldSuccess = walkManifoldDSD(
            rcVertexSd,
            dstRcPrevVertexHit,
            srcRcPrevPrevVertexSd,
            dstRcPrevPrevVertexSd,
            x2_new
        );

        print("----- Is update manifold success:", isManifoldSuccess);

        if (isManifoldSuccess)
        {
            // Update dstRcPrev as the new x2
            dstRcPrevVertexHit = x2_new;
            dstRcPrevVertexSd = loadShadingDataWithPrevVertexPosition(dstRcPrevVertexHit, dstRcPrevPrevVertexSd.posW, false);

            // Update dstRcPrev dndu, dndv after the exploration
            computeNormalDerivatives(dstRcPrevVertexHit, dstRcPrevVertexSd.V, dstRcPrevDnDu, dstRcPrevDnDv);

            print("------ Updated dstRcPrevVertex", 0);
            print("------ dstRcPrevVertexSd.posW", dstRcPrevVertexSd.posW);
            print("------ dstRcPrevDnDu", dstRcPrevDnDu);
            print("------ dstRcPrevDnDv", dstRcPrevDnDv);
        }


        float reconnectionJacobian = 1.f;

        // Finally, compute the shifted integrand and jacobian
        rcTp = computeShiftedIntegrandDSD(
            params,
            dstRcPrevVertexSd,
            dstRcPrevPrevVertexSd,
            dstRcPrevDnDu,
            dstRcPrevDnDv,
            srcRcPrevVertexSd,
            srcRcPrevPrevVertexSd,
            srcRcPrevDnDu,
            srcRcPrevDnDv,
            srcReservoir,
            reconnectionJacobian,
            evalVisibility,
        );

        dstJacobian *= reconnectionJacobian;

        // tempPathReservoir.cachedJacobian = srcReservoir.cachedJacobian;
        // tempPathReservoir.cachedJacobian[0] = 0.f;
        // tempPathReservoir.cachedJacobian[1] = 0.f;
        // tempPathReservoir.cachedJacobian[2] = 0.f;
    }
    else
    {
        print("---- Invalid Reconnection Data", 0);
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
float3 computeShiftedIntegrandReconnection(
    const RestirPathTracerParams params,
    inout float dstJacobian,
    ShadingData dstPrimarySd,
    ShadingData srcPrimarySd,
    inout PathReservoir srcReservoir,
    bool evalVisibility,
    bool useHybridShift=false,
    bool useCachedJacobian = false
)
{
    // useCachedJacobian = false;
    float3 dstCachedJacobian;
    dstJacobian = 0.f;

    int rcVertexLength = !useHybridShift ? 1 : srcReservoir.pathFlags.rcVertexLength();

    HitInfo rcVertexHit = srcReservoir.rcVertexHit.getHitInfo();
    float3 rcVertexIrradiance = srcReservoir.rcVertexIrradiance[0];
    float3 rcVertexWi = srcReservoir.rcVertexWi[0];
    bool rcVertexHitExists = rcVertexHit.isValid();

    bool isTransmission = srcReservoir.pathFlags.decodeIsTransmissionEvent(true);
    // Is prevRcVertex sampled outgoing direction from a specular lobe?
    // If yes, all samplers types (SampledBSDFFlags) allowed.
    // Otherwise, only diffuse reflection and transmission are allowed.
    uint allowedSampledTypes1 = getAllowedBSDFFlags(srcReservoir.pathFlags.decodeIsSpecularBounce(true));

    srcPrimarySd.posW = srcPrimarySd.computeNewRayOrigin(!isTransmission);
    dstPrimarySd.posW = dstPrimarySd.computeNewRayOrigin(!isTransmission);

    if (!rcVertexHitExists)
    {
        print("------ srcReservoir rcVertex does not exists", 0);
        float3 dstIntegrand = 0.f;

        // are we having an infinite light (evn map) as rcVertex?
        if (
            kUseMIS
            // Use light map
            && srcReservoir.pathFlags.lightType() == (uint)PathTracer::LightSampleType::EnvMap
            // path ends exactly before rcVertexLength (in this case is 15 for reservoir that doesn't have a valid rcVertex)
            && srcReservoir.pathFlags.pathLength() + 1 == rcVertexLength
            // And at the end did not sampled a light source
            && !srcReservoir.pathFlags.lastVertexNEE()
        )
        {

            print("------- use envMap as RC", 0);

            // Out going direction toward the env map.
            float3 wi = rcVertexWi;

            // Does the rcPrevVertex of the reconnection (dst) have a clear view
            // toward the env map
            bool isVisible = evalSegmentVisibility(dstPrimarySd.posW, wi, true); // test along a direction

            if (isVisible)
            {
                // reservoir pdf at rcPrev hit
                float srcPDF1 = useCachedJacobian ? srcReservoir.cachedJacobian.x : evalPdfBSDF(srcPrimarySd, wi);

                // reconnection pdf for every possible sampling cases at rcPrev hit
                float dstPDF1All;

                // reconnection pdf for allowed sampling cases that at rcPrev hit
                float dstPDF1 = evalPdfBSDF(dstPrimarySd, wi, dstPDF1All, allowedSampledTypes1);

                // reconnection F term
                float3 dstF1 = evalBSDFCosine(dstPrimarySd, wi, allowedSampledTypes1);

                // Compute MIS weight of reconnection and reservoir at the prevRc
                // srcReservoir.lightPdf is NEE light pdf.
                float misWeight = PathTracer::evalMIS(1, dstPDF1All, 1, srcReservoir.lightPdf);//   dstPDF1 / (dstPDF1 + srcReservoir.lightPdf);


                dstIntegrand = (dstF1 / dstPDF1) * misWeight * rcVertexIrradiance;

                // Random Replay Jacobian Shift in Primary Sample Sapce[GRIS Eq. 53]
                dstJacobian = dstPDF1 / srcPDF1;

                // Update cache
                dstCachedJacobian.x = dstPDF1;

                print("------- jacobian", dstJacobian);
                print("------- integrand", dstIntegrand);
            }
        }

        if (useCachedJacobian)
            srcReservoir.cachedJacobian = dstCachedJacobian;

        // fill in rcVertex0 information
        if (isJacobianInvalid(dstJacobian))
            dstJacobian = 0.f;

        if (any(isnan(dstIntegrand) || isinf(dstIntegrand)))
            return 0.f;

        return dstIntegrand;
    }

    print("------ rcVertex does exists", 0);

    bool isDelta1 = srcReservoir.pathFlags.decodeIsDeltaEvent(true);
    bool isDelta2 = srcReservoir.pathFlags.decodeIsDeltaEvent(false);

    // delta bounce before/after rcVertex (if isRcVertexNEE, deltaAfterRc won't be set)
    print("------ path flags ", srcReservoir.pathFlags.flags);
    if (isDelta1 || isDelta2)
    {
        print("------ is delta bounce around RC", (isDelta1 ? 1 : 0) | (isDelta2 ? 2 : 0));
        return 0.f;
    }

    print("------ is not delta bounce around RC", 0);

    bool isRcVertexFinal = srcReservoir.pathFlags.pathLength() == rcVertexLength;
    bool isRcVertexEscapedVertex = srcReservoir.pathFlags.pathLength() + 1 == rcVertexLength && !srcReservoir.pathFlags.lastVertexNEE();
    bool isRcVertexNEE = isRcVertexFinal && srcReservoir.pathFlags.lastVertexNEE();

    // dstPrimary is either primary or dstRcPrev
    ShadingData rcVertexSd = loadShadingDataWithPrevVertexPosition(rcVertexHit, dstPrimarySd.posW, false);

    // need to evaluate source PDF of BSDF sampling
    // rcVertexSd.V is dir from rc to prev
    // norm(dstPrev -> rc)
    float3 dstConnectionV = -rcVertexSd.V; // direction point from dst primary hit point to reconnection vertex
    // norm(srcPrev -> rc)
    float3 srcConnectionV = normalize(rcVertexSd.posW - srcPrimarySd.posW);

    // dstPrev -> rc
    float3 shiftedDisp = rcVertexSd.posW - dstPrimarySd.posW;
    // (dstPrev -> rc)^2
    float shifted_dist2 = dot(shiftedDisp, shiftedDisp);
    // angle between rc.N and dir to dstPrev
    float shifted_cosine = abs(dot(rcVertexSd.faceN, -dstConnectionV));

    if ((params.localStrategyType & (uint)LocalStrategy::DistanceCondition) && useHybridShift)
    {
        bool isFarField = sqrt(shifted_dist2) >= params.nearFieldDistance;
        if (!isFarField)
        {
            // Only trigger for Spaitial reuse
            print("------ Rejected for being too close", 0);
            return 0.f;
        }
    }

    print("------ Compute Jacobian", 0);

    // Cache the dst terms
    dstCachedJacobian.z = shifted_cosine / shifted_dist2;

    // Compute Jacobian: path Reconnection shift [Gradient-domain path tracing, Eq. 13]
    //  det((partial(offset incident)/partial(base incident)))
    float Jacobian;

    print("------ If were to use cached Jacobian", dstCachedJacobian.z / srcReservoir.cachedJacobian.z);
    if (useCachedJacobian)
    {
        Jacobian = dstCachedJacobian.z / srcReservoir.cachedJacobian.z;
    }
    else
    {
        // srcPrev -> rc
        float3 originalDisp = rcVertexSd.posW - srcPrimarySd.posW;
        // (srcPrev -> rc)^2
        float original_dist2 = dot(originalDisp, originalDisp);
        // angle between rc.N and dir to srcPrev
        float original_cosine = abs(dot(rcVertexSd.faceN, -srcConnectionV));

        // Jac = (dstAngle / dstDist^2) * (srcDist^2 / srcAngle)
        //     = (dstAngle / srcAngle) * (srcDist^2 / dstDist^2)
        Jacobian = dstCachedJacobian.z * original_dist2 / original_cosine;
        // TODO: Investigate why is this always 1???

        print("------ src G':", original_cosine / original_dist2);
        print("------ dst G':", shifted_cosine / shifted_dist2);
    }

    print("------- Jacobian Reconnection shift:", Jacobian);

    if (isJacobianInvalid(Jacobian)) return 0.f;

    // Compute Jacobian: Random Replay of choosing same RC [GRIS, Eq. 54]
    // assuming BSDF sampling
    assert(kUseBSDFSampling);

    // assuming bsdf sampling
    float dstPDF1All = 0.f;
    // pdf of sampling dir from dstRcPrev -> rc
    float dstPDF1 = evalPdfBSDF(dstPrimarySd, dstConnectionV, dstPDF1All, allowedSampledTypes1);

    dstCachedJacobian.x = dstPDF1;
    // pdf of sampling dir from srcRcPrev -> rc
    float srcPDF1 = useCachedJacobian ? srcReservoir.cachedJacobian.x : evalPdfBSDF(srcPrimarySd, srcConnectionV, allowedSampledTypes1); //

    // Adding in Random Replay Jacobian Shift in Primary Sample Sapce[GRIS Eq. 54]
    Jacobian *= dstPDF1 / srcPDF1;

    print("------- Jacobian Random Replay:", dstPDF1 / srcPDF1);

    if (isJacobianInvalid(Jacobian)) return 0.f;

    print("------- total Jacobian det:", Jacobian);

    print("------ Compute reconnection integrand", 0);

    float3 dstF1 = evalBSDFCosine(dstPrimarySd, dstConnectionV, allowedSampledTypes1);

    float dstRcVertexScatterPdfAll = 0.f;
    // pdf of sampling dir from rc -> srcRcWi or srcRcLight if NEE
    float dstPDF2 = 1.f;
    float dstRcVertexScatterPdf = 1.f;
    float srcRcVertexScatterPdf = 1.f;

    uint allowedSampledTypes2 = isRcVertexNEE ? -1 : getAllowedBSDFFlags(srcReservoir.pathFlags.decodeIsSpecularBounce(false));

    if (!isRcVertexEscapedVertex)
    {
        // assuming bsdf sampling
        // pdf of sampling dir(rc -> srcRcWi)
        dstRcVertexScatterPdf = evalPdfBSDF(rcVertexSd, rcVertexWi, dstRcVertexScatterPdfAll, allowedSampledTypes2);

        dstCachedJacobian.y = dstRcVertexScatterPdf;

        // pdf of sampling dir(rc -> RcWi) with prior from srcPrevRc
        srcRcVertexScatterPdf = useCachedJacobian ? srcReservoir.cachedJacobian.y : evalPdfBSDFWithV(rcVertexSd, -srcConnectionV, rcVertexWi, allowedSampledTypes2);

        if (isRcVertexNEE)
            dstPDF2 = srcReservoir.lightPdf;
        else
            dstPDF2 = dstRcVertexScatterPdf;
    }

    float3 dstF2 = 1.f;

    // base path continue tracing after rc
    if (!isRcVertexEscapedVertex)
        dstF2 = evalBSDFCosine(rcVertexSd, rcVertexWi, allowedSampledTypes2);

    // connection point behind surface
    if (all(dstF1 == 0.f) || all(dstF2 == 0.f)) return 0.f;

    print("------ valid connection points", 0);

    // Contribution after RC
    float3 dstIntegrandNoF1 = dstF2 / dstPDF2 * rcVertexIrradiance;
    // Total contribution = contribution before RC + contribution after RC
    float3 dstIntegrand = dstF1 / dstPDF1 * dstIntegrandNoF1; // TODO: might need to reevaluate Le for changing emissive lights

    if (isRcVertexEscapedVertex)
    {
        // MIS between getting to RC via BSDF sampling and NEE
        float misWeight = PathTracer::evalMIS(1, dstPDF1All, 1, srcReservoir.lightPdf);// dstPDF1 / (srcReservoir.lightPdf + dstPDF1);

        dstIntegrand *= misWeight;
    }

    // If path ends at RC then add in MIS weight
    if (isRcVertexFinal && kUseMIS)
    {
        // If at the ending RC, and base path sampled the light by EnvMap or Emissive means then adding in the contribution
        if (srcReservoir.pathFlags.lightType() != (uint)PathTracer::LightSampleType::Analytic) // TODO: optimize way this check
        {
            float lightPdf = srcReservoir.lightPdf;
            float misWeight = PathTracer::evalMIS(1, isRcVertexNEE ? lightPdf : dstRcVertexScatterPdfAll, 1, isRcVertexNEE ? dstRcVertexScatterPdfAll : lightPdf);
            dstIntegrand = dstIntegrand * misWeight;
            dstIntegrandNoF1 = dstIntegrandNoF1 * misWeight;

            // If at the ending RC does not sampled direct light then it
            //  go to sample EnvMap therefore adding in Jacobian of getting this sampling direction in offset path compare to in base path
            if (!isRcVertexNEE)
                // dstRcVertexScatterPdf: pdf of sampling dir(rc -> rcWi)
                // srcRcVertexScatterPdf: pdf of sampling dir(rc -> rcWi) with prior from srcPrevRc
                // prob of sampling continuing direction on offset based on base
                Jacobian *= dstRcVertexScatterPdf / srcRcVertexScatterPdf;
        }
    }

    // need to account for non-identity jacobian due to BSDF sampling
    if (!isRcVertexFinal && !isRcVertexEscapedVertex)
    {
        // dstRcVertexScatterPdf: pdf of sampling dir(rc -> rcWi)
        // srcRcVertexScatterPdf: pdf of sampling dir(rc -> rcWi) with prior from srcPrevRc
        // prob of sampling continuing direction on offset path based on base's
        Jacobian *= dstRcVertexScatterPdf / srcRcVertexScatterPdf;
    }

    print("------ Jacobian with RC Wi:", Jacobian);

    if (isJacobianInvalid(Jacobian))
        return 0.f;


    // Evaluate visibility: vertex 1 <-> vertex 2 (reconnection vertex).
    if (evalVisibility)
    {
        bool isVisible = evalSegmentVisibility(dstPrimarySd.posW, rcVertexSd.posW);
        if (!isVisible)
            return 0.f;
    }

    print("------ Visible", 0);

    if (any(isnan(dstIntegrand) || isinf(dstIntegrand))) return 0.f;

    if (params.rejectShiftBasedOnJacobian)
    {
        if (Jacobian > 0.f && (max(Jacobian, 1 / Jacobian) > 1 + params.jacobianRejectionThreshold))
        {
            print("------- Reject shift based on Jacobian", 0);
            // discard based on Jacobian (unbiased)
            Jacobian = 0.f;
            dstIntegrand = 0.f;
        }
    }

    dstJacobian = Jacobian;

    // Final update cached Jacobian
    if (useCachedJacobian)
    {
        srcReservoir.cachedJacobian = dstCachedJacobian;
    }

    print("------ dstIntegrand", dstIntegrand);

    return dstIntegrand;
}


bool shiftAndMergeReservoir(const RestirPathTracerParams params, bool temporalUpdateForDynamicScene, inout float dstJacobian, const PackedHitInfo dstPrimaryHitPacked, const ShadingData dstPrimarySd, inout PathReservoir dstReservoir,
                                    const ShadingData srcPrimarySd, const PathReservoir srcReservoir, ReconnectionData rcData, bool evalVisibility, inout SampleGenerator sg, bool isSpatialReuse, float misWeight = 1.f, bool forceMerge = false)
{
    // Merge srcReservoir into dstReservoir
    PathReservoir tempPathReservoir = srcReservoir;

    // Compute srcReservoir shiftedIntegrand with its reconnection data
    float3 dstIntegrand = computeShiftedIntegrand_(params, dstJacobian, dstPrimaryHitPacked, dstPrimarySd, srcPrimarySd,
    tempPathReservoir, rcData, sg, evalVisibility, false, temporalUpdateForDynamicScene);

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

