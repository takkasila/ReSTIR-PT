/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/
import Params;
import Shift;
import Scene.HitInfo;
import PathReservoir;
import PathTracer;
import Scene.Scene;
import Utils.Debug.PixelDebug;
import Rendering.Utils.PixelStats;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Math.Ray;
import LoadShadingData;
import Rendering.Materials.MaterialShading;
import DebugPathDataType;

/** TODO.
*/
struct PathReusePass
{
    static const uint kNeighborOffsetCount = NEIGHBOR_OFFSET_COUNT;
    static const uint kNeighborOffsetMask = kNeighborOffsetCount - 1;

    // Resources
    RestirPathTracerParams params;                        ///< Runtime parameters.

    Texture2D<PackedHitInfo> vbuffer;                     ///< Fullscreen V-buffer for the primary hits.
    Texture2D<PackedHitInfo> temporalVbuffer;
    Texture2D<float2> motionVectors;

    RWTexture2D<float4> outputColor;                      ///< Output resolved color.
    RWStructuredBuffer<PathReservoir> outputReservoirs;    // reservoir for next pass
    RWStructuredBuffer<PathReservoir> temporalReservoirs;  // reservoir from previous frame
    StructuredBuffer<PixelReconnectionData> reconnectionDataBuffer;

    RWStructuredBuffer<DebugManifoldWalk> debugManifoldWalk1Buffer;
    RWStructuredBuffer<DebugManifoldWalk> debugManifoldWalk2Buffer;

    RWStructuredBuffer<DebugPathData> temporalFinalReservoir;

    Texture2D<float4> directLighting;
    bool useDirectLighting;

    int  gNumSpatialRounds;

    bool gIsLastRound;
    bool gEnableTemporalReprojection;

    float gTemporalHistoryLength;
    bool gNoResamplingForTemporalReuse;

    bool isValidPackedHitInfo(PackedHitInfo packed)
    {
        return packed.x != 0;
    }

    bool isValidScreenRegion(int2 pixel) { return all(pixel >= 0 && pixel < params.frameDim); }

    ShadingData getPixelShadingData(int2 pixel, out PackedHitInfo PrimaryHitPacked)
    {
        ShadingData sd = {};
        Ray ray = gScene.camera.computeRayPinhole(pixel, params.frameDim);
        PrimaryHitPacked = vbuffer[pixel];
        if (isValidPackedHitInfo(PrimaryHitPacked))
        {
            HitInfo PrimaryHit; PrimaryHit.unpack(PrimaryHitPacked);
            sd = loadShadingData(PrimaryHit, -ray.dir, true);
        }
        return sd;
    }

    ShadingData getPixelTemporalShadingData(int2 pixel, out PackedHitInfo PrimaryHitPacked)
    {
        ShadingData sd = {};
        Ray ray = gScene.camera.computeRayPinholePrevFrame(pixel, params.frameDim);
        PrimaryHitPacked = temporalVbuffer[pixel];
        if (isValidPackedHitInfo(PrimaryHitPacked))
        {
            HitInfo PrimaryHit; PrimaryHit.unpack(PrimaryHitPacked);
            sd = loadShadingData(PrimaryHit, -ray.dir, true);
        }
        return sd;
    }

    /*
    * Build a reservoir of paths whose suffix can be merged to a pixel's prefix.
    * This method is optimized for small windows such that we can run the entire
    * algorithm in one pass.
    */
    void ReSTIR(const uint2 pixel)
    {
        bool isDebug = all(getSelctedPixelLog() == pixel);

        var sg = TinyUniformSampleGenerator(pixel, (kCandidateSamples + 1 + gNumSpatialRounds) * params.seed + kCandidateSamples);

        // Compute offset into per-sample buffers. All samples are stored consecutively at this offset.
        const uint centralOffset = params.getReservoirOffset(pixel);
        PathReservoir dstReservoir = outputReservoirs[centralOffset];
        float currentM = dstReservoir.M;
        float3 initialColor = dstReservoir.F * dstReservoir.weight;

        if (ReSTIRMISKind(kTemporalReSTIRMISKind) == ReSTIRMISKind::Talbot)
        {
            dstReservoir.init();
        }
        else
        {
            dstReservoir.prepareMerging();
        }

        if (isDebug)
            checkReservoirValidity(dstReservoir);

        PackedHitInfo centralPrimaryHitPacked;
        ShadingData centralPrimarySd = getPixelShadingData(pixel, centralPrimaryHitPacked);
        if (!isValidPackedHitInfo(centralPrimaryHitPacked)) return;

        float3 color = 0.f;
        ReconnectionData dummyRcData;
        dummyRcData.Init();

        {
            bool chooseCurrent = true;

            // color averaging test

            // fetch temporal reservoir
            float2 motionVector = motionVectors[pixel];
            int2 prevPixel = pixel;

            bool foundTemporalSurface = true;

            if (gEnableTemporalReprojection)
            {
                prevPixel = pixel + motionVector * params.frameDim + (sampleNext2D(sg) * 1.f - 0.f);
            }

            if (!isValidScreenRegion(prevPixel) || !foundTemporalSurface) return;

            PackedHitInfo temporalPrimaryHitPacked;
            ShadingData temporalPrimarySd = getPixelTemporalShadingData(prevPixel, temporalPrimaryHitPacked);
            if (!isValidPackedHitInfo(temporalPrimaryHitPacked)) return;

            bool doTemporalUpdateForDynamicScene = kTemporalUpdateForDynamicScene;

            PathReservoir temporalReservoir = temporalReservoirs[params.getReservoirOffset(prevPixel)];

            temporalReservoir.M = min(gTemporalHistoryLength * currentM, temporalReservoir.M);

            if (isDebug)
                checkReservoirValidity(temporalReservoir);

            float dstJacobian;

            ///////////////////
            /// TALBOT RMIS ///
            ///////////////////
            if (ReSTIRMISKind(kTemporalReSTIRMISKind) == ReSTIRMISKind::Talbot)
            {
                // Note:
                //  - this is the default option in Temporal Reuse
                //  - dstReservoir is base path's
                //  - temporalReservoir is offset path's

                static const int curSampleId = -1;
                static const int prevSampleId = 0;

                DebugManifoldWalk debugManifoldWalk1;
                DebugManifoldWalk debugManifoldWalk2;

                debugManifoldWalk1.init();
                debugManifoldWalk2.init();
                if (isDebug)
                {
                    debugManifoldWalk1Buffer[0] = debugManifoldWalk1;
                    debugManifoldWalk2Buffer[0] = debugManifoldWalk2;
                }

                print("{loopSampleId i", 0);
                for (int i = curSampleId; i <= prevSampleId; i++)
                {
                    print("{i", i);

                    float p_sum = 0;
                    float p_self = 0;

                    PathReservoir tempDstReservoir = dstReservoir;

                    bool possibleToBeSelected = false;

                    if (i == curSampleId)
                    {
                        // Connecting to itself. No reconnection data.
                        tempDstReservoir = outputReservoirs[centralOffset];
                        dstJacobian = 1.f;
                        possibleToBeSelected = tempDstReservoir.weight > 0;
                    }
                    else
                    {
                        ReconnectionData rcData;
                        if (
                            (
                                // Is doing hybrid shift mapping
                                ShiftMapping(kShiftStrategy) == ShiftMapping::Hybrid
                                || ShiftMapping(kShiftStrategy) == ShiftMapping::HybridShiftSMS
                            )
                            //  And the retrace was successful.
                            //  Why don't we just check the ReconnectionData directly? Is it because of doing PathReuse without ReSTIR?
                            && temporalReservoir.pathFlags.rcVertexLength() > 1
                            // Also, should we check if hitinfo is valid instead?
                        )
                        {
                            print("Have RC 1", 0);
                            rcData = reconnectionDataBuffer[centralOffset].data[1];
                        }
                        else
                        {
                            print("Dummy RC 1", 0);
                            rcData = dummyRcData;
                        }

                        //  Compute shifted integrand of temporal reservoir and it's reconnection data.
                        //  Then try to merge temporal reservoir into central reservoir
                        //  If successful, the new value will be saved in the
                        //  temporary reservoir "tempDstReservoir" to be used
                        //  in computeShiftedIntegrand()

                        print("shiftAndMergeReservoir", 0);
                        possibleToBeSelected = shiftAndMergeReservoir(
                            params,
                            doTemporalUpdateForDynamicScene,
                            dstJacobian,
                            centralPrimaryHitPacked,
                            centralPrimarySd,
                            tempDstReservoir,
                            temporalPrimarySd,
                            temporalReservoir,
                            rcData,
                            true,
                            sg,
                            debugManifoldWalk1,
                            false,
                            1.f,
                            true    // "true" means hypothetically selected as the sample
                        );

                        if(isDebug)
                            checkReservoirValidity(tempDstReservoir);
                    }

                    print("possibleToBeSelected:", possibleToBeSelected);

                    if (possibleToBeSelected)
                    {
                        print("{loopSampleId j", 0);
                        for (int j = curSampleId; j <= prevSampleId; ++j)
                        {
                            print("{j", j);

                            if (j == curSampleId)
                            {
                                float cur_p = PathReservoir::toScalar(tempDstReservoir.F) * currentM;
                                p_sum += cur_p;
                                if (i == curSampleId) p_self = cur_p;
                            }
                            else
                            {
                                if (i == j)
                                {
                                    p_self = PathReservoir::toScalar(temporalReservoir.F) / dstJacobian * temporalReservoir.M;
                                    p_sum += p_self;
                                    print("}j", j);
                                    continue;
                                }

                                float p_ = 0.f;
                                float tneighborJacobian;

                                ReconnectionData rcData;
                                if (
                                    (
                                        // Is doing hybrid shift mapping
                                        ShiftMapping(kShiftStrategy) == ShiftMapping::Hybrid
                                        || ShiftMapping(kShiftStrategy) == ShiftMapping::HybridShiftSMS
                                    )
                                    && tempDstReservoir.pathFlags.rcVertexLength() > 1
                                )
                                {
                                    print("Have RC 2", 0);
                                    rcData = reconnectionDataBuffer[centralOffset].data[0];
                                }
                                else
                                {
                                    rcData = dummyRcData;
                                    print("Dummy RC 2", 0);
                                }

                                print("computeShiftedIntegrand", 0);
                                float3 tneighborIntegrand = computeShiftedIntegrand(
                                    params,
                                    tneighborJacobian,
                                    temporalPrimaryHitPacked,
                                    temporalPrimarySd,
                                    centralPrimarySd,
                                    tempDstReservoir,
                                    rcData,
                                    sg,
                                    true,
                                    debugManifoldWalk2,
                                    true
                                ); //usePrev
                                p_ = PathReservoir::toScalar(tneighborIntegrand) * tneighborJacobian;
                                p_sum += p_ * temporalReservoir.M;

                            }

                            print("}j", j);
                        }
                        print("}loopSampleId j", 0);
                    }

                    float misWeight = p_sum == 0.f ? 0.f : p_self / p_sum;

                    PathReservoir neighborReservoir;
                    if (i == curSampleId)
                        neighborReservoir = tempDstReservoir;
                    else
                        neighborReservoir = temporalReservoir;

                    // Finally merge into base reservoir (dstReservoir)
                    mergeReservoirWithResamplingMIS(
                        params,
                        tempDstReservoir.F,
                        dstJacobian,
                        dstReservoir,
                        tempDstReservoir,
                        neighborReservoir,
                        sg,
                        false,
                        misWeight
                    );

                    if (isDebug)
                        checkReservoirValidity(dstReservoir);

                    print("}i", i);
                }
                print("}loopSampleId i", 0);

                if (dstReservoir.weight > 0)
                {
                    dstReservoir.finalizeGRIS();
                }

                if (isDebug)
                {
                    debugManifoldWalk1Buffer[0] = debugManifoldWalk1;
                    debugManifoldWalk2Buffer[0] = debugManifoldWalk2;
                }
            }
            /////////////////////
            /// CONSTANT RMIS ///
            /////////////////////
            else
            {
                float chosen_Jacobian = 1.f;

                bool selected;

                if (gNoResamplingForTemporalReuse)
                    selected = mergeReservoirNoResampling(params, dstReservoir, temporalReservoir, sg);
                else
                {
                    DebugManifoldWalk debugManifoldWalk;
                    selected = shiftAndMergeReservoir(
                        params,
                        doTemporalUpdateForDynamicScene,
                        dstJacobian,
                        centralPrimaryHitPacked,
                        centralPrimarySd,
                        dstReservoir,
                        temporalPrimarySd,
                        temporalReservoir,
                        dummyRcData,
                        true,
                        sg,
                        debugManifoldWalk,
                        false
                    );
                }

                if (selected)
                {
                    chooseCurrent = false;
                    chosen_Jacobian = dstJacobian;
                }

                // MIS weight
                if (dstReservoir.weight > 0 && ReSTIRMISKind(kTemporalReSTIRMISKind) != ReSTIRMISKind::ConstantBiased && !gNoResamplingForTemporalReuse)
                {
                    if (chooseCurrent) // need to know prev pdf
                    {
                        float count = currentM;
                        float prefixJacobian = 1.f;

                        // compute two shifts

                        int shiftCount = 1;
                        float prefix_approxPdf;

                        DebugManifoldWalk debugManifoldWalk;

                        float3 prefixIntegrand = computeShiftedIntegrand(
                            params,
                            prefixJacobian,
                            temporalPrimaryHitPacked,
                            temporalPrimarySd,
                            centralPrimarySd,
                            dstReservoir,
                            dummyRcData,
                            sg,
                            true,
                            debugManifoldWalk,
                            true
                        ); //usePrev // determinisitically select the reconnection shift
                        prefix_approxPdf = PathReservoir::toScalar(prefixIntegrand) * prefixJacobian;

                        if (prefix_approxPdf > 0.f)
                        {
                            count += temporalReservoir.M;
                        }

                        float misWeight = 1.f / max(1.f, count);
                        if (ReSTIRMISKind(kSpatialReSTIRMISKind) == ReSTIRMISKind::Constant)
                            misWeight = PathReservoir::toScalar(dstReservoir.F) / (PathReservoir::toScalar(dstReservoir.F) * currentM + prefix_approxPdf * temporalReservoir.M);
                        dstReservoir.weight *= dstReservoir.M * misWeight;
                    }
                    else // have already computed everything
                    {
                        float count = currentM;

                        // compute the other shift
                        if (ReSTIRMISKind(kSpatialReSTIRMISKind) == ReSTIRMISKind::Constant)
                        {
                            float sum_pdf = PathReservoir::toScalar(temporalReservoir.F) / chosen_Jacobian;

                            float misWeight = (PathReservoir::toScalar(temporalReservoir.F) / chosen_Jacobian) / (sum_pdf * temporalReservoir.M + PathReservoir::toScalar(dstReservoir.F) * currentM);

                            if (ReSTIRMISKind(kSpatialReSTIRMISKind) == ReSTIRMISKind::ConstantBinary)
                            {
                                misWeight = 1.f / max(1.f, count);
                            }
                            dstReservoir.weight *= dstReservoir.M * misWeight;
                        }
                    }
                }

                dstReservoir.finalizeRIS();
            }

            if (dstReservoir.weight < 0.f || isinf(dstReservoir.weight) || isnan(dstReservoir.weight))
                dstReservoir.weight = 0.f;

            if (isDebug)
                checkReservoirValidity(dstReservoir);

            outputReservoirs[centralOffset] = dstReservoir;

            if (isDebug)
            {
                HitInfo rcVertexHitInfo = dstReservoir.rcVertexHit.getHitInfo();
                HitInfo rcPrevVertexHitInfo = dstReservoir.rcPrevVertexHit.getHitInfo();
                HitInfo rcPrevPrevVertexHitInfo = dstReservoir.rcPrevPrevVertexHit.getHitInfo();

                uint vertexCount = 0;
                if (rcVertexHitInfo.isValid())
                {
                    VertexData rcVertex = gScene.getVertexData(rcVertexHitInfo.getTriangleHit());
                    temporalFinalReservoir[0].vertices[vertexCount] = float4(rcVertex.posW, 1);
                    vertexCount++;

                    if (rcPrevVertexHitInfo.isValid())
                    {
                        VertexData rcPrevVertex = gScene.getVertexData(rcPrevVertexHitInfo.getTriangleHit());
                        temporalFinalReservoir[0].vertices[vertexCount] = float4(rcPrevVertex.posW, 1);
                        vertexCount++;

                        if (rcPrevPrevVertexHitInfo.isValid())
                        {
                            VertexData rcPrevPrevVertex = gScene.getVertexData(rcPrevPrevVertexHitInfo.getTriangleHit());
                            temporalFinalReservoir[0].vertices[vertexCount] = float4(rcPrevPrevVertex.posW, 1);
                            vertexCount++;
                        }
                    }
                }
                temporalFinalReservoir[0].vertexCount = vertexCount;
                print("Output Temporal Final Reservoir", vertexCount);
            }

            color = dstReservoir.F * dstReservoir.weight;

            print("dstReservoir.F", dstReservoir.F);
            print("dstReservoir.weight", dstReservoir.weight);
            print("color", color);

            if (dstReservoir.weight > 10.f)
            {
                print("Alert! High weight", dstReservoir.weight);
            }
            if (any(color > 10.f))
            {
                print("Alert! High color value", color);
            }


            if (gIsLastRound)
            {
                if (useDirectLighting)
                {
                    color += directLighting[pixel].rgb;
                    if (any(isnan(color) || isinf(color))) color = 0.f;
                }

                outputColor[pixel] += float4(color / kSamplesPerPixel, 1.f);

                print("outputColor[pixel]", outputColor[pixel]);
            }
        }

    }

    /** Entry point for merge pass.
        \param[in] pixel Pixel coordinates.
    */
    void execute(const uint2 _pixel)
    {
        const uint2 pixel = _pixel;

        if (any(pixel >= params.frameDim)) return;

        printSetPixel(pixel);
        logSetPixel(pixel);

        print("{Temporal Reuse", 0);
        ReSTIR(pixel);
        print("}Temporal Reuse", 0);
    }
};

cbuffer CB
{
    PathReusePass gPathReusePass;
}


[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gPathReusePass.execute(dispatchThreadId.xy);
}
