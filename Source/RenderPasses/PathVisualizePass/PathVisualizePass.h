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
#pragma once
#include "Falcor.h"
#include "Utils/Debug/PixelDebug.h"
#include "DebugPathDataType.slang";


using namespace Falcor;

class PathVisualizePass : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<PathVisualizePass>;

    /** Create a new render pass object.
        \param[in] pRenderContext The render context.
        \param[in] dict Dictionary of serialized parameters.
        \return A new object, or an exception is thrown if creation failed.
    */
    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    virtual std::string getDesc() override;
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pRenderContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override;
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override;

private:
    PathVisualizePass(const Dictionary& dict);

    void createRasterPass();
    void filterCopyPathData();
    void updateRenderData();

    static glm::mat4 computeTransformMatToLineSegment(float3 lineBegin, float3 lineEnd);

    uint2 mFixedOutputSize = { 512, 512 };    //< Output size in pixels when 'Fixed' size is selected.

    Sampler::SharedPtr mpPointSampler;

    PixelDebug::SharedPtr mpPixelDebug;

    Scene::SharedPtr mpScene;

    uint2 mSelectedCursorPosition = uint2(0);

    //  Path data

    struct PathDataBundle
    {
        DebugPathData basePath;
        DebugPathData temporalCentralPath;
        DebugPathData temporalTemporalPath;
        DebugPathData spatialCentralPath;
        DebugPathData spatialNeighborPath;

        bool isFullyComplete = false;   // signify that the bundle is a "complete" package that satisfies all conditions
        bool isPartiallyComplete = false;  // have atleast a base path

        void init()
        {
            basePath.init();
            temporalCentralPath.init();
            temporalTemporalPath.init();
            spatialCentralPath.init();
            spatialNeighborPath.init();
            isFullyComplete = false;
            isPartiallyComplete = false;
        }

        void clear()
        {
            basePath.vertexCount = 0;
            temporalCentralPath.vertexCount = 0;
            temporalTemporalPath.vertexCount = 0;
            spatialCentralPath.vertexCount = 0;
            spatialNeighborPath.vertexCount = 0;
            isFullyComplete = false;
            isPartiallyComplete = false;
        }

        void deepCopy(PathDataBundle srcPathDataBundle)
        {
            basePath.deepCopy(srcPathDataBundle.basePath);
            temporalCentralPath.deepCopy(srcPathDataBundle.temporalCentralPath);
            temporalTemporalPath.deepCopy(srcPathDataBundle.temporalTemporalPath);
            spatialCentralPath.deepCopy(srcPathDataBundle.spatialCentralPath);
            spatialNeighborPath.deepCopy(srcPathDataBundle.spatialNeighborPath);
            isFullyComplete = srcPathDataBundle.isFullyComplete;
            isPartiallyComplete = srcPathDataBundle.isPartiallyComplete;
        }

    };

    PathDataBundle mCurrentFramePathBundle;
    PathDataBundle mBuildingPathBundle;
    PathDataBundle mReservedPathBundle;
    PathDataBundle mRenderedPathBundle;

    //  Rendering

    RasterPass::SharedPtr mpRasterPass;

    bool mRecreateRasterPass = true;

    Buffer::SharedPtr mpVertexBuffer;
    Buffer::SharedPtr mpIndexBuffer;

    uint mTotalIndices = 0;

    //  Options

    bool mIsDisplayBasePath = true;
    bool mIsDisplayNEESegments = true;
    bool mIsDisplayTemporalCentralPath = true;
    bool mIsDisplayTemporalTemporalPath = true;
    bool mIsDisplaySpatialCentralPath = true;
    bool mIsDisplaySpatialNeighborPath = true;
    float mRayWidth = 0.014;
};
