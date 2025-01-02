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
#include "PathVisualizePass.h"


namespace
{
    const char kDesc[] = "Visualize a path (vertices) on top of an input image with a depth buffer.";
    const char kInputImg[] = "inputImg";
    const char kOutputImg[] = "outputImg";

    const char kPathVisualizeShaderPassFile[] = "RenderPasses/PathVisualizePass/PathVisualizePass.ps.slang";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("PathVisualizePass", kDesc, PathVisualizePass::create);
}

PathVisualizePass::SharedPtr PathVisualizePass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new PathVisualizePass(dict));
    return pPass;
}

PathVisualizePass::PathVisualizePass(const Dictionary& dict)
{
    // Future handle of dict
    createPathVisualizeShaderPass();
    

    // Create sampler(s)
    Sampler::Desc samplerDesc;
    samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point);
    mpPointSampler = Sampler::create(samplerDesc);

    return;
}

void PathVisualizePass::createPathVisualizeShaderPass()
{
    mpPathVisualizeShaderPass = FullScreenPass::create(kPathVisualizeShaderPassFile);
}


std::string PathVisualizePass::getDesc() { return kDesc; }

Dictionary PathVisualizePass::getScriptingDictionary()
{
    return Dictionary();
}

RenderPassReflection PathVisualizePass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;

    reflector.addInput(kInputImg, "Input image");

    reflector.addOutput(kOutputImg, "Output image");

    // Get default texture size
    //const uint2 size = RenderPassHelpers::calculateIOSize()

    // const uint2 size = RenderPassHelpers::calculateIOSize(RenderPassHelpers::IOSize::Default, mFixedOutputSize, compileData.defaultTexDims);

    // auto& output = reflector.addOutput(kOutputImg, "Output image").texture2D(size.x, size.y);
    // if (mOutputFormat != ResourceFormat::Unknown)
    // {
    //     output.format(mOutputFormat);
    // }

    return reflector;
}

void PathVisualizePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData["src"]->asTexture();

    auto pInputImg = renderData[kInputImg]->asTexture();
    auto pOutputImg = renderData[kOutputImg]->asTexture();
    assert(pInputImg && pOutputImg);

    Fbo::SharedPtr pFbo = Fbo::create();
    pFbo->attachColorTarget(pOutputImg, 0);

    if (mRecreatePathVisualizeShaderPass)
    {
        createPathVisualizeShaderPass();
        mUpdatePathVisualizeShaderPass = true;
        mUpdatePathVisualizeShaderPass = false;
    }

    if (mUpdatePathVisualizeShaderPass)
    {
        // Update params here in future

        mUpdatePathVisualizeShaderPass = false;
    }

    mpPathVisualizeShaderPass["gColorTex"] = pInputImg;
    //mpPathVisualizeShaderPass["gDepthTex"] = ...;
    mpPathVisualizeShaderPass["gColorSampler"] = mpPointSampler;

    // Run the shader pass
    mpPathVisualizeShaderPass->execute(pRenderContext, pFbo);

}

void PathVisualizePass::renderUI(Gui::Widgets& widget)
{
    widget.text("It's a me, Mario!");
}
