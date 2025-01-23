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
#include <iostream>


namespace
{
    const char kDesc[] = "Visualize a path (vertices) on top of an input image with a depth buffer.";
    const char kInputImg[] = "inputImg";
    const char kOutputImg[] = "outputImg";
    const char kDepth[] = "depth";
    const char kInputVBuffer[] = "vbuffer";

    const char kPathVisualizeShaderPassFile[] = "RenderPasses/PathVisualizePass/PathVisualizePass.ps.slang";
    const char kRasterPassShaderFile[] = "RenderPasses/PathVisualizePass/PathVisualizeRasterPassShader.slang";

    const float kPyramidHeight = 1;
    const float kPyramidHalfWidth = 0.01;

	float3 kVertices[5] = {
        {-kPyramidHalfWidth, 0, kPyramidHalfWidth},
        {kPyramidHalfWidth, 0, kPyramidHalfWidth},
        {kPyramidHalfWidth, 0, -kPyramidHalfWidth},
        {-kPyramidHalfWidth, 0, -kPyramidHalfWidth},
        {0, kPyramidHeight, 0},
	};

    const uint kIndicesCount = 18;
	uint kIndices[kIndicesCount] = {
		0, 1, 2,
        0, 2, 3,
        0, 1, 4,
        1, 2, 4,
        2, 3, 4,
        3, 0, 4,
	};
    
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

    // Create shader passes
    //createPathVisualizeShaderPass();

    // Create sampler(s)
    Sampler::Desc samplerDesc;
    samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point);
    mpPointSampler = Sampler::create(samplerDesc);

    // Debug
    mpPixelDebug = PixelDebug::create(1000);
    mpPixelDebug->setEnabled(true);

    return;
}

void PathVisualizePass::createPathVisualizeShaderPass()
{
    Program::DefineList defines;
    defines.add(mpScene->getSceneDefines());

    mpPathVisualizeShaderPass = FullScreenPass::create(kPathVisualizeShaderPassFile, defines);
}

void PathVisualizePass::createRasterPass()
{
    Program::DefineList defines;
    defines.add(mpScene->getSceneDefines());

    mpRasterPass = RasterPass::create(kRasterPassShaderFile, "vs", "ps", defines);

    mpVertexBuffer = Buffer::create(
        sizeof(kVertices),
        ResourceBindFlags::Vertex,
        Buffer::CpuAccess::Write,
        (void*)kVertices
    );

    //  Create sample index buffer
    Buffer::SharedPtr pIndexBuffer = Buffer::create(
        sizeof(kIndices),
        Resource::BindFlags::Index,
        Buffer::CpuAccess::Write,
        (void*)kIndices
    );

    // Create vertex array object
    //      Define buffer layout
    auto pVLayout = VertexLayout::create();
    auto pVBLayout = VertexBufferLayout::create();
    //      Must set arraySize = 1. This is not a mistake and I don't know why it must set to be 1.
    pVBLayout->addElement("POSITION", 0, ResourceFormat::RGB32Float, 1, 0);

    pVLayout->addBufferLayout(0, pVBLayout);
    Vao::BufferVec vaoBuffers{ mpVertexBuffer };

    //      Create VAO
    auto vao = Vao::create(
        Vao::Topology::TriangleList,
        pVLayout,
        vaoBuffers,
        pIndexBuffer,
        ResourceFormat::R32Uint
    );


    // Set VAO to raster pass
    auto pRasterState = mpRasterPass->getState();
    pRasterState->setVao(vao);


    // create the depth-state
    DepthStencilState::Desc dsDesc;
    dsDesc.setDepthEnabled(false);
    pRasterState->setDepthStencilState(DepthStencilState::create(dsDesc));


    // Rasterizer state
    RasterizerState::Desc rsState;
    rsState.setCullMode(RasterizerState::CullMode::Back);
    rsState.setFillMode(RasterizerState::FillMode::Solid);
    pRasterState->setRasterizerState(RasterizerState::create(rsState));


    // Blend state
    BlendState::Desc blendDesc;
    blendDesc.setRtBlend(0, true).setRtParams(0, BlendState::BlendOp::Add,
        BlendState::BlendOp::Add,
        BlendState::BlendFunc::SrcAlpha,
        BlendState::BlendFunc::OneMinusSrcAlpha,
        BlendState::BlendFunc::One,
        BlendState::BlendFunc::One);
    pRasterState->setBlendState(BlendState::create(blendDesc));
}

void PathVisualizePass::updatePathData()
{

    // Construct change of basis mat
    float3 A(0, 0.3, 0), B(0.1, 0.1, 0.1);


    // Apply over vertices
    //      Get pointer to current vertex buffer in device
    float3* verts = (float3*)mpVertexBuffer->map(Buffer::MapType::Write);

    glm::mat4 M = computeTransformMatToLineSegment(A, B);

    for (uint i = 0; i < 5; i++)
    {
        verts[i] = (M * float4(kVertices[i], 1)).xyz;
    }

    mpVertexBuffer->unmap();

    return;

    if (mDebugPathData.length == 0)
        return;

    if (!mDebugPathData.hasRCVertex)
        return;

    mPathVertices.clear();

    for (int i = 0; i < mDebugPathData.length; i++)
    {
        mPathVertices.emplace_back(mDebugPathData.vertices[i]);
    }

    // Convert path vertices into a 1D-rgb texture.
    Texture::SharedPtr pathVerticesTex = Texture::create1D((uint32_t)mPathVertices.size(), ResourceFormat::RGB32Float, 1, 1, mPathVertices.data());

    // Bind pointer to texture
    mpPathVisualizeShaderPass["gPathVerticesTex"] = pathVerticesTex;

    mpPathVisualizeShaderPass["PerFrameCB"]["gPathLenght"] = mPathVertices.size();
}

glm::mat4 PathVisualizePass::computeTransformMatToLineSegment(float3 lineBegin, float3 lineEnd)
{
    float3 v_prime = lineEnd - lineBegin;
    float3 up(0, 1, 0);

    float3 crossProd = glm::cross(v_prime, up);
    float area = glm::length(crossProd);
    if (glm::epsilonEqual(area, 0.f, glm::epsilon<float>()))
    {
        // Change from up vec to down vec;
        crossProd = glm::cross(v_prime, -up);
    }

    float3 u_prime = glm::normalize(crossProd);
    float3 w_prime = glm::normalize(glm::cross(u_prime, v_prime));

    float3 t = lineBegin;

    glm::mat4 M(
        float4(u_prime, 0), float4(v_prime, 0), float4(w_prime, 0), float4(t, 1)
    );

    return M;
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

    reflector.addInput(kDepth, "Depth buffer");

    reflector.addInput(kInputVBuffer, "Vertex buffer");

    reflector.addOutput(kOutputImg, "Output image");

    return reflector;
}

void PathVisualizePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
	if (!mpScene)
	{
		return;
	}

    Texture::SharedPtr pInputImg = renderData[kInputImg]->asTexture();
    Texture::SharedPtr pDepth = renderData[kDepth]->asTexture();
    Texture::SharedPtr pOutputImg = renderData[kOutputImg]->asTexture();

    assert(pInputImg && pDepth && pOutputImg);

    // Create FBO
    Fbo::SharedPtr pFbo = Fbo::create();

    //  Render on top of input image by copying from input to the render target
    pRenderContext->blit(pInputImg->getSRV(), pOutputImg->getRTV());

    //  Bind render target
    pFbo->attachColorTarget(pOutputImg, 0);

    if (mUseRasterPass)
    {
		if (mRecreateRasterPass)
		{
			createRasterPass();
			mRecreateRasterPass = false;
		}

		//// Vertex buffer
		//float3* verts = (float3*)mpVertexBuffer->map(Buffer::MapType::WriteDiscard);
		//verts[0] = float3(3, 4, 5);
		//verts[1] = float3(2, 7, 9);
		//mpVertexBuffer->unmap();

		//	Bind data
		GraphicsState::SharedPtr pRasterState = mpRasterPass->getState();

		//		FBO
		pRasterState->setFbo(pFbo);

		//		Scene
		mpRasterPass["gScene"] = mpScene->getParameterBlock();

		//		Camera
		const Camera::SharedPtr& pCamera = mpScene->getCamera();
		pCamera->setShaderData(mpRasterPass["PerFrameCB"]["gCamera"]);

		//		PixelDebug
		mpRasterPass["PerFrameCB"]["gSelectedPixel"] = mSelectedCursorPosition;


		// Enable pixel debug
		const uint2 resolution = renderData.getDefaultTextureDims();
		mpPixelDebug->beginFrame(pRenderContext, resolution);
		mpPixelDebug->prepareProgram(mpRasterPass->getProgram(), mpRasterPass->getRootVar());

		// Run the shader pass
		mpRasterPass->drawIndexed(pRenderContext, kIndicesCount, 0, 0);

		mpPixelDebug->endFrame(pRenderContext);

    }
    else
    {
        // Fallback to ray-marching pass

        if (mRecreatePathVisualizeShaderPass)
        {
            createPathVisualizeShaderPass();
		    mRecreatePathVisualizeShaderPass = false;
        }

        //  Scene
        mpPathVisualizeShaderPass["gScene"] = mpScene->getParameterBlock();

        // Camera param
	    const Camera::SharedPtr& pCamera = mpScene->getCamera();
	    pCamera->setShaderData(mpPathVisualizeShaderPass["PerFrameCB"]["gCamera"]);

	    // Image size
	    const uint2 resolution = renderData.getDefaultTextureDims();
	    mpPathVisualizeShaderPass["PerFrameCB"]["gResolution"] = resolution;

	    // Selected pixel
	    mpPathVisualizeShaderPass["PerFrameCB"]["gSelectedPixel"] = mSelectedCursorPosition;

	    //	Global params
        mpPathVisualizeShaderPass["gInputImgTex"] = pInputImg;
        mpPathVisualizeShaderPass["gDepthTex"] = pDepth;
        mpPathVisualizeShaderPass["gPointSampler"] = mpPointSampler;

        //  Store path data log from ReSTIRPTPass
        auto& renderDataDict = renderData.getDictionary();
        auto incomingDebugPathData = renderDataDict.getValue<DebugPathData*>("debugPathData");
        // Filter to only path that has an RC vertex.
        if (incomingDebugPathData->hasRCVertex)
            mDebugPathData = *incomingDebugPathData;

        // Enable pixel debug
        mpPixelDebug->beginFrame(pRenderContext, resolution);

        mpPixelDebug->prepareProgram(mpPathVisualizeShaderPass->getProgram(), mpPathVisualizeShaderPass->getRootVar());

        // Run the shader pass
        mpPathVisualizeShaderPass->execute(pRenderContext, pFbo);


        mpPixelDebug->endFrame(pRenderContext);

    }
}

void PathVisualizePass::renderUI(Gui::Widgets& widget)
{
    if (auto group = widget.group("Debugging", true))
    {
        if (group.button("Update Path value", false))
        {
            updatePathData();
        }

        mpPixelDebug->renderUI(group);
    }
}

void PathVisualizePass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
	mpScene = pScene;
}

bool PathVisualizePass::onMouseEvent(const MouseEvent& mouseEvent)
{
    mSelectedCursorPosition = uint2(mouseEvent.screenPos);

	return mpPixelDebug->onMouseEvent(mouseEvent);
}
