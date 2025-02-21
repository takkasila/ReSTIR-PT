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
#include <algorithm>
#include "DebugPathDataType.slang";


namespace
{
    const char kDesc[] = "Visualize a path (vertices) on top of an input image with a depth buffer.";
    const char kInputImg[] = "inputImg";
    const char kOutputImg[] = "outputImg";
    const char kDepth[] = "depth";
    const char kInputVBuffer[] = "vbuffer";

    const char kRasterPassShaderFile[] = "RenderPasses/PathVisualizePass/PathVisualizeRasterPassShader.slang";

    struct Vertex
    {
        float3 pos;
        float2 texCoord;
        float4 color;
    };

    const float kPyramidHeight = 1;     // Should always be 1 because it'll be normalized to fit new space.
    const float kPyramidHalfWidth = 0.007;  // In world space. This value will not be normalized to fit new space.

    const uint kPyramidVertCount = 5;

    float3 kPyramidVerts[kPyramidVertCount] = {
        {-kPyramidHalfWidth, 0, kPyramidHalfWidth},
        {kPyramidHalfWidth, 0, kPyramidHalfWidth},
        {kPyramidHalfWidth, 0, -kPyramidHalfWidth},
        {-kPyramidHalfWidth, 0, -kPyramidHalfWidth},
        {0, kPyramidHeight, 0},
    };

    //  Total number of debug paths.
    //      - canonical path
    //      - NEE branch
    //      - temporal central-resevoir retrace path
    //      - temporal temporal-resevoir retrace path
    const uint kTotalDebugPath = 4;

    //  Compute maximum vertex buffer size.
    const size_t kMaxVertexBufferSize = sizeof(Vertex) * kPyramidVertCount * kMaxPathLength * kTotalDebugPath;

    const uint kPyramidIndicesCount = 18;

	uint kPyramidIndices[kPyramidIndicesCount] = {
		0, 2, 1,
        0, 3, 2,
        0, 1, 4,
        1, 2, 4,
        2, 3, 4,
        3, 0, 4,
	};

    const size_t kMaxIndicesBufferSize = sizeof(uint) * kPyramidIndicesCount * kMaxPathLength * kTotalDebugPath;

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

    // Create sampler(s)
    Sampler::Desc samplerDesc;
    samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point);
    mpPointSampler = Sampler::create(samplerDesc);

    // Debug
    mpPixelDebug = PixelDebug::create(1000);
    mpPixelDebug->setEnabled(true);

    return;
}

void PathVisualizePass::createRasterPass()
{
    Program::DefineList defines;
    defines.add(mpScene->getSceneDefines());

    mpRasterPass = RasterPass::create(kRasterPassShaderFile, "vs", "ps", defines);

    // Test
    Vertex verts[1 * kPyramidVertCount];
    uint indices[1 * kPyramidIndicesCount];
    {

        float3 A(0, 0, 0), B(0.1, 0, 0);

        uint vertexOffset = 0;
        uint indexOffset = 0;

        // Construct change of basis mat
        glm::mat4 M = computeTransformMatToLineSegment(A, B);

        for (uint j = 0; j < kPyramidVertCount; j++)
        {
            verts[vertexOffset + j].pos = (M * float4(kPyramidVerts[j], 1)).xyz;
            verts[vertexOffset + j].texCoord = float2(0.5, 0.5);
            verts[vertexOffset + j].color = float4(1, 0, 0.5, 1);
        }

        for (uint j = 0; j < kPyramidIndicesCount; j++)
        {
            indices[indexOffset + j] = kPyramidIndices[j] + vertexOffset;
        }

        vertexOffset += kPyramidVertCount;
        indexOffset += kPyramidIndicesCount;
    }

    mpVertexBuffer = Buffer::create(
        kMaxVertexBufferSize,
        //sizeof(verts),
        ResourceBindFlags::Vertex,
        Buffer::CpuAccess::Write,
        nullptr
        //(void*)verts
    );

    //  Create sample index buffer
    mpIndexBuffer = Buffer::create(
        kMaxIndicesBufferSize,
        //sizeof(indices),
        Resource::BindFlags::Index,
        Buffer::CpuAccess::Write,
        nullptr
        //(void*)indices
    );

    // Create vertex array object
    //      Define buffer layout
    auto pVLayout = VertexLayout::create();
    auto pVBLayout = VertexBufferLayout::create();
    //      Must set arraySize = 1. This is not a mistake and I don't know why it must set to be 1. Perhaps it means per vertex?
    pVBLayout->addElement("POSITION", offsetof(Vertex, Vertex::pos), ResourceFormat::RGB32Float, 1, 0);
    pVBLayout->addElement("TEXCOORD", offsetof(Vertex, Vertex::texCoord), ResourceFormat::RG32Float, 1, 1);
    pVBLayout->addElement("COLOR", offsetof(Vertex, Vertex::color), ResourceFormat::RGBA32Float, 1, 2);
    pVLayout->addBufferLayout(0, pVBLayout);
    Vao::BufferVec vaoBuffers{ mpVertexBuffer };

    //      Create VAO
    auto vao = Vao::create(
        Vao::Topology::TriangleList,
        pVLayout,
        vaoBuffers,
        mpIndexBuffer,
        ResourceFormat::R32Uint
    );


    // Set VAO to raster pass
    auto pRasterState = mpRasterPass->getState();
    pRasterState->setVao(vao);


    // create the depth-state
    DepthStencilState::Desc dsDesc;
    dsDesc.setDepthEnabled(true);
	dsDesc.setDepthFunc(ComparisonFunc::Less);
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

void PathVisualizePass::filterCopyPathData(DebugPathData* incomingDebugPathData)
{

    //  Filter and copy debug path data from ReSTIRPT pass
    bool isCopyNewPathData = true;

    //  Going through the conditions

    //      Has atleast 2 vertices (1 path)
    isCopyNewPathData &= incomingDebugPathData->vertexCount > 1;

    //      Path with RC vertex
    isCopyNewPathData &= !mShowOnlyPathWithRCVertex
        || (mShowOnlyPathWithRCVertex && incomingDebugPathData->hasRCVertex);

    //      Path with NEE
    bool hasAnNEE = false;
    if (mShowOnlyPathWithNEE)
    {
        hasAnNEE = std::any_of(
            std::begin(incomingDebugPathData->isSampledLight),
            std::begin(incomingDebugPathData->isSampledLight) + incomingDebugPathData->vertexCount,
            [](bool b) {return b; }
        );
    }

    isCopyNewPathData &= !mShowOnlyPathWithNEE
        || (mShowOnlyPathWithNEE && hasAnNEE);

    //      Finally copy if all conditions are met
    if (isCopyNewPathData)
        mRunningPathData = *incomingDebugPathData;
}

void PathVisualizePass::updatePathData()
{
    if (mRunningPathData.vertexCount < 1)
        return;

    // Copy from temporary to current
    mDebugPathData = mRunningPathData;

    mTemporalCentralPathData = mRunningTemporalCentralPathData;
    mTemporalTemporalPathData = mRunningTemporalTemporalPathData;

    //
    // Construct path geometry
    //
    //      Get pointer to current vertex buffer in device
    Vertex* verts = static_cast<Vertex*>(mpVertexBuffer->map(Buffer::MapType::WriteDiscard));
    uint* indices = static_cast<uint*>(mpIndexBuffer->map(Buffer::MapType::WriteDiscard));

    float3 A, B;
    uint vertexOffset = 0;
    uint indexOffset = 0;
	float4 colorBegin(0, 1, 0, 1), colorEnd(0, 0, 0, 1);
    float4 color;
    glm::mat4 M;

    for (uint i = 0; i < mDebugPathData.vertexCount - 1; i++)
    {
        A = mDebugPathData.vertices[i].xyz;
        B = mDebugPathData.vertices[i + 1].xyz;

        // Construct change of basis mat
        M = computeTransformMatToLineSegment(A, B);

		float t = i / float(mDebugPathData.vertexCount - 1);
		color = colorBegin + t * (colorEnd - colorBegin);

        // Vertex
        for (uint j = 0; j < kPyramidVertCount; j++)
        {
            verts[vertexOffset + j].pos = (M * float4(kPyramidVerts[j], 1)).xyz;

			// TODO: use proper tex coord
            verts[vertexOffset + j].texCoord = float2(0.5, 0.5);

            // If target vertex is an RC vertex, color code as red
            if (i + 1 == mDebugPathData.rcVertexIndex)
            {
				color = float4(1, 0, 0, 1);
            }
			verts[vertexOffset + j].color = color;
        }

        // Index
        for (uint j = 0; j < kPyramidIndicesCount; j++)
        {
            indices[indexOffset + j] = kPyramidIndices[j] + vertexOffset;
        }

        vertexOffset += kPyramidVertCount;
        indexOffset += kPyramidIndicesCount;
    }


    //
    // Construct NEE segments geometry
    //
    bool hasAnNEE = std::any_of(
        std::begin(mDebugPathData.isSampledLight),
        std::begin(mDebugPathData.isSampledLight) + mDebugPathData.vertexCount,
        [](bool b) {return b; }
    );
    if (hasAnNEE)
    {
        // Gather indices of vertex that has NEE
        std::vector<uint> neeVertexIndex;
        for (uint i = 0; i < mDebugPathData.vertexCount; i++)
        {
            if (mDebugPathData.isSampledLight[i])
                neeVertexIndex.emplace_back(i);
        }

        for (uint i : neeVertexIndex)
        {
            A = mDebugPathData.vertices[i].xyz;
            B = mDebugPathData.sampledLightPosition[i].xyz;

            // Construct change of basis mat
            M = computeTransformMatToLineSegment(A, B);

            // Vertex
            for (uint j = 0; j < kPyramidVertCount; j++)
            {
                verts[vertexOffset + j].pos = (M * float4(kPyramidVerts[j], 1)).xyz;

                // TODO: use proper tex coord
                verts[vertexOffset + j].texCoord = float2(0.5, 0.5);

                verts[vertexOffset + j].color = float4(1, 1, 0, 1);
            }

            // Index
            for (uint j = 0; j < kPyramidIndicesCount; j++)
            {
                indices[indexOffset + j] = kPyramidIndices[j] + vertexOffset;
            }

            vertexOffset += kPyramidVertCount;
            indexOffset += kPyramidIndicesCount;
        }
    }

    //
    //  Construct temporal central-resevoir retrace path geometry
    //
    colorBegin = float4(0, 0, 1, 1);
    for (int i = 0; i < int(mTemporalCentralPathData.vertexCount) - 1; i++)
    {
        A = mTemporalCentralPathData.vertices[i].xyz;
        B = mTemporalCentralPathData.vertices[i + 1].xyz;

        // Construct change of basis mat
        M = computeTransformMatToLineSegment(A, B);

        float t = i / float(mTemporalCentralPathData.vertexCount - 1);
        color = colorBegin + t * (colorEnd - colorBegin);

        // Vertex
        for (uint j = 0; j < kPyramidVertCount; j++)
        {
            verts[vertexOffset + j].pos = (M * float4(kPyramidVerts[j], 1)).xyz;

            // TODO: use proper tex coord
            verts[vertexOffset + j].texCoord = float2(0.5, 0.5);
            verts[vertexOffset + j].color = color;
        }

        // Index
        for (uint j = 0; j < kPyramidIndicesCount; j++)
        {
            indices[indexOffset + j] = kPyramidIndices[j] + vertexOffset;
        }

        vertexOffset += kPyramidVertCount;
        indexOffset += kPyramidIndicesCount;
    }

    //
    //  Construct temporal temporal-resevoir retrace path geometry
    //
    colorBegin = float4(0, 1, 1, 1);
    for (int i = 0; i < int(mTemporalTemporalPathData.vertexCount) - 1; i++)
    {
        A = mTemporalTemporalPathData.vertices[i].xyz;
        B = mTemporalTemporalPathData.vertices[i + 1].xyz;

        // Construct change of basis mat
        M = computeTransformMatToLineSegment(A, B);

        float t = i / float(mTemporalTemporalPathData.vertexCount - 1);
        color = colorBegin + t * (colorEnd - colorBegin);

        // Vertex
        for (uint j = 0; j < kPyramidVertCount; j++)
        {
            verts[vertexOffset + j].pos = (M * float4(kPyramidVerts[j], 1)).xyz;

            // TODO: use proper tex coord
            verts[vertexOffset + j].texCoord = float2(0.5, 0.5);
            verts[vertexOffset + j].color = color;
        }

        // Index
        for (uint j = 0; j < kPyramidIndicesCount; j++)
        {
            indices[indexOffset + j] = kPyramidIndices[j] + vertexOffset;
        }

        vertexOffset += kPyramidVertCount;
        indexOffset += kPyramidIndicesCount;
    }

    mTotalIndices = indexOffset;

    mpVertexBuffer->unmap();
    mpIndexBuffer->unmap();
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

    //  Store running path data
    {
        InternalDictionary& renderDataDict = renderData.getDictionary();

        //      Get debug path data from ReSTIRPTPass
        DebugPathData* incomingDebugPathData = renderDataDict.getValue<DebugPathData*>("debugPathData");
        DebugPathData* incomingTemporalCentralPathData = renderDataDict.getValue<DebugPathData*>("temporalCentralPathData");
        DebugPathData* incomingTemporalTemporalPathData = renderDataDict.getValue<DebugPathData*>("temporalTemporalPathData");

        filterCopyPathData(incomingDebugPathData);

        // TODO: filter this
        mRunningTemporalCentralPathData = *incomingTemporalCentralPathData;
        mRunningTemporalTemporalPathData = *incomingTemporalTemporalPathData;
    }

    // Create FBO
    Fbo::SharedPtr pFbo = Fbo::create();

    //  Render on top of input image by copying from input to the render target
    pRenderContext->blit(pInputImg->getSRV(), pOutputImg->getRTV());

    //  Bind render target
    pFbo->attachColorTarget(pOutputImg, 0);

	if (mRecreateRasterPass)
	{
		createRasterPass();
		mRecreateRasterPass = false;
	}

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

	// Run the raster pass
    mpRasterPass->drawIndexed(pRenderContext, mTotalIndices, 0, 0);

	mpPixelDebug->endFrame(pRenderContext);

}

void PathVisualizePass::renderUI(Gui::Widgets& widget)
{
    widget.checkbox("Show only path with Reconnection Vertex", mShowOnlyPathWithRCVertex);
    widget.checkbox("Show only path with NEE", mShowOnlyPathWithNEE);

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

bool PathVisualizePass::onKeyEvent(const KeyboardEvent& keyEvent)
{
    // shortcut key "u" to call updatePath,
    if (keyEvent.key == KeyboardEvent::Key::U
        && keyEvent.type == KeyboardEvent::Type::KeyReleased
        && keyEvent.mods.isAltDown == false
        && keyEvent.mods.isCtrlDown == false
        && keyEvent.mods.isShiftDown == false)
    {
        updatePathData();
    }

    return false;
}
