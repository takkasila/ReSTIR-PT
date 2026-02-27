from falcor import *
import os

EnablePathVisualizePass = False
EnableErrorMeasure = False

def render_graph_ReSTIRPT():
    g = RenderGraph("ReSTIRPTPass")
    loadRenderPassLibrary("AccumulatePass.dll")
    loadRenderPassLibrary("GBuffer.dll")
    loadRenderPassLibrary("ReSTIRPTPass.dll")
    loadRenderPassLibrary("ToneMapper.dll")
    loadRenderPassLibrary("ScreenSpaceReSTIRPass.dll")
    loadRenderPassLibrary("ImageLoader.dll")

    if EnableErrorMeasure:
        loadRenderPassLibrary("ErrorMeasurePass.dll")

    if EnablePathVisualizePass:
        loadRenderPassLibrary('PathVisualizePass.dll')

    ScreenSpaceReSTIRPass = createPass("ScreenSpaceReSTIRPass")
    g.addPass(ScreenSpaceReSTIRPass, "ScreenSpaceReSTIRPass")

    ReSTIRGIPlusPass = createPass("ReSTIRPTPass", {'samplesPerPixel': 1})
    g.addPass(ReSTIRGIPlusPass, "ReSTIRPTPass")
    VBufferRT = createPass("VBufferRT", {'samplePattern': SamplePattern.Center, 'sampleCount': 1, 'texLOD': TexLODMode.Mip0, 'useAlphaTest': True})
    g.addPass(VBufferRT, "VBufferRT")

    AccumulatePass = createPass("AccumulatePass", {'enableAccumulation': False, 'precisionMode': AccumulatePrecision.Double})
    g.addPass(AccumulatePass, "AccumulatePass")

    if EnableErrorMeasure:
        ErrorMeasurePass = createPass("ErrorMeasurePass")
        g.addPass(ErrorMeasurePass, "ErrorMeasurePass")

    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0, 'operator': ToneMapOp.Linear})
    g.addPass(ToneMapper, "ToneMapper")

    if EnablePathVisualizePass:
        PathVisualizePass = createPass("PathVisualizePass")
        g.addPass(PathVisualizePass, "PathVisualizePass")

    g.addEdge("VBufferRT.vbuffer", "ReSTIRPTPass.vbuffer")
    g.addEdge("VBufferRT.mvec", "ReSTIRPTPass.motionVectors")

    g.addEdge("VBufferRT.vbuffer", "ScreenSpaceReSTIRPass.vbuffer")
    g.addEdge("VBufferRT.mvec", "ScreenSpaceReSTIRPass.motionVectors")
    g.addEdge("ScreenSpaceReSTIRPass.color", "ReSTIRPTPass.directLighting")

    g.addEdge("ReSTIRPTPass.color", "AccumulatePass.input")

    if EnableErrorMeasure:
        g.addEdge("AccumulatePass.output", "ErrorMeasurePass.Source")
        g.addEdge("ErrorMeasurePass.Output", "ToneMapper.src")

    else:
        g.addEdge("AccumulatePass.output", "ToneMapper.src")

    if EnablePathVisualizePass:
        g.addEdge("ToneMapper.dst", "PathVisualizePass.inputImg")
        g.addEdge("VBufferRT.depth", "PathVisualizePass.depth")
        g.addEdge("VBufferRT.vbuffer", "PathVisualizePass.vbuffer")
        g.markOutput("PathVisualizePass.outputImg")

    g.markOutput("ReSTIRPTPass.color")
    g.markOutput("ReSTIRPTPass.albedo")
    g.markOutput("ReSTIRPTPass.pathLength")
    g.markOutput("ToneMapper.dst")
    g.markOutput("AccumulatePass.output")

    return g

graph_ReSTIRPT = render_graph_ReSTIRPT()

m.addGraph(graph_ReSTIRPT)
