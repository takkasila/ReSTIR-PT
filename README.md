# Manifold Hybrid Shift for ReSTIR Path Tracing
![](manifold_hybrid_shift_banner.png)

## Introduction
We proposed a new shift mapping method: Manifold Hybrid Shift, to allow vertex-reconnection through a specular vertex, along with its Jacobian determinant, and evaluted them in the ReSTIR PT framework (Lin, Kettunen, et al., 2022).


**Full report**: [[.pdf]](https://drive.google.com/file/d/1btgREZXPf4LCprVblFp_MOsg5i3Cevqb/view?usp=drive_link)

**Presentation slide**: [[.pdf]](https://drive.google.com/file/d/1mnXu6C-tJ4n0HjIyhIdYHa1qIAxq-FXl/view?usp=drive_link)


- The method is implemented as a render pass called "ReSTIRPTPass" (Source\RenderPasses\ReSTIRPass) in Falcor 4.4.

- A script `RunReSTIRPTDemo.bat` is provided to show how the method works in an animated version of the VeachAjar scene (from [Benedikt Bitterli's rendering resources](https://benedikt-bitterli.me/resources/)) which is contained in the repo.
- Before running the scripts, you need to compile the program and download the scene files following the instruction below.

## Prerequisites
- Windows 10 version 20H2 or newer
- Visual Studio 2022
- [Windows 10 SDK version 10.0.19041.1 Or Newer] (https://developer.microsoft.com/en-us/windows/downloads/sdk-archive)
- NVIDIA driver 466.11 or newer
- RTX 2060 or Higher (NVIDIA graphics card with raytracing support)
- Get NVAPI, head over to https://developer.nvidia.com/nvapi and download the latest version of NVAPI. Create a folder called `.packman` under `Source/Externals`, Extract the content of the zip file into `Source/Externals/.packman/` and rename `Rxxx-developer` to `nvapi`.

## How to compile
- Make sure you have NVAPI in `Source/Externals/.packman/`
- Open Falcor.sln and Build Solution in configuration ReleaseD3D12

## Run the demo
- execute `RunReSTIRPTDemo.bat`
- The GUI contains self-explanatory settings to turn on/off different components of ReSTIR PT or change its quality.

## Test with more scenes
- You can test your custom scene by running Bin\x64\Release\Mogwai.exe first, then load a scene file.
- A Falcor pyscene is recommended. For how to create a pyscene, please check out the `Source/RenderPasses/ReSTIRPTPass/Data/VeachAjar/VeachAjar.pyscene` as a template.
Details can be found in Falcor's [documentation](Docs/Usage/Scene-Formats.md)
- Alternatively, if you have a scene file with well defined lighting, material, and camera information that is supported by Falocr (like FBX), you can also create a one-line
pyscene file, e.g. `sceneBuilder.importScene(YOUR_SCENE_FILE)`.

## Offline rendering
- ReSTIR PT is an unbiased algorithm and can be used for offline rendering. The recommended setting is to disable temporal reuse, use 32 candidate samples per pixel, and set the number of spatial reuse rounds, spatial neighbors, and spatial reuse radius to 3, 6, 10, respectively (more details in the paper).

## An example screenshot (running on an RTX 3090)
![](Screenshot.png)


