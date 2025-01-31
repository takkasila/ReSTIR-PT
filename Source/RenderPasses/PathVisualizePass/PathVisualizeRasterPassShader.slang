import Scene.Scene;
import Utils.Debug.PixelDebug;
import Scene.Camera.Camera;

cbuffer PerFrameCB : register(b0)
{
    Camera gCamera;
    uint2 gSelectedPixel;
};

struct VSIn
{
    float3 pos : POSITION;
    float2 texCoord : TEXCOORD;
    float4 color : COLOR;
};

struct VSOut
{
    float4 pos : SV_POSITION;
    float2 texCoord : TEXCOORD;
    float4 color : COLOR;
};

VSOut vs(VSIn vsIn)
{
    //print("input pos", vsIn.pos);
    //print("input texCoord", vsIn.texCoord);
    //print("input color", vsIn.color);

    VSOut vsOut;

    vsOut.pos = mul(float4(vsIn.pos, 1), gCamera.getViewProj());

    vsOut.texCoord = vsIn.texCoord;
    vsOut.color = vsIn.color;

    return vsOut;
}

float4 ps(VSOut vsOut) : SV_TARGET
{
    float4 color = vsOut.color;
	return color;
}


