import Scene.Scene;
import Utils.Debug.PixelDebug;
import Scene.Camera.Camera;

cbuffer PerFrameCB : register(b0)
{
	float4x4 gvpTransform;
    Camera gCamera;
    uint2 gSelectedPixel;
};

void vs(float3 pos : POSITION, out float4 posSV : SV_POSITION)
{
    // posSV = transform(pos);
    // posSV = float4(pos, 1);
    print("input pos", pos);
    posSV = mul(float4(pos, 1), gCamera.getViewProj());

    // print("pos", posSV);

}

float4 transform(float3 pos)
{
	return mul(float4(pos, 1), gvpTransform);
}

float4 ps() : SV_TARGET0
{
	return float4(1, 0, 0, 1);
}


