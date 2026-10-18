cbuffer PerPassBuffer : register(b0)
{
    matrix viewProj;
    float aspectRatio;
    float2 renderSize;
    float padding;
    float4 stereoTint;
};

cbuffer PerObjectBuffer : register(b1)
{
    matrix model;
    float4 objectColor;
};

struct PSInput
{
    float4 pos : SV_POSITION;
    float3 worldPos : WORLDPOS;
};

float4 main(PSInput input) : SV_Target
{
    float3 dpdx = ddx(input.worldPos);
    float3 dpdy = ddy(input.worldPos);
    float3 normal = normalize(cross(dpdy, dpdx));    
    
    float3 lightDir1 = normalize(float3(0.4f, 0.8f, -0.4f));
    float3 lightDir2 = normalize(float3(-0.5f, 0.5f, 0.6f));
    
    float diff = max(dot(normal, lightDir1), 0.0f) * 0.65f
               + max(dot(normal, lightDir2), 0.0f) * 0.25f;
    
    float lighting = saturate(0.20f + diff);
    
    return float4(objectColor.rgb * lighting, objectColor.a) * stereoTint;
}