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

struct VSInput
{
    float3 pos : POSITION;
};

struct VSOutput
{
    float4 pos : SV_POSITION;
    float3 worldPos : WORLDPOS;
};

VSOutput main(VSInput input)
{
    VSOutput output;
    float4 worldPos = mul(float4(input.pos, 1.0f), model);
    output.worldPos = worldPos.xyz;
    output.pos = mul(worldPos, viewProj);
    return output;
}