cbuffer perFrame : register(b0)
{
    matrix viewProj;
    float aspectRatio;
    float2 renderSize;
    float padding;
    float4 stereoTint;
};

cbuffer perObject : register(b1)
{
    matrix model;
    float4 objectColor;
}

struct VSIn
{
    float3 pos : POSITION;
    float2 uv : TEXCOORD;
};

struct VSOut
{
    float4 pos : SV_POSITION;
    float2 uv : TEXCOORD;
};

VSOut main(VSIn i)
{
    VSOut o;
    o.pos = mul(float4(i.pos, 1.0f), model);
    o.pos = mul(o.pos, viewProj);
    o.uv = i.uv;
    return o;
}