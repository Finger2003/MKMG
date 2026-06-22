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


Texture2D trimTexture : register(t0);
SamplerState trimSampler : register(s0);

struct VSOut
{
    float4 pos : SV_POSITION;
    float2 uv : TEXCOORD;
    //float4 color : COLOR;
};

float4 main(VSOut i) : SV_TARGET
{
    float mask = trimTexture.Sample(trimSampler, i.uv).r;
    
    if (mask < 0.5f)
        discard;
    
    return objectColor * stereoTint;
}