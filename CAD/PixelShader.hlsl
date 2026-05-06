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


struct VSOut
{
    float4 pos : SV_POSITION;
    //float4 color : COLOR;
};

float4 main(VSOut i) : SV_TARGET
{
    return objectColor * stereoTint;
}