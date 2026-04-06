cbuffer perFrame : register(b0)
{
    matrix viewProj;
    float aspectRatio;
}

cbuffer perObject : register(b1)
{
    matrix model;
    float4 objectColor;
}

struct GSOut
{
    float4 pos : SV_POSITION;
    //float4 color : COLOR;
    float2 uv : TEXCOORD0;
};


float4 main(GSOut input) : SV_TARGET
{
    float dist = dot(input.uv, input.uv);
    
    if (dist > 1.0f)
    {
        discard;
    }
    
    return objectColor;
}