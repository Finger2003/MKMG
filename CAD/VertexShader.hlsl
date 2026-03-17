cbuffer perFrame : register(b0)
{
    matrix viewProj;
}

cbuffer perObject : register(b1)
{
    matrix model;
}

struct VSIn
{
    float3 pos : POSITION;
};

struct VSOut
{
    float4 pos : SV_POSITION;
};

VSOut main(VSIn i)
{
    VSOut o;
    o.pos = mul(float4(i.pos, 1.0f), model);
    o.pos = mul(o.pos, viewProj);
    return o;
}