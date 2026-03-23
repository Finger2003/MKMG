cbuffer perFrame : register(b0)
{
	matrix viewProj;
    float aspectRatio;
};

cbuffer perObject : register(b1)
{
    float4 color;
};

struct VSIn
{
	float3 pos : POSITION;
};

struct VSOut
{
	float4 pos : SV_POSITION;
	float4 color : COLOR;
    float aspect : ASPECT;
};


VSOut main(VSIn i)
{
    VSOut o;
    o.pos = mul(float4(i.pos, 1.0f), viewProj);
    o.color = color;
    o.aspect = aspectRatio;
    return o;
}