cbuffer PerPassBuffer : register(b0)
{
    matrix viewProj;
    float aspectRatio;
    float2 renderSize;
    float padding; // Ensures 16-byte alignment
    float4 stereoTint;
};

cbuffer PerObjectBuffer : register(b1)
{
    matrix model;
    float4 color;
};

struct VertexIn
{
    float3 PosL : POSITION;
};

struct VertexOut
{
    float4 PosW : SV_POSITION;
};

VertexOut main(VertexIn vin)
{
    VertexOut vout;
    vout.PosW = float4(vin.PosL, 1.0f);
    return vout;
}