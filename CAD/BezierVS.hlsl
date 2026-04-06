cbuffer PerPassBuffer : register(b0)
{
    matrix viewProj;
    float aspectRatio;
    float2 renderSize;
    float padding; // Ensures 16-byte alignment
};

cbuffer PerObjectBuffer : register(b1)
{
    matrix model;
    float4 color;
};

struct VertexIn
{
    float3 PosL : POSITION; // Local space position
};

struct VertexOut
{
    float4 PosH : SV_POSITION; // Clip Space
    float4 PosW : POSITION; // World Space
};

VertexOut main(VertexIn vin)
{
    VertexOut vout;

    float4 posW = mul(float4(vin.PosL, 1.0f), model);
    vout.PosW = posW;
    vout.PosH = mul(posW, viewProj);

    return vout;
}