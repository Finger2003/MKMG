cbuffer PerPassBuffer : register(b0)
{
    matrix viewProj;
    float aspectRatio;
    float2 renderSize;
    float padding; // Ensures 16-byte alignment
};

cbuffer PerObjectBuffer : register(b1)
{
    float4 controlPoints[4];
    float4 color;
};

//struct VertexIn
//{
//    float3 PosL : POSITION; // Local space position
//};

struct VertexOut
{
    float4 PosH : SV_POSITION; // Clip Space
    float4 PosW : POSITION; // World Space
    uint InstanceID : SV_InstanceID; // Instance ID for instancing
};

VertexOut main(uint vid : SV_VertexID, uint instanceID : SV_InstanceID)
{
    VertexOut vout;

    float4 posW = controlPoints[vid];
    vout.PosW = posW;
    vout.PosH = mul(posW, viewProj);
    vout.InstanceID = instanceID;

    return vout;
}