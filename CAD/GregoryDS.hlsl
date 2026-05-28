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
    float4 surfaceParams; // x: isoline direction (0: u, 1: v), y: line density, z: line smoothness
}

struct DS_OUTPUT
{
    float4 PosH : SV_POSITION;
};

struct HS_CONTROL_POINT_OUTPUT
{
    float4 PosW : SV_POSITION;
};

struct HS_CONSTANT_DATA_OUTPUT
{
    float EdgeTessFactor[2] : SV_TessFactor;
};

#define NUM_CONTROL_POINTS 20


float3 DeCasteljau(float3 p0, float3 p1, float3 p2, float3 p3, float t)
{
    float3 q0 = lerp(p0, p1, t);
    float3 q1 = lerp(p1, p2, t);
    float3 q2 = lerp(p2, p3, t);

    float3 r0 = lerp(q0, q1, t);
    float3 r1 = lerp(q1, q2, t);

    return lerp(r0, r1, t);
}

[domain("isoline")]
DS_OUTPUT main(
	HS_CONSTANT_DATA_OUTPUT input,
	float2 domain : SV_DomainLocation,
	const OutputPatch<HS_CONTROL_POINT_OUTPUT, NUM_CONTROL_POINTS> patch)
{
    DS_OUTPUT Output;
    
    float u = domain.x;
    float v = domain.y;
    
    float density = input.EdgeTessFactor[0];
    if (density > 1.0f)
        v = v * (density / (density - 1.0f));
    
    if (surfaceParams.x > 0.5f)
    {
        float temp = u;
        u = v;
        v = temp;
    }
    
    float3 P11 = (u * patch[5].PosW.xyz + v * patch[6].PosW.xyz) / (u + v + 1e-6f);
    float3 P21 = ((1.0f - u) * patch[7].PosW.xyz + v * patch[8].PosW.xyz) / ((1.0f - u) + v + 1e-6f);
    float3 P12 = (u * patch[11].PosW.xyz + (1.0f - v) * patch[12].PosW.xyz) / (u + (1.0f - v) + 1e-6f);
    float3 P22 = ((1.0f - u) * patch[13].PosW.xyz + (1.0f - v) * patch[14].PosW.xyz) / ((1.0f - u) + (1.0f - v) + 1e-6f);
	
    float3 B[16];
    B[0] = patch[0].PosW.xyz;
    B[1] = patch[1].PosW.xyz;
    B[2] = patch[2].PosW.xyz;
    B[3] = patch[3].PosW.xyz;
    B[4] = patch[4].PosW.xyz;
    B[5] = P11;
    B[6] = P21;
    B[7] = patch[9].PosW.xyz;
    B[8] = patch[10].PosW.xyz;
    B[9] = P12;
    B[10] = P22;
    B[11] = patch[15].PosW.xyz;
    B[12] = patch[16].PosW.xyz;
    B[13] = patch[17].PosW.xyz;
    B[14] = patch[18].PosW.xyz;
    B[15] = patch[19].PosW.xyz;    
    
    float3 vControl[4];
    vControl[0] = DeCasteljau(B[0], B[1], B[2], B[3], u);
    vControl[1] = DeCasteljau(B[4], B[5], B[6], B[7], u);
    vControl[2] = DeCasteljau(B[8], B[9], B[10], B[11], u);
    vControl[3] = DeCasteljau(B[12], B[13], B[14], B[15], u);
    
    float3 worldPos = DeCasteljau(vControl[0], vControl[1], vControl[2], vControl[3], v);
    
    Output.PosH = mul(float4(worldPos, 1.0f), viewProj);
    return Output;
}