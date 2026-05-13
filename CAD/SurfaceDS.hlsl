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
    float4 surfaceParams;
}

struct DS_OUTPUT
{
	float4 PosH  : SV_POSITION;
};

struct HS_CONTROL_POINT_OUTPUT
{
    float4 PosW : SV_POSITION;
};

struct HS_CONSTANT_DATA_OUTPUT
{
	float EdgeTessFactor[2]			: SV_TessFactor;
	//float InsideTessFactor			: SV_InsideTessFactor;
};

#define NUM_CONTROL_POINTS 16


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
    //if (surfaceParams.x > 0.5f)
    //{
    //    u = domain.y;
    //    v = domain.x;
    //}
    
    float density = input.EdgeTessFactor[0];
    if (density > 1.0f)
    {
        v = v * (density / (density - 1.0f));
    }
    
    if (surfaceParams.x > 0.5f)
    {
        float temp = u;
        u = v;
        v = temp;
    }

    float3 vControl[4];
    vControl[0] = DeCasteljau(patch[0].PosW.xyz, patch[1].PosW.xyz, patch[2].PosW.xyz, patch[3].PosW.xyz, u);
    vControl[1] = DeCasteljau(patch[4].PosW.xyz, patch[5].PosW.xyz, patch[6].PosW.xyz, patch[7].PosW.xyz, u);
    vControl[2] = DeCasteljau(patch[8].PosW.xyz, patch[9].PosW.xyz, patch[10].PosW.xyz, patch[11].PosW.xyz, u);
    vControl[3] = DeCasteljau(patch[12].PosW.xyz, patch[13].PosW.xyz, patch[14].PosW.xyz, patch[15].PosW.xyz, u);
    
    float3 worldPos = DeCasteljau(vControl[0], vControl[1], vControl[2], vControl[3], v);
    
    Output.PosH = mul(float4(worldPos, 1.0f), viewProj);
	//Output.PosH = float4(
	//	patch[0].PosH*domain.x+patch[1].PosH*domain.y+patch[2].PosH*domain.z,1);

	return Output;
}
