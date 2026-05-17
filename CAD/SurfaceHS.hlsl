struct VS_CONTROL_POINT_OUTPUT
{
    float4 PosW : SV_POSITION;
};

struct HS_CONTROL_POINT_OUTPUT
{
    float4 PosW : SV_POSITION;
};

struct HS_CONSTANT_DATA_OUTPUT
{
	float EdgeTessFactor[2]			: SV_TessFactor;
};

cbuffer perObject : register(b1)
{
    matrix model;
    float4 objectColor;
    float4 surfaceParams;	 // x: isoline direction (0: u, 1: v), y: line density, z: line smoothness
}

#define NUM_CONTROL_POINTS 16

HS_CONSTANT_DATA_OUTPUT CalcHSPatchConstants(
	InputPatch<VS_CONTROL_POINT_OUTPUT, NUM_CONTROL_POINTS> ip,
	uint PatchID : SV_PrimitiveID)
{
	HS_CONSTANT_DATA_OUTPUT Output;

    Output.EdgeTessFactor[0] = surfaceParams.y;
    Output.EdgeTessFactor[1] = surfaceParams.z;

	return Output;
}

[domain("isoline")]
[partitioning("integer")]
[outputtopology("line")]
[outputcontrolpoints(NUM_CONTROL_POINTS)]
[patchconstantfunc("CalcHSPatchConstants")]
HS_CONTROL_POINT_OUTPUT main( 
	InputPatch<VS_CONTROL_POINT_OUTPUT, NUM_CONTROL_POINTS> ip, 
	uint i : SV_OutputControlPointID,
	uint PatchID : SV_PrimitiveID )
{
	HS_CONTROL_POINT_OUTPUT Output;

	Output.PosW = ip[i].PosW;


	return Output;
}
