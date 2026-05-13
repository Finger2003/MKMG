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
	//float InsideTessFactor			: SV_InsideTessFactor;
};

#define NUM_CONTROL_POINTS 16

HS_CONSTANT_DATA_OUTPUT CalcHSPatchConstants(
	InputPatch<VS_CONTROL_POINT_OUTPUT, NUM_CONTROL_POINTS> ip,
	uint PatchID : SV_PrimitiveID)
{
	HS_CONSTANT_DATA_OUTPUT Output;

    Output.EdgeTessFactor[0] = 4.0f;
    Output.EdgeTessFactor[1] = 25.0f;

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
