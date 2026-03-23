struct VSOut
{
    float4 pos : SV_POSITION;
    float4 color : COLOR;
    float aspect : ASPECT;
};


struct GSOut
{
	float4 pos : SV_POSITION;
    float4 color : COLOR;
    float2 uv : TEXCOORD0;
};

[maxvertexcount(4)]
void main(
	point VSOut input[1] : SV_POSITION,
	inout TriangleStream< GSOut > outputStream
)
{
    GSOut output;
    output.color = input[0].color;
    float radiusY = 0.005f;
    float radiusX = radiusY / input[0].aspect;
	
    float4 center = input[0].pos;
	
	// Top left
    output.pos = center + float4(-radiusX, radiusY, 0.0f, 0.0f) * center.w;
    output.uv = float2(-1.0f, 1.0f);
	outputStream.Append(output);
	
	
	// Top right
	output.pos = center + float4(radiusX, radiusY, 0.0f, 0.0f) * center.w;
    output.uv = float2(1.0f, 1.0f);
    outputStream.Append(output);
		
	// Bottom left
    output.pos = center + float4(-radiusX, -radiusY, 0.0f, 0.0f) * center.w;
	output.uv = float2(-1.0f, -1.0f);
    outputStream.Append(output);
	
	// Bottom right
	output.pos = center + float4(radiusX, -radiusY, 0.0f, 0.0f) * center.w;
	output.uv = float2(1.0f, -1.0f);
    outputStream.Append(output);
}