cbuffer PerPassBuffer : register(b0)
{
    matrix viewProj;
    float aspectRatio;
    float2 renderSize;
    float padding;
};

struct VSOut
{
    float4 PosH : SV_POSITION; // Clip Space
    float4 PosW : POSITION; // World Space
};

struct GSOut
{
    float4 PosH : SV_POSITION; // Clip Space
};

[maxvertexcount(128)]
void main(lineadj VSOut input[4], inout LineStream<GSOut> outputStream)
{    
    float2 p0 = (input[0].PosH.xy / input[0].PosH.w) * renderSize * 0.5f;
    float2 p1 = (input[1].PosH.xy / input[1].PosH.w) * renderSize * 0.5f;
    float2 p2 = (input[2].PosH.xy / input[2].PosH.w) * renderSize * 0.5f;
    float2 p3 = (input[3].PosH.xy / input[3].PosH.w) * renderSize * 0.5f;
    
    float polyLength = length(p1 - p0) + length(p2 - p1) + length(p3 - p2);
       
    int segments = clamp(ceil(polyLength / 3.0f), 1, 127);

    GSOut v;
    for (int i = 0; i <= segments; ++i)
    {
        float t = (float) i / (float) segments;
        
        float4 q0 = lerp(input[0].PosW, input[1].PosW, t);
        float4 q1 = lerp(input[1].PosW, input[2].PosW, t);
        float4 q2 = lerp(input[2].PosW, input[3].PosW, t);
        
        float4 r0 = lerp(q0, q1, t);
        float4 r1 = lerp(q1, q2, t);
        
        float4 worldPos = lerp(r0, r1, t);
       
        v.PosH = mul(worldPos, viewProj);
        
        outputStream.Append(v);
    }
    outputStream.RestartStrip();
}