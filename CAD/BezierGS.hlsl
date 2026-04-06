cbuffer PerPassBuffer : register(b0)
{
    matrix viewProj;
    float aspectRatio;
    float2 renderSize;
    float padding;
};

struct VertexOut
{
    float4 PosH : SV_POSITION; // Clip Space
    float4 PosW : POSITION; // World Space
    //float4 Color : COLOR;
};

[maxvertexcount(128)]
void main(lineadj VertexOut input[4], inout LineStream<VertexOut> outputStream)
{
    // 1. Project control points to screen space to calculate presence
    float2 p0 = (input[0].PosH.xy / input[0].PosH.w) * renderSize * 0.5f;
    float2 p1 = (input[1].PosH.xy / input[1].PosH.w) * renderSize * 0.5f;
    float2 p2 = (input[2].PosH.xy / input[2].PosH.w) * renderSize * 0.5f;
    float2 p3 = (input[3].PosH.xy / input[3].PosH.w) * renderSize * 0.5f;

    // 2. Control polygon length represents the conservative upper bound of the curve length
    float polyLength = length(p1 - p0) + length(p2 - p1) + length(p3 - p2);
       
    int segments = clamp(ceil(polyLength / 3.0f), 1, 127);

    VertexOut v;
    //v.Color = input[0].Color;

    // 3. Evaluate Cubic Bezier
    for (int i = 0; i <= segments; ++i)
    {
        float t = (float) i / (float) segments;
        //float mt = 1.0f - t;
        
        float4 q0 = lerp(input[0].PosW, input[1].PosW, t);
        float4 q1 = lerp(input[1].PosW, input[2].PosW, t);
        float4 q2 = lerp(input[2].PosW, input[3].PosW, t);
        
        float4 r0 = lerp(q0, q1, t);
        float4 r1 = lerp(q1, q2, t);
        
        float4 worldPos = lerp(r0, r1, t);
        
        // Bernstein basis polynomials
        //float b0 = mt * mt * mt;
        //float b1 = 3.0f * mt * mt * t;
        //float b2 = 3.0f * mt * t * t;
        //float b3 = t * t * t;

        //float4 worldPos = b0 * input[0].PosW +
        //                  b1 * input[1].PosW +
        //                  b2 * input[2].PosW +
        //                  b3 * input[3].PosW;
        
        v.PosW = worldPos;
        v.PosH = mul(worldPos, viewProj);
        
        outputStream.Append(v);
    }
    outputStream.RestartStrip();
}