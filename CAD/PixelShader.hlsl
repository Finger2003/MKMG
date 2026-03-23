struct VSOut
{
    float4 pos : SV_POSITION;
    float4 color : COLOR;
};

float4 main(VSOut i) : SV_TARGET
{
    return i.color;
}