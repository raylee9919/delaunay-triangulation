R"(

in v2 fP;
flat in v2 fcenter;

out v4 result_color;

uniform f32 radius = 0.01;
uniform f32 outline = 0.002;

void main()
{
    f32 d = distance(fP, fcenter);
    if (d < radius - outline) {
        result_color = v4(0.25,0.41,0.88,1);
    } else if (d < radius) {
        result_color = v4(1.0f);
    } else {
        discard;
    }
}

)";
