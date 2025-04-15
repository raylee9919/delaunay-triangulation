R"(

layout (location = 0) in v3 vP;

out v2 gP;
flat out v2 gcenter;

uniform f32 scale;

void main()
{
    gP = scale*vP.xz;
    gcenter = scale*vP.xz;
    gl_Position = v4(scale*vP.xz, -1, 1);
}

)";
