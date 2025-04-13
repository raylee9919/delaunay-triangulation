R"(

layout (location = 0) in v3 vP;

out v2 gP;
flat out v2 gcenter;

void main()
{
    gP = vP.xz;
    gcenter = vP.xz;
    gl_Position = v4(vP.xz, -1, 1);
}

)";
