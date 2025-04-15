R"(

layout (location = 0) in v3 vP;
out v3 fP;
uniform f32 scale;

void main()
{
    fP = scale*vP;
    gl_Position = v4(scale*vP.xzy, 1);
}

)";
