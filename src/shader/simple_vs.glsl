R"(

layout (location = 0) in v3 vP;
out v3 fP;

void main()
{
    fP = vP;
    gl_Position = v4(vP.xzy, 1);
}

)";
