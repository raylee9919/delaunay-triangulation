R"(
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in v2 gP[];
flat in v2 gcenter[];

out v2 fP;
flat out v2 fcenter;

uniform f32 halfdim = 0.01;

void main() {
    v4 pos = gl_in[0].gl_Position;

    gl_Position = (pos + vec4(-halfdim, -halfdim, 0.0, 0.0));
    fP = gP[0] + v2(-halfdim, -halfdim);
    fcenter = gcenter[0];
    EmitVertex();

    gl_Position = (pos + vec4(halfdim, -halfdim, 0.0, 0.0));
    fP = gP[0] + v2(halfdim, -halfdim);
    fcenter = gcenter[0];
    EmitVertex();

    gl_Position = (pos + vec4(-halfdim, halfdim, 0.0, 0.0));
    fP = gP[0] + v2(-halfdim, halfdim);
    fcenter = gcenter[0];
    EmitVertex();

    gl_Position = (pos + vec4(halfdim, halfdim, 0.0, 0.0));
    fP = gP[0] + v2(halfdim, halfdim);
    fcenter = gcenter[0];
    EmitVertex();

    EndPrimitive();
}
)";
