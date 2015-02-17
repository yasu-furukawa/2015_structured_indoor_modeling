uniform mat4 global_to_local;
varying vec3 local;

void main()
{
//gl_TexCoord[0] = gl_MultiTexCoord0;
//gl_TexCoord[1] = gl_MultiTexCoord1;
gl_Position = ftransform();
gl_TexCoord[0] = gl_MultiTexCoord0;

local = (global_to_local * gl_Vertex).xyz;

// global_to_local * gl_Vertex;
}
