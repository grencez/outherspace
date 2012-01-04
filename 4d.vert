
varying vec3 N;
varying vec3 L;
varying vec3 E;
varying vec3 H;
uniform float alpha;
attribute vec3 hivert;
/* attribute vec3 hivnml; */

void main()
{
    vec4 v;
    v.xyz = hivert;
    v.w = 1.0;

    v = (1.0-alpha) * gl_Vertex + alpha * v;
    gl_Position = gl_ModelViewProjectionMatrix * v;

    gl_TexCoord[0] = gl_MultiTexCoord0;

    vec4 eyePosition = gl_ModelViewMatrix * gl_Vertex;
    vec4 eyeLightPos = gl_LightSource[0].position;

    /* v.xyz = hivnml; */
    N = normalize(gl_NormalMatrix * gl_Normal);
    L = normalize(eyeLightPos.xyz - eyePosition.xyz);
    E = -normalize(eyePosition.xyz);
    H = normalize(L + E);
}

