// phong.vert
// Vertex shader for per-pixel Phong shading.

varying vec3 N;
varying vec3 L;
varying vec3 E;
varying vec3 H;
varying vec2 txpt;

void main()
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    txpt = gl_MultiTexCoord0.xy;

    vec4 eyePosition = gl_ModelViewMatrix * gl_Vertex;
    vec4 eyeLightPos = gl_LightSource[0].position;

    N = normalize(gl_NormalMatrix * gl_Normal);
    L = normalize(eyeLightPos.xyz - eyePosition.xyz);
    E = -normalize(eyePosition.xyz);
    H = normalize(L + E);
}

