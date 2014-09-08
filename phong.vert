// phong.vert
// Vertex shader for per-pixel Phong shading.

varying vec3 N;
varying vec4 isect;
varying vec2 txpt;

void main()
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
  txpt = gl_MultiTexCoord0.xy;

  isect = gl_Vertex;

  N = normalize(gl_NormalMatrix * gl_Normal);
}

