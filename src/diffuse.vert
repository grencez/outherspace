// diffuse.vert
// Vertex shader for per-pixel "light-at-eye" shading.

varying vec3 N;
varying vec4 isect;

void main()
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

  isect = gl_Vertex;

  N = normalize(gl_NormalMatrix * gl_Normal);
}

