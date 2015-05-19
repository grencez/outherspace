// diffuse.frag
// Fragment shader for per-pixel "light-at-eye" shading.

varying vec3 N;
varying vec4 isect;

void main()
{
  vec3 Eye = -normalize((gl_ModelViewMatrix * isect).xyz);
  vec3 Normal = normalize(N);

#if 0
  vec4 diffuse = gl_FrontMaterial.diffuse;
#else
  vec3 diffuse;
  diffuse.r = 0.8;
  diffuse.g = 0.8;
  diffuse.b = 0.8;
#endif

  float d = abs(dot(Eye, Normal));
  gl_FragColor.rgb = diffuse.rgb * d;
  gl_FragColor.a = 1.0;
}

