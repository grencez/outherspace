// phong.frag
// Fragmet shader for per-pixel Phong shading.

varying vec3 N;
varying vec4 isect;
varying vec2 txpt;
uniform sampler2D AmbientTex;
uniform sampler2D DiffuseTex;
uniform sampler2D SpecularTex;
uniform sampler2D NormalTex;
uniform int HaveAmbientTex;
uniform int HaveDiffuseTex;
uniform int HaveSpecularTex;
uniform int HaveNormalTex;
uniform int DiffuseCameraOn;

void main()
{
#if 1
  vec4 eyePosition = gl_ModelViewMatrix * isect;
  vec4 eyeLightPos = gl_LightSource[0].position;
  vec3 Light = normalize(eyeLightPos.xyz - eyePosition.xyz);
  vec3 Eye = -normalize(eyePosition.xyz);
  /* This part is always == N. Normal mapping doesn't work yet!*/
  vec3 Normal = ((HaveNormalTex == 1)
                 ? (gl_NormalMatrix *
                    texture2D(NormalTex, txpt).xyz)
                 : N);
  Normal = normalize(Normal);

  vec4 ambient = gl_FrontMaterial.ambient;
  vec4 diffuse = gl_FrontMaterial.diffuse;
  vec4 specular = gl_FrontMaterial.specular;

  if (HaveAmbientTex == 1)
  {
    vec4 tex = texture2D(AmbientTex, txpt);
    ambient.rgb += (tex.rgb - ambient.rgb) * tex.a;
  }
  if (HaveDiffuseTex == 1)
  {
    vec4 tex = texture2D(DiffuseTex, txpt);
    diffuse.rgb += (tex.rgb - diffuse.rgb) * tex.a;
  }
  if (HaveSpecularTex == 1)
  {
    vec4 tex = texture2D(SpecularTex, txpt);
    specular.rgb += (tex.rgb - specular.rgb) * tex.a;
  }

  float spotEffect = 1.0;
  if (gl_LightSource[0].spotCutoff <= 90.0) {
    spotEffect = dot(normalize(gl_LightSource[0].spotDirection), -Light);
    if (spotEffect > gl_LightSource[0].spotCosCutoff) {
      spotEffect = pow(spotEffect, gl_LightSource[0].spotExponent);
    }
    else {
      spotEffect = 0.0;
    }
  }

  float att = (spotEffect /
               (gl_LightSource[0].constantAttenuation
                + gl_FogFragCoord *
                gl_LightSource[0].linearAttenuation +
                + gl_FogFragCoord * gl_FogFragCoord *
                gl_LightSource[0].quadraticAttenuation));

  ambient *= gl_LightModel.ambient + att * gl_LightSource[0].ambient;

  diffuse *= (att *
              gl_LightSource[0].diffuse *
              max(dot(Normal, Light), 0.0));

#ifdef USE_HALFWAY_VECTOR
  vec3 Half  = normalize(Light + Eye);
  float tcos = max(dot(Half, Normal), 0.0);
#else
  vec3 Refl  = - reflect(Light, Normal);
  float tcos = max(dot(Refl, Eye), 0.0);
#endif
  specular *= (att *
               gl_LightSource[0].specular *
               pow(tcos, gl_FrontMaterial.shininess));

  if (DiffuseCameraOn == 1) {
    float d = abs(dot(Eye, Normal));
    gl_FragColor.r = d;
    gl_FragColor.g = d;
    gl_FragColor.b = d;
  }
  else {
    gl_FragColor = ambient + diffuse + specular;
  }
#else
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
#endif
}

