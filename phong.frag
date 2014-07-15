// phong.frag
// Fragmet shader for per-pixel Phong shading.

varying vec3 N;
varying vec3 L;
varying vec3 E;
varying vec3 H; // normalize(L+E);
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
        /* This part is always == N. Normal mapping doesn't work yet!*/
    vec3 Normal = ((HaveNormalTex == 1)
                    ? (gl_NormalMatrix *
                       texture2D(NormalTex, txpt).xyz)
                    : N);
    Normal = normalize(Normal);

    vec3 Light  = normalize(L);

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
    vec3 Half  = normalize(H);
    float tcos = max(dot(Half, Normal), 0.0);
#else
    vec3 Refl  = - reflect(Light, Normal);
    vec3 Eye   = normalize(E);
    float tcos = max(dot(Refl, Eye), 0.0);
#endif
    specular *= (att *
                 gl_LightSource[0].specular *
                 pow(tcos, gl_FrontMaterial.shininess));

    if (DiffuseCameraOn == 1) {
      gl_FragColor = abs(dot(normalize(E), Normal));
    }
    else {
      gl_FragColor = ambient + diffuse + specular;
    }
}

