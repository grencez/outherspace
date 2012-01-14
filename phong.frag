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

void main()
{
        /* This part is always == N. Normal mapping doesn't work yet!*/
    vec3 Normal = ((HaveNormalTex == 1)
                    ? (gl_NormalMatrix *
                       texture2D(NormalTex, txpt).xyz)
                    : N);
    Normal = normalize(Normal);

    vec3 Light  = normalize(L);
    vec3 Eye    = normalize(E);
    vec3 Half   = normalize(H);

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

    ambient *= gl_LightSource[0].ambient;

    diffuse *= gl_LightSource[0].diffuse *
        max(dot(Normal, Light), 0.0);

    specular *= gl_LightSource[0].specular *
        pow(max(dot(Half, Normal), 0.0), gl_FrontMaterial.shininess);

    gl_FragColor = ambient + diffuse + specular;
}

