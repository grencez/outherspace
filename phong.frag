// phong.frag
// Fragmet shader for per-pixel Phong shading.

varying vec3 N;
varying vec3 L;
varying vec3 E;
varying vec3 H; // normalize(L+E);
varying vec2 txpt;
uniform sampler2D DiffuseTex;
uniform int HaveDiffuseTex;
uniform sampler2D NormalTex;
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
    vec4 diffuse = ((HaveDiffuseTex == 1)
                    ? texture2D(DiffuseTex, txpt)
                    : gl_FrontMaterial.diffuse);
    vec4 specular = gl_FrontMaterial.specular;

    ambient *= gl_FrontLightProduct[0].ambient;

    diffuse *= gl_FrontLightProduct[0].diffuse *
        max(dot(Normal, Light), 0.0);

    specular *= gl_FrontLightProduct[0].specular *
        pow(max(dot(Half, Normal), 0.0), gl_FrontMaterial.shininess);

    gl_FragColor = ambient + diffuse + specular;
}

