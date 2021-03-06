
#ifndef MATERIAL_H_
#ifndef __OPENCL_VERSION__
#define MATERIAL_H_
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

typedef struct Texture Texture;
typedef struct Material Material;

struct Texture
{
    uint nrows;
    uint ncols;
    byte* pixels;
    bool alpha;
    byte pixelsz;
};

struct Material
{
    Color ambient; /* Ka */
    Color diffuse; /* Kd */
    Color specular; /* Ks */
    Color emissive; /* Ke */
    Color transmission; /* Tf */
    real opacity; /* d */
    real shininess; /* Ns */
    real optical_density; /* Ni */
    bool reflective;
    uint ambient_texture;
    uint diffuse_texture;
    uint specular_texture;
    uint emissive_texture;
    uint bump_texture;
};

void init_Material (Material* mat);
void
copy_Texture (Texture* dst, const Texture* src);
real
map_Texture (Color* color, const Texture* texture, const BaryPoint* p);
void
map_bump_Texture (Point* normal, const Texture* texture, const BaryPoint* p);
void
map_sky_Texture (Color* color, const Texture* texture, const Point* p);

bool
readin_Texture (Texture* texture, const char* pathname, const char* filename);

void
remap_bumps_Texture (Texture* texture, const IAMap* map);

#endif

