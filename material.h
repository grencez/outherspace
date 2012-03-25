
#ifndef MATERIAL_H_
#define MATERIAL_H_
#include "util.h"

#include "affine.h"
#include "simplex.h"

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
    real ambient[NColors]; /* Ka */
    real diffuse[NColors]; /* Kd */
    real specular[NColors]; /* Ks */
    real emissive[NColors]; /* Ke */
    real transmission[NColors]; /* Tf */
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
map_Texture (real* colors, const Texture* texture, const BaryPoint* p);
void
map_bump_Texture (Point* normal, const Texture* texture, const BaryPoint* p);
void
map_sky_Texture (real* colors, const Texture* texture, const Point* p);

bool
readin_Texture (Texture* texture, const char* pathname, const char* filename);

void
remap_bumps_Texture (Texture* texture, const AffineMap* map);

#ifdef IncludeC
#include "material.c"
#endif
#endif

