
#ifndef MATERIAL_H_
#define MATERIAL_H_
#include "util.h"

#include "simplex.h"

struct texture_struct
{
    uint nrows;
    uint ncols;
    byte* pixels;
};
typedef struct texture_struct Texture;

struct material_struct
{
    real ambient[NColors]; /* Ka */
    real diffuse[NColors]; /* Kd */
    real specular[NColors]; /* Ks */
    real transmission[NColors]; /* Tf */
    real opacity; /* d */
    real shininess; /* Ns */
    real optical_density; /* Ni */
    bool illumination;
    uint ambient_texture;
    uint diffuse_texture;
};
typedef struct material_struct Material;
    
void init_Material (Material* mat);
void
map_Texture (real* colors, const Texture* texture, const BaryPoint* p);
void
map_sky_Texture (real* colors, const Texture* texture, const Point* p);

bool
readin_Texture (Texture* texture, const char* pathname, const char* filename);

#ifdef INCLUDE_SOURCE
#include "material.c"
#endif
#endif

