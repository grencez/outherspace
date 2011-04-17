
#ifndef MATERIAL_H_
#define MATERIAL_H_
#include "util.h"

#define NColors 3

struct material_struct
{
    real ambient[NColors]; /* Ka */
    real diffuse[NColors]; /* Kd */
    real specular[NColors]; /* Ks */
    real transmission[NColors]; /* Tf */
    real alpha; /* d */
    real shininess; /* Ns */
    bool illumination;
        /* TODO Texture map! */
};
typedef struct material_struct Material;
    
void init_Material (Material* mat);

#ifdef INCLUDE_SOURCE
#include "material.c"
#endif
#endif

