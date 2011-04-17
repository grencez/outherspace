
#include "material.h"

void init_Material (Material* mat)
{
    uint i;
    UFor( i, NColors )
    {
        mat->ambient[i] = 0.2;
        mat->diffuse[i] = 0.8;
        mat->specular[i] = 1;
        mat->transmission[i] = 1;
    }

    mat->alpha = 1;
    mat->shininess = 0;
    mat->illumination = 1;
}

