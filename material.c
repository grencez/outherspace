
#include "material.h"

#include "simplex.h"

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

    mat->opacity = 1;
    mat->shininess = 0;
    mat->optical_density = 1;
    mat->illumination = 1;
    mat->ambient_texture = Max_uint;
    mat->diffuse_texture = Max_uint;
}

    void
map_Texture (real* colors, const Texture* texture, const BaryPoint* p)
{
    uint i, row, col;
    const byte* pixels;
    row = texture->nrows * p->coords[1];
    col = texture->ncols * p->coords[0];
    if (row >= texture->nrows)
    {
        fprintf (stderr, "Passed nrows by:%u\n", row - texture->nrows);
        row = texture->nrows - 1;
    }
    if (col >= texture->ncols)
    {
        fprintf (stderr, "Passed nrows by:%u\n", col - texture->ncols);
        col = texture->ncols - 1;
    }

    pixels = &texture->pixels[NColors * (col + row * texture->ncols)];
    UFor( i, NColors )
        colors[i] = (real) pixels[i] / 255;
}

