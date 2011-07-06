
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

    void
map_sky_Texture (real* colors, const Texture* texture, const Point* p)
{
    uint i, row, col;
    real x, y, z;
    real zenith, azimuth;
    const byte* pixels;
    
    x = p->coords[0];
    y = p->coords[1];
    z = p->coords[2];

        /* TODO: In 4D, this does not work!(?)*/
    zenith = acos (x);

    azimuth = atan2_real (y, z);
    if (azimuth < 0)  azimuth += 2 * M_PI;

    if (x > 0)
    {
#if 0
        AssertApprox(x, sin (M_PI/2 - zenith), 1, 1e3);

        printf ("%f  %f\n", y, cos (M_PI/2 - zenith) * cos (- azimuthcc));
        AssertApprox(y,
                     cos (M_PI/2 - zenith) * cos (- azimuthcc),
                     1, 1e3);
#endif
#if 0
        printf ("%f  %f\n", z, cos (M_PI/2 - zenith) * sin (- azimuthcc));
        AssertApprox(z,
                     cos (M_PI/2 - zenith) * sin (- azimuthcc),
                     1, 1e3);
#endif
    }



#if 0
    x = zenith / (M_PI / 2);
#else
    x = zenith / M_PI;
#endif
    y = azimuth / (2 * M_PI);

    /*
    assert (x > -0.01);
    assert (x < +1.01);
    fprintf(stderr,"y=%f\n", y);
    assert (y > -0.01);
    assert (y < +1.01);
    */
    x = clamp_real (1 - x, 0, 1);
    y = clamp_real (y, 0, 1);

    row = (uint) (x * texture->nrows);
    col = (uint) (y * texture->ncols);

    if (row >= texture->nrows)  row = texture->nrows - 1;
    if (col >= texture->ncols)  col = texture->ncols - 1;

    pixels = &texture->pixels[NColors * (col + row * texture->ncols)];
    UFor( i, NColors )
        colors[i] = (real) pixels[i] / 255;
}

