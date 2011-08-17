
#include "material.h"

#include "pnm-image.h"

#include <math.h>
#ifdef SupportImage
#include <SDL_image.h>
#endif

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



    x = 1 / M_PI * zenith;
#if 0
    x *= .5;
#endif
    y = 1 / (2 * M_PI) * azimuth;

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

    bool
readin_Texture (Texture* texture, const char* pathname, const char* filename)
{
#ifdef SupportImage
    SDL_PixelFormat* fmt;
    SDL_Surface* surface;
    char* path;
    uint nrows, ncols;
    uint row;

    path = cat_filepath (pathname, filename);
    surface = IMG_Load (path);
    free (path);
    if (!surface)  return false;
    fmt = surface->format;

    texture->nrows = nrows = surface->h;
    texture->ncols = ncols = surface->w;
    texture->pixels = AllocT( byte, 3 * nrows * ncols );

    UFor( row, nrows )
    {
        uint surf_row, col;
        Uint8* scanline;

        surf_row = nrows - 1 - row;
        scanline = &((Uint8*) surface->pixels)[surf_row * surface->pitch];

        UFor( col, ncols )
        {
            uint idx;
            Uint8 r, g, b;
            Uint32 pixel;
            pixel = *(Uint32*)&scanline[col*fmt->BytesPerPixel];
            SDL_GetRGB (pixel, fmt, &r, &g, &b);
            idx = 3 * (row * ncols + col);
            texture->pixels[idx+0] = r;
            texture->pixels[idx+1] = g;
            texture->pixels[idx+2] = b;
        }
    }
    SDL_FreeSurface (surface);
    return true;
#else
    texture->pixels = readin_PPM_image (&texture->nrows, &texture->ncols,
                                        pathname, filename);
    return (texture->pixels != 0);
#endif
}

