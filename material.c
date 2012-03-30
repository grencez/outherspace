
#include "material.h"

#include "color.h"
#include "point.h"
#include "pnm-image.h"
#include "xfrm.h"

#include <math.h>
#ifdef SupportImage
#include <SDL_image.h>
#endif

void init_Material (Material* mat)
{
    set_Color (&mat->ambient, 0.2);
    set_Color (&mat->diffuse, 0.8);
    set_Color (&mat->specular, 1);
    set_Color (&mat->emissive, 0);
    set_Color (&mat->transmission, 1);

    mat->opacity = 1;
    mat->shininess = 0;
    mat->optical_density = 1;
    mat->reflective = false;
    mat->ambient_texture = Max_uint;
    mat->diffuse_texture = Max_uint;
    mat->specular_texture = Max_uint;
    mat->emissive_texture = Max_uint;
    mat->bump_texture = Max_uint;
}

    void
copy_Texture (Texture* dst, const Texture* src)
{
    const uint npixels = src->nrows * src->ncols;
    byte* pixels;

    pixels = AllocT( byte, (npixels) * src->pixelsz );
    CopyT( byte, pixels, src->pixels, 0, npixels * src->pixelsz );

    *dst = *src;
    dst->pixels = pixels;
}

    /** Coords are row/col.**/
static void
map_coords_Texture (uint* coords, const Texture* texture, const BaryPoint* p)
{
    uint row, col;
    real x;
    x = fmod (texture->nrows * p->coords[1], texture->nrows);
    row = ((uint) x + texture->nrows) % texture->nrows;
    x = fmod (texture->nrows * p->coords[0], texture->ncols);
    col = ((uint) x + texture->ncols) % texture->ncols;
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
    coords[0] = row;
    coords[1] = col;
}

    real
map_Texture (Color* color, const Texture* texture, const BaryPoint* p)
{
    uint i;
    uint coords[2];
    const byte* pixels;

    map_coords_Texture (coords, texture, p);
    i = coords[1] + coords[0] * texture->ncols;
    pixels = &texture->pixels[texture->pixelsz * i];

    UFor( i, NColors )
        color->coords[i] = pixels[i] * (1.0 / 255);
    return (texture->alpha ? (real) pixels[3] / 255 : 1);
}

    void
map_bump_Texture (Point* normal, const Texture* texture, const BaryPoint* p)
{
    uint i;
    uint coords[2];
    const signed char* pixels;

    map_coords_Texture (coords, texture, p);
    i = coords[1] + coords[0] * texture->ncols;
    pixels = (signed char*) &texture->pixels[texture->pixelsz * i];

    zero_Point (normal);
    UFor( i, 3 )
        normal->coords[i] = pixels[i];
}

    void
map_sky_Texture (Color* color, const Texture* texture, const Point* p)
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

    pixels = &texture->pixels[texture->pixelsz * (col + row * texture->ncols)];
    UFor( i, NColors )
        color->coords[i] = (real) pixels[i] / 255;
}

#ifdef SupportImage
static bool
readin_SDL_Image (Texture* texture,
                  const char* pathname, const char* filename)
{
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
    texture->pixelsz = fmt->BytesPerPixel;
    texture->alpha = (texture->pixelsz == 4);
    texture->pixels = AllocT( byte, texture->pixelsz * nrows * ncols );

    UFor( row, nrows )
    {
        uint surf_row, col;
        Uint8* scanline;

        surf_row = nrows - 1 - row;
        scanline = &((Uint8*) surface->pixels)[surf_row * surface->pitch];

        UFor( col, ncols )
        {
            uint idx;
            Uint8 r, g, b, a;
            union LocalUnion
            {
                Uint32 u;
                byte bytes[Ceil_uint( 32, NBitsInByte )];
            } pixel;

            pixel.u = 0;
            UFor( idx, fmt->BytesPerPixel )
                pixel.bytes[idx] = scanline[idx + col*fmt->BytesPerPixel];

            idx = texture->pixelsz * (row * ncols + col);
            if (texture->alpha)
            {
                SDL_GetRGBA (pixel.u, fmt, &r, &g, &b, &a);
                texture->pixels[idx+3] = a;
            }
            else
            {
                SDL_GetRGB (pixel.u, fmt, &r, &g, &b);
            }
            texture->pixels[idx+0] = r;
            texture->pixels[idx+1] = g;
            texture->pixels[idx+2] = b;
        }
    }
    SDL_FreeSurface (surface);
    return true;
}
#endif  /* defined(SupportImage) */

    bool
readin_Texture (Texture* texture, const char* pathname, const char* filename)
{
    if (strends_with (filename, ".ppm"))
    {
        texture->alpha = false;
        texture->pixelsz = 3;
        texture->pixels = readin_PPM_image (&texture->nrows, &texture->ncols,
                                            pathname, filename);
        return (texture->pixels != 0);
    }
#ifdef SupportImage
    return readin_SDL_Image (texture, pathname, filename);
#else
    return false;
#endif
}

    void
remap_bumps_Texture (Texture* texture, const AffineMap* map)
{
    uint i, n;
    signed char* pixels;

    n = texture->nrows * texture->ncols;
    pixels = (signed char*) texture->pixels;

    UFor( i, n )
    {
        Point p;
        real m;
        uint j;
        zero_Point (&p);
        UFor( j, 3 )
            p.coords[j] = pixels[3*i+j];

        xfrm_Point (&p, &map->xfrm, &p);
        
        m = match_real (abs_real (p.coords[0]),
                        match_real (p.coords[1], p.coords[2]));
        m = 127.5 / m;

        UFor( j, 3 )
            pixels[3*i+j] = (signed char) (int) (m * p.coords[j]);
    }
}

