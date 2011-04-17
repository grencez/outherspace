
#ifndef RAYTRACE_H_
#ifndef __OPENCL_VERSION__
#define RAYTRACE_H_

#include "kdtree.h"
#include "scene.h"
#include "simplex.h"
#include "xfrm.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

#define DirDimension (NDimensions - 1)

struct ray_space_struct;
struct ray_space_object_struct;
struct ray_image_struct;
struct multi_ray_cast_params_struct;
typedef struct ray_space_struct RaySpace;
typedef struct ray_space_object_struct RaySpaceObject;
typedef struct ray_image_struct RayImage;
typedef struct multi_ray_cast_params_struct MultiRayCastParams;

struct ray_space_struct
{
    uint nelems;
    Triangle* elems;
    BarySimplex* simplices;
    Scene scene;
    KDTree tree;
    uint nobjects;
    RaySpaceObject* objects;
    KDTree object_tree;
};

struct ray_space_object_struct
{
    real radius;
    Point centroid;
    PointXfrm orientation;
    RaySpace space;
};

struct ray_image_struct
{
    uint* hits;
    real* mags;
    byte* pixels;
    uint nrows;
    uint ncols;
    real view_light;
    bool shading_on;
    bool color_distance_on;
};

struct multi_ray_cast_params_struct
{
    Point origin;
    Point dir_start;
    Point dir_delta[2];
    BoundingBox box;
    uint nelems;
    uint npixels[2];
    bool inside_box;
};

void dir_from_MultiRayCastParams (Point* dir, uint row, uint col,
                                  const MultiRayCastParams* params);
void
fill_pixel (byte* ret_red, byte* ret_green, byte* ret_blue,
            real mag,
            const RayImage* image,
            const Point* dir,
            const BarySimplex* simplex,
            const Material* material);
void
cast_ray (uint* restrict ret_hit,
          real* restrict ret_mag,
          const Point* restrict origin,
          const Point* restrict dir,
          const uint nelems,
          __global const Triangle* restrict elems,
          __global const uint* restrict elemidcs,
          __global const KDTreeNode* restrict nodes,
          __global const BoundingBox* restrict box,
          bool inside_box);

#ifndef __OPENCL_VERSION__
void
rays_to_hits_fish (RayImage* restrict image,
                   const RaySpace* space,
                   const Point* origin,
                   const PointXfrm* view_basis,
                   real view_angle);
void
rays_to_hits_fixed_plane (uint* hits, real* mags,
                          uint nrows, uint ncols,
                          const RaySpace* space, real zpos);
void
rays_to_hits_parallel (RayImage* restrict image,
                       const RaySpace* space,
                       const Point* origin,
                       const PointXfrm* view_basis,
                       real view_angle);
void
setup_ray_pixel_deltas (Point* dir_start,
                        Point* row_delta,
                        Point* col_delta,
                        uint nrows, uint ncols,
                        const PointXfrm* view_basis,
                        real view_width);
void
rays_to_hits (RayImage* restrict image,
              const RaySpace* space,
              const Point* origin,
              const PointXfrm* view_basis,
              real view_angle);
void
rays_to_hits_row (RayImage* image, uint row,
                  const RaySpace* restrict space,
                  const Point* restrict origin,
                  const PointXfrm* restrict view_basis,
                  const Point* dir_start,
                  const Point* row_delta,
                  const Point* col_delta,
                  bool inside_box);

void build_MultiRayCastParams (MultiRayCastParams* params,
                               uint nrows, uint ncols,
                               const RaySpace* space,
                               const Point* origin,
                               const PointXfrm* view_basis,
                               real view_angle);

void init_RaySpace (RaySpace* space);
void cleanup_RaySpace (RaySpace* space);
void partition_RaySpace (RaySpace* space);
void init_RayImage (RayImage* image);
void resize_RayImage (RayImage* image);
void downsample_RayImage (RayImage* image, uint inv);
void cleanup_RayImage (RayImage* image);
#ifdef INCLUDE_SOURCE
#include "raytrace.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

