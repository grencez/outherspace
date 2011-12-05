
#ifndef RAYTRACE_H_
#ifndef __OPENCL_VERSION__
#define RAYTRACE_H_

#include "kdtree.h"
#include "kptree.h"
#include "scene.h"
#include "simplex.h"
#include "xfrm.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

typedef struct RaySpace RaySpace;
typedef struct ObjectRaySpace ObjectRaySpace;
typedef struct PointLightSource PointLightSource;
typedef struct RayImage RayImage;
typedef struct RayCastAPriori RayCastAPriori;

struct ObjectRaySpace
{
    Point centroid;
    PointXfrm orientation;
    uint nelems;
    Simplex* elems;
    BarySimplex* simplices;
    Scene scene;
    BoundingBox box;
    KDTree tree;
    KPTree verttree;
    bool visible;
};

struct PointLightSource
{
    Point location;
    real intensity[NColors];
    bool diffuse;
    bool on;
};

struct RaySpace
{
    ObjectRaySpace main;
    uint nobjects;
    uint nlights;
    ObjectRaySpace* objects;
    PointLightSource* lights;
    uint skytxtr; /* Index of sky texture in main object.*/

    bool partition;  /* When false, the below two are unused.*/
    KDTree object_tree;
    BoundingBox box;
};

struct RayImage
{
    uint* hits;
    real* mags;
    byte* pixels;
    uint nrows;
    uint ncols;
    uint stride;
    real hifov;  /* Large field of view parameter.*/
    bool perspective;
    real ambient[NColors];
    real view_light;
    bool shading_on;
    bool color_distance_on;
    uint nbounces_max;
};

struct RayCastAPriori
{
    Point origin;
    Point dir_start;
    Point row_delta;
    Point col_delta;
    bool inside_box;
};

void
map_vertex_normal (Point* normal,
                   const Point* vnmls,
                   const SceneElement* elem,
                   const Point* bpoint);
void
map_isect_height (Point* ret_isect,
                  const Point* isect,
                  const Point* bpoint,
                  const SceneElement* elem,
                  const Point* verts,
                  const Point* vnmls);
void
cast1_ObjectRaySpace (uint* ret_hit, real* ret_mag,
                      const Point* origin,
                      const Point* direct,
                      const ObjectRaySpace* object,
                      bool inside_box);
void
cast_nopartition (uint* ret_hit,
                  real* ret_mag,
                  uint* ret_object,
                  const RaySpace* restrict space,
                  const Point* restrict origin,
                  const Point* restrict dir,
                  bool inside_box,
                  uint ignore_object);

#ifndef __OPENCL_VERSION__
void
rays_to_hits_fish (RayImage* restrict image,
                   const RaySpace* space,
                   const Point* origin,
                   const PointXfrm* view_basis,
                   real view_angle);
void
rays_to_hits_fixed_plane (uint* hits, real* mags,
                          uint nrows, uint ncols, uint stride,
                          const RaySpace* space, real zpos);
void
setup_RayCastAPriori (RayCastAPriori* dst,
                      const RayImage* image,
                      const Point* origin,
                      const PointXfrm* view_basis,
                      const BoundingBox* box);
void
ray_from_RayCastAPriori (Point* origin, Point* dir,
                         const RayCastAPriori* known,
                         uint row, uint col,
                         const RayImage* image);
void
cast_partial_RayImage (RayImage* restrict image,
                       uint row_off, uint row_nul,
                       const RaySpace* restrict space,
                       const RayCastAPriori* restrict known);
void
cast_RayImage (RayImage* restrict image,
               const RaySpace* restrict space,
               const Point* restrict origin,
               const PointXfrm* restrict view_basis);

void
init_RaySpace (RaySpace* space);
void
init_ObjectRaySpace (ObjectRaySpace* object);
void
init_PointLightSource (PointLightSource* light);
void
copy_PointLightSource (PointLightSource* dst, const PointLightSource* src);
void
cleanup_RaySpace (RaySpace* space);
void
cleanup_ObjectRaySpace (ObjectRaySpace* object);
void
init_filled_RaySpace (RaySpace* space);
void
init_filled_ObjectRaySpace (ObjectRaySpace* object);
void
init_trivial_ObjectRaySpace (ObjectRaySpace* object);
void
update_trivial_ObjectRaySpace (ObjectRaySpace* object);
void
update_dynamic_RaySpace (RaySpace* space);
void
init_Scene_KDTreeGrid (KDTreeGrid* grid, const Scene* scene,
                       const BoundingBox* box);
void init_RayImage (RayImage* image);
void resize_RayImage (RayImage* image);
void restride_RayImage (RayImage* image);
void unstride_RayImage (RayImage* image);
void downsample_RayImage (RayImage* image, uint inv);
void cleanup_RayImage (RayImage* image);
#ifdef INCLUDE_SOURCE
#include "raytrace.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

