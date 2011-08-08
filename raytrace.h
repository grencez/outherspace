
#ifndef RAYTRACE_H_
#ifndef __OPENCL_VERSION__
#define RAYTRACE_H_

#include "kdtree.h"
#include "kptree.h"
#include "scene.h"
#include "simplex.h"
#include "xfrm.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct ray_space_struct;
struct object_ray_space_struct;
struct point_light_source_struct;
struct ray_image_struct;
struct ray_cast_a_priori_struct;
typedef struct ray_space_struct RaySpace;
typedef struct object_ray_space_struct ObjectRaySpace;
typedef struct point_light_source_struct PointLightSource;
typedef struct ray_image_struct RayImage;
typedef struct ray_cast_a_priori_struct RayCastAPriori;

struct object_ray_space_struct
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
};

struct point_light_source_struct
{
    Point location;
    real intensity[NColors];
    bool diffuse;
};

struct ray_space_struct
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

struct ray_image_struct
{
    uint* hits;
    real* mags;
    byte* pixels;
    uint nrows;
    uint ncols;
    real hifov;  /* Large field of view parameter.*/
    bool perspective;
    real ambient[NColors];
    real view_light;
    bool shading_on;
    bool color_distance_on;
    uint nbounces_max;
};

struct ray_cast_a_priori_struct
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
const ObjectRaySpace*
ray_to_ObjectRaySpace (Point* ret_origin, Point* ret_dir,
                       const Point* origin, const Point* dir,
                       const RaySpace* space, uint objidx);
void
cast1_ObjectRaySpace (uint* ret_hit, real* ret_mag,
                      const Point* origin,
                      const Point* direct,
                      const ObjectRaySpace* object,
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
update_dynamic_RaySpace (RaySpace* space);
void
init_Scene_KDTreeGrid (KDTreeGrid* grid, const Scene* scene,
                       const BoundingBox* box);
void init_RayImage (RayImage* image);
void resize_RayImage (RayImage* image);
void downsample_RayImage (RayImage* image, uint inv);
void cleanup_RayImage (RayImage* image);
#ifdef INCLUDE_SOURCE
#include "raytrace.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

