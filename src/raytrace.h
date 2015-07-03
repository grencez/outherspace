
#ifndef RAYTRACE_H_
#ifndef __OPENCL_VERSION__
#define RAYTRACE_H_

#include "kdtree.h"
#include "kptree.h"
#include "scene.h"
#include "space.h"
#include "cx/bstree.h"
#include "cx/table.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

typedef struct LightCutNode LightCutNode;
typedef struct LightCutTree LightCutTree;
typedef struct RaySpace RaySpace;
typedef struct ObjectRaySpace ObjectRaySpace;
typedef struct PointLightSource PointLightSource;
typedef struct RayImage RayImage;
typedef struct RayCastAPriori RayCastAPriori;
typedef struct RayHit RayHit;

struct LightCutNode
{
    BSTNode bst;
    Ray iatt;  /* location + orientation */
    Color color;
    BBox bbox;
    real cos_bound;
    real area;
        /* Leaf nodes.*/
    uint nlights;
    LightCutNode* lights;
};
DeclTableT( LightCutNode, LightCutNode );

struct LightCutTree
{
    LightCutNode sentinel;
    BSTree bst;
    TableT( LightCutNode ) nodes;
    real area;
};


struct ObjectRaySpace
{
    Point centroid;
    PointXfrm orientation;
    uint nelems;
    __global Simplex* elems;
    __global BarySimplex* simplices;
    Scene scene;
    BBox box;
    KDTree tree;
    KPTree verttree;
    bool visible;
};

struct PointLightSource
{
    Point location;
    Point direct;
    Color intensity;
    bool diffuse;
    bool hemisphere;
    bool on;
};

struct RaySpace
{
    ObjectRaySpace main;
    uint nobjects;
    uint nlights;
    ObjectRaySpace* objects;
    PointLightSource* lights;
    LightCutTree lightcuts;
    uint skytxtr; /* Index of sky texture in main object.*/

    bool partition;  /* When false, the below two are unused.*/
    KDTree object_tree;
    BBox box;
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
    Bool perspective;
    real ambient[NColors];
    real view_light;
    Bool shading_on;
    Bool color_distance_on;
    Bool diffuse_camera_on;
    uint nbounces_max;
  Bool culling_on;
};

struct RayCastAPriori
{
    Point origin;
    PointXfrm basis;
    real up_scale;
    real rt_scale;
    Bool inside_box;
};

struct RayHit
{
    Point isect;
    Point incid;
    Point normal;
    Trit front;
    real mag;
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
cast_Ray (uint* restrict ret_hit, real* restrict ret_mag,
          const Ray* restrict ray,
          const uint nelems,
          __global const uint* restrict elemidcs,
          __global const KDTreeNode* restrict nodes,
          __global const BarySimplex* restrict simplices,
          __global const Simplex* restrict tris,
          const BBox* restrict box,
          bool inside_box,
          Trit front);
void
cast1_ObjectRaySpace (uint* ret_hit, real* ret_mag,
                      const Point* origin,
                      const Point* direct,
                      const ObjectRaySpace* object,
                      bool inside_box,
                      Trit front);
void
cast1_RaySpace (uint* ret_hit, real* ret_mag,
                uint* ret_objidx,
                const Ray* ray,
                const RaySpace* space,
                Trit front);
bool
cast_to_light (const RaySpace* restrict space,
               const Ray* restrict ray,
               Trit front,
               real magtolight);
void
cast_nopartition (uint* ret_hit,
                  real* ret_mag,
                  uint* ret_object,
                  const RaySpace* restrict space,
                  const Point* restrict origin,
                  const Point* restrict dir,
                  bool inside_box,
                  Trit front,
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
                      const BBox* box);
void
ray_from_RayCastAPriori (Ray* ray,
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
                       const BBox* box);
void init_RayImage (RayImage* image);
void resize_RayImage (RayImage* image);
void restride_RayImage (RayImage* image);
void unstride_RayImage (RayImage* image);
void downsample_RayImage (RayImage* image, uint inv);
void cleanup_RayImage (RayImage* image);
#endif  /* #ifndef __OPENCL_VERSION__ */

void
find_holes (TableT(uint2)* holes, const RayImage* ray_image, uint nelems);
#endif

