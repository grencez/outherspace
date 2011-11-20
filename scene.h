
#ifndef SCENE_H_
#ifndef __OPENCL_VERSION__
#define SCENE_H_

#include "simplex.h"
#include "material.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

typedef struct SceneElement SceneElement;
typedef struct Scene Scene;

struct SceneElement
{
    uint verts[NDimensions];
    uint vnmls[NDimensions];
    uint txpts[NDimensions];
    uint material;
};

struct Scene
{
    uint nelems;
    uint nverts;
    uint nvnmls;
    uint ntxpts;
    uint nmatls;
    uint ntxtrs;
    SceneElement* elems; /* Elements.*/
    Point* verts; /* Vertices.*/
    Point* vnmls; /* Vertex normals.*/
    BaryPoint* txpts; /* Texture points.*/
    Material* matls; /* Materials.*/
    Texture* txtrs; /* Textures.*/
};

void
xlate_Scene (Scene* scene, const Point* displacement);
void
xfrm_Scene (Scene* scene, const PointXfrm* xfrm);
void
recenter_Scene (Scene* scene, const Point* new_centroid);

#ifndef __OPENCL_VERSION__
void init_Scene (Scene* scene);
void init_SceneElement (SceneElement* elem);
void copy_SceneElement (SceneElement* dst, const SceneElement* src);
void cleanup_Scene (Scene* scene);
void
copy_Scene (Scene* dst, const Scene* src);
void
concat0_Scene (Scene* scene, Scene* a);
void
output_SceneElement (FILE* out, const Scene* scene, uint ei);
void vert_Scene (Point* dst, const Scene* scene, uint idx);
void
simplex_Scene (Simplex* dst, const Scene* scene, uint idx);
void
setup_1elem_Scene (Scene* scene);
void
interpolate_Scene (Scene* dst, uint k, uint nscenes, const Scene* scenes);
void
interpolate1_Scene (Scene* dst, real alpha,
                    const Scene* scene_a, const Scene* scene_b);
void
condense_Scene (Scene* scene);
#ifdef INCLUDE_SOURCE
#include "scene.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

