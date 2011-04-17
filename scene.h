
#ifndef SCENE_H_
#ifndef __OPENCL_VERSION__
#define SCENE_H_

#include "simplex.h"
#include "material.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct scene_element_struct
{
    uint pts[NTrianglePoints];
    uint material;
};
typedef struct scene_element_struct SceneElement;

struct scene_struct
{
    uint nverts;
    uint nelems;
    uint nmatls;
    Point* verts;
    SceneElement* elems;
    Material* matls;
    BoundingBox box;
};
typedef struct scene_struct Scene;

#ifndef __OPENCL_VERSION__
void init_Scene (Scene* scene);
void init_SceneElement (SceneElement* elem);
void copy_SceneElement (SceneElement* dst, const SceneElement* src);
void cleanup_Scene (Scene* scene);
void vert_Scene (Point* dst, const Scene* scene, uint idx);
void elem_Scene (Triangle* dst, const Scene* scene, uint idx);
#ifdef INCLUDE_SOURCE
#include "scene.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

