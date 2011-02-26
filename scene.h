
#ifndef SCENE_H_
#ifndef __OPENCL_VERSION__
#define SCENE_H_

#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct scene_triangle_struct
{
    uint pts[NTrianglePoints];
};
typedef struct scene_triangle_struct SceneTriangle;

struct scene_struct
{
    uint nverts;
    uint nelems;
    Point* verts;
    SceneTriangle* elems;
    BoundingBox box;
};
typedef struct scene_struct Scene;

#ifndef __OPENCL_VERSION__
void cleanup_Scene (Scene* scene);
void vert_Scene (Point* dst, const Scene* scene, uint idx);
void elem_Scene (Triangle* dst, const Scene* scene, uint idx);
#ifdef INCLUDE_SOURCE
#include "scene.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

