
#ifndef SCENE_H_
#define SCENE_H_

#include "space.h"

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
};
typedef struct scene_struct Scene;

void cleanup_Scene (Scene* scene);
void vert_Scene (Point* dst, const Scene* scene, uint idx);
void elem_Scene (Triangle* dst, const Scene* scene, uint idx);

#include "scene.c"
#endif

