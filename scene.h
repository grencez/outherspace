
#ifndef SCENE_H_
#ifndef __OPENCL_VERSION__
#define SCENE_H_

#include "simplex.h"
#include "material.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct scene_element_struct
{
    uint pts[NDimensions];
    uint txpts[NDimensions];
    uint material;
};
typedef struct scene_element_struct SceneElement;

struct scene_struct
{
    uint nelems;
    uint nverts;
    uint ntxpts;
    uint nmatls;
    uint ntxtrs;
    SceneElement* elems; /* Elements.*/
    Point* verts; /* Vertices.*/
    BaryPoint* txpts; /* Texture points.*/
    Material* matls; /* Materials.*/
    Texture* txtrs; /* Textures.*/
};
typedef struct scene_struct Scene;

#ifndef __OPENCL_VERSION__
void init_Scene (Scene* scene);
void init_SceneElement (SceneElement* elem);
void copy_SceneElement (SceneElement* dst, const SceneElement* src);
void cleanup_Scene (Scene* scene);
void
output_SceneElement (FILE* out, const Scene* scene, uint ei);
void vert_Scene (Point* dst, const Scene* scene, uint idx);
void
simplex_Scene (Simplex* dst, const Scene* scene, uint idx);
void
interpolate_Scene (Scene* dst, uint k, uint nscenes, const Scene* scenes);
#ifdef INCLUDE_SOURCE
#include "scene.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

