
#ifndef SCENE_H_
#ifndef __OPENCL_VERSION__
#define SCENE_H_

#include "space.h"
#include "material.h"
#include "cx/table.h"
#include <stdio.h>
#endif  /* #ifndef __OPENCL_VERSION__ */

typedef struct SceneElement SceneElement;
typedef struct GeomSurf GeomSurf;
typedef struct GeomObj GeomObj;
typedef struct GeomInst GeomInst;
typedef struct Scene Scene;

struct SceneElement
{
    uint verts[NDimensions];
    uint vnmls[NDimensions];
    uint txpts[NDimensions];
    uint material;
    uint surface;
};

struct GeomSurf
{
    uint nelems;
    uint vidcs_offset;
    uint verts_offset;
    uint vnmls_offset;
    uint txpts_offset;
    uint material;
};

#ifndef Table_uint
#define Table_uint Table_uint
DeclTableT( uint, uint );
#endif

struct GeomObj
{
    BBox box;
    TableT(uint) surfidcs;
};

struct GeomInst
{
    Point centroid;
    PointXfrm orientation;
};

struct Scene
{
    uint ndims;
    uint nelems;
    uint nsurfs;
    uint nverts;
    uint nvnmls;
    uint ntxpts;
    uint nmatls;
    uint ntxtrs;
    SceneElement* elems; /* Elements.*/
    GeomSurf* surfs; /* Surfaces.*/
    uint* vidcs; /* Vertex indices.*/
    Point* verts; /* Vertices.*/
    Point* vnmls; /* Vertex normals.*/
    BaryPoint* txpts; /* Texture points.*/
    Material* matls; /* Materials.*/
    Texture* txtrs; /* Textures.*/
};

void
map_Scene (Scene* scene, const IAMap* map);
void
recenter_Scene (IAMap* map, const Scene* scene,
                const Point* new_centroid);

#ifndef __OPENCL_VERSION__
void init_Scene (Scene* scene);
void init_SceneElement (SceneElement* elem);
void
init_GeomSurf (GeomSurf* surf);
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
reshuffle_for_surfaces_Scene (Scene* scene);
void
setup_surfaces_Scene (Scene* scene);
void
condense_Scene (Scene* scene);
#ifdef IncludeC
#include "scene.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

