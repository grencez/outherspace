
#include "scene.h"

void copy_SceneTriangle (SceneTriangle* dst, const SceneTriangle* src)
{
    memcpy (dst, src, sizeof (SceneTriangle));
}

void cleanup_Scene (Scene* scene)
{
    if (scene->nverts > 0)  free (scene->verts);
    if (scene->nelems > 0)  free (scene->elems);
}

void vert_Scene (Point* dst, const Scene* scene, uint idx)
{
    copy_Point (dst, &scene->verts[idx]);
}

void elem_Scene (Triangle* dst, const Scene* scene, uint idx)
{
    uint i;
    const SceneTriangle* tri;

    tri = &scene->elems[idx];

    UFor( i, NTrianglePoints )
        vert_Scene (&dst->pts[i], scene, tri->pts[i]);
}

