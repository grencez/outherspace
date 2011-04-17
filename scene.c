
#include "scene.h"

void init_Scene (Scene* scene)
{
    scene->nverts = 0;
    scene->nelems = 0;
    scene->nmatls = 0;
}

void init_SceneElement (SceneElement* elem)
{
    elem->material = Max_uint;
}

void copy_SceneElement (SceneElement* dst, const SceneElement* src)
{
    memcpy (dst, src, sizeof (SceneElement));
}

void cleanup_Scene (Scene* scene)
{
    if (scene->nverts > 0)  free (scene->verts);
    if (scene->nelems > 0)  free (scene->elems);
    if (scene->nmatls > 0)  free (scene->matls);
}

void vert_Scene (Point* dst, const Scene* scene, uint idx)
{
    copy_Point (dst, &scene->verts[idx]);
}

void elem_Scene (Triangle* dst, const Scene* scene, uint idx)
{
    uint i;
    const SceneElement* elem;

    elem = &scene->elems[idx];

    UFor( i, NTrianglePoints )
        vert_Scene (&dst->pts[i], scene, elem->pts[i]);
}

