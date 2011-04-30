
#include "scene.h"

void init_Scene (Scene* scene)
{
    scene->nelems = 0;
    scene->nverts = 0;
    scene->ntxpts = 0;
    scene->nmatls = 0;
    scene->ntxtrs = 0;
}

void init_SceneElement (SceneElement* elem)
{
    uint i;
    elem->material = Max_uint;
    UFor( i, NDimensions-1 )
        elem->txpts[i] = Max_uint;
}

void copy_SceneElement (SceneElement* dst, const SceneElement* src)
{
    memcpy (dst, src, sizeof (SceneElement));
}

void cleanup_Scene (Scene* scene)
{
    if (scene->nelems > 0)  free (scene->elems);
    if (scene->nverts > 0)  free (scene->verts);
    if (scene->ntxpts > 0)  free (scene->txpts);
    if (scene->nmatls > 0)  free (scene->matls);
    if (scene->ntxtrs > 0)
    {
        uint i;
        UFor( i, scene->ntxtrs )
            free (scene->txtrs[i].pixels);
        free (scene->txtrs);
    }
}

void vert_Scene (Point* dst, const Scene* scene, uint idx)
{
    copy_Point (dst, &scene->verts[idx]);
}

    void
tri_Scene (Triangle* dst, const Scene* scene, uint idx)
{
    uint i;
    const SceneElement* elem;

    elem = &scene->elems[idx];

    UFor( i, NTrianglePoints )
        vert_Scene (&dst->pts[i], scene, elem->pts[i]);
}

    void
elem_Scene (PointXfrm* dst, const Scene* scene, uint idx)
{
    uint i;
    const SceneElement* elem;

    elem = &scene->elems[idx];

    UFor( i, NDimensions )
        vert_Scene (&dst->pts[i], scene, elem->pts[i]);
}

