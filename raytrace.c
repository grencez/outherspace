
#include "raytrace.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <omp.h>

void cleanup_RaySpace (RaySpace* space)
{
    cleanup_Scene (&space->scene);
    cleanup_KDTree (&space->tree);
    if (space->nelems > 0)
    {
        free (space->elems);
    }
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if 0
static
    bool
hit_tri (Point* hit,
         const Point* origin, const Point* dir,
         const Triangle* elem)
{
    uint i, j, k;
    Triangle t;
    real trdots[NTrianglePoints];
    real tdots[NTrianglePoints];
    real dirdot;
    bool inbounds = false;

    dirdot = dot_Point (dir, dir);
    UFor( i, NTrianglePoints )
    {
        diff_Point (&t.pts[i], &elem->pts[i], origin);
        trdots[i] = dot_Point (dir, &t.pts[i]);
    }
    UFor( i, NTrianglePoints )
    {
        j = (1+i) % NTrianglePoints;
        k = (2+i) % NTrianglePoints;
        tdots[i] = dirdot * dot_Point (&t.pts[j], &t.pts[k])
            - trdots[j] * trdots[k];
    }
    UFor( i, NTrianglePoints )
    {
        j = (1+i) % NTrianglePoints;
        k = (2+i) % NTrianglePoints;
        if (tdots[j] <= 0 && tdots[k] <= 0)
        {
            tristate sign;
            real x;
            x = dirdot * dot_Point (&t.pts[i], &t.pts[i])
                - trdots[i] * trdots[i];
            sign = compare_real (x * tdots[i], tdots[j] * tdots[k]);
            inbounds = sign <= 0;
            break;
        }
    }
    if (inbounds)
    {
        zero_Point (hit);
        UFor( i, NTrianglePoints )
        {
            Point p;
            scale_Point (&p, dir, trdots[i]);
            summ_Point (hit, hit, &p);
        }

        UFor( i, NDimensions )
        {
            if (signum_real (hit->coords[i]) != signum_real (dir->coords[i]))
            {
                inbounds = false;
                break;
            }
        }

        if (inbounds)
        {
            scale_Point (hit, hit, 1 / (3 * dirdot));
            summ_Point (hit, hit, origin);
        }
    }
    return inbounds;
}
#else


static
    void
cross_Point (Point* dst, const Point* a, const Point* b)
{
    dst->coords[0] = a->coords[1] * b->coords[2] - a->coords[2] * b->coords[1];
    dst->coords[1] = a->coords[2] * b->coords[0] - a->coords[0] * b->coords[2];
    dst->coords[2] = a->coords[0] * b->coords[1] - a->coords[1] * b->coords[0];
}


    /* code rewritten to do tests on the sign of the determinant */
    /* the division is before the test of the sign of the det    */
    /* and one CROSS has been moved out from the if-else if-else */
static
    bool
hit_tri (real* dist,
         const Point* origin, const Point* dir,
         const Triangle* elem)
{
    const real epsilon = 0.000001;
    Point edge1, edge2, tvec, pvec, qvec;
    real det, inv_det;
    real u, v;

        /* find vectors for two edges sharing vert0 */
    diff_Point (&edge1, &elem->pts[1], &elem->pts[0]);
    diff_Point (&edge2, &elem->pts[2], &elem->pts[0]);

        /* begin calculating determinant - also used to calculate U parameter */
    cross_Point (&pvec, dir, &edge2);

        /* if determinant is near zero, ray lies in plane of triangle */
    det = dot_Point (&edge1, &pvec);

        /* calculate distance from vert0 to ray origin */
    diff_Point (&tvec, origin, &elem->pts[0]);
    inv_det = 1 / det;

    cross_Point (&qvec, &tvec, &edge1);

    if (det > epsilon)
    {
        u = dot_Point (&tvec, &pvec);
        if (u < 0.0 || u > det)
            return false;

            /* calculate V parameter and test bounds */
        v = dot_Point (dir, &qvec);
        if (v < 0.0 || u + v > det)
            return false;

    }
    else if (det < -epsilon)
    {
            /* calculate U parameter and test bounds */
        u = dot_Point (&tvec, &pvec);
        if (u > 0.0 || u < det)
            return false;

            /* calculate V parameter and test bounds */
        v = dot_Point (dir, &qvec);
        if (v > 0.0 || u + v < det)
            return false;
    }
    else return false;  /* ray is parallel to the plane of the triangle */

    *dist = dot_Point (&edge2, &qvec) * inv_det;

        /* u *= inv_det; */
        /* v *= inv_det; */

        /* Return false when the ray is behind the origin. */
    return *dist >= 0;
}
#endif


static
    bool
closer_hit (const Point* newhit, const Point* oldhit, const Point* dir)
{
    uint i;
    UFor( i, NDimensions )
    {
        tristate sign;
        sign = signum_real (dir->coords[i]);
        if (sign != 0)
            return sign == compare_real (oldhit->coords[i],
                                         newhit->coords[i]);
    }
    return false;
}

uint cast_ray (const Point* origin,
               const Point* dir,
               const RaySpace* space,
               bool inside_box)
{
    Point salo_entrance;
    uint node_idx, parent = 0;

    const BoundingBox* box;
    Point* entrance;
    const KDTree* tree;
    uint elemIdx;

    entrance = &salo_entrance;
    tree = &space->tree;
    elemIdx = space->scene.nelems;

    if (inside_box)
    {
            /* Find the initial node.*/
        node_idx = find_KDTreeNode (&parent, origin, tree);
        box = &tree->nodes[node_idx].as.leaf.box;
        assert (inside_BoundingBox (box, origin));
    }
    else
    {
        box = &space->scene.box;
        if (! hit_outer_BoundingBox (entrance, box, origin, dir))
            return space->scene.nelems;
        node_idx = 0;
    }

    while (1)
    {
        const KDTreeNode* node;
        node = &tree->nodes[node_idx];

        if (leaf_KDTreeNode (node))
        {
            uint i;
            const KDTreeLeaf* leaf;
            Point close_hit;

            leaf = &node->as.leaf;
            box = &leaf->box;
                /* output_BoundingBox (stdout, box); */
                /* fputc ('\n', stdout); */

            UFor( i, leaf->nelems )
            {
                Point hit;
                real mag;
                const Triangle* tri;
                tri = &space->elems[leaf->elems[i]];
                    /* Triangle tri; */
                    /* elem_Scene (&tri, &space->scene, leaf->elems[i]); */

                if (!hit_tri (&mag, origin, dir, tri))  continue;

                scale_Point (&hit, dir, mag);
                summ_Point (&hit, &hit, origin);

                if (!inside_BoundingBox (box, &hit))  continue;
#if 0
                if (!inside_BoundingBox (box, &hit))
                {
                    output_BoundingBox (stderr, box);
                    fputs ("\n", stderr);
                    output_Point (stderr, origin);
                    fputs (" => ", stderr);
                    output_Point (stderr, &hit);
                    fputs ("\n", stderr);
                }
#endif
                if (elemIdx != space->scene.nelems)
                {
                    if (closer_hit (&hit, &close_hit, dir))
                    {
                        elemIdx = leaf->elems[i];
                        copy_Point (&close_hit, &hit);
                    }
                }
                else
                {
                    elemIdx = leaf->elems[i];
                    copy_Point (&close_hit, &hit);
                }
            }
            if (elemIdx != space->scene.nelems)  break;

            node_idx = upnext_KDTreeNode (entrance, &parent,
                                          origin, dir, node_idx, tree->nodes);
            if (node_idx == parent)  break;
        }
        else
        {
            const KDTreeInner* inner;
            inner = &node->as.inner;
            parent = node_idx;

                /* Subtlety: Inclusive case here must be opposite of
                 * inclusive case in upnext_KDTreeNode to avoid infinite
                 * iteration on rays in the splitting plane's subspace.
                 */
            if (entrance->coords[node->split_dim] <= inner->split_pos)
                node_idx = inner->children[0];
            else
                node_idx = inner->children[1];
        }
    }
    return elemIdx;
}

void rays_to_hits_fish (uint* hits, uint nrows, uint ncols,
                        const RaySpace* space, real zpos)
{
    uint row;
    bool inside_box;
    const uint dir_dim = 2;
    const uint row_dim = 1;
    const uint col_dim = 0;
    Point origin, tdir;
    real col_start, row_start;
    real col_delta, row_delta;

    row_start = - M_PI / 3;
    row_delta = 2 * M_PI / (3 * nrows);
    row_start += row_delta / 2;

    col_start = - M_PI / 3;
    col_delta = 2 * M_PI / (3 * ncols);
    col_start += col_delta / 2;

    origin.coords[dir_dim] = zpos;
    origin.coords[row_dim] = 50;
    origin.coords[col_dim] = 50;
    
    tdir.coords[dir_dim] = 1;
    tdir.coords[row_dim] = 0;
    tdir.coords[col_dim] = 0;

    inside_box = inside_BoundingBox (&space->scene.box, &origin);

#pragma omp parallel for
    UFor( row, nrows )
    {
        uint col;
        real row_angle;
        uint* hitline;

        hitline = &hits[row * ncols];

        row_angle = row_start + row_delta * (nrows - row -1);

        UFor( col, ncols )
        {
            Point dir;
            real col_angle;
            col_angle = col_start + col_delta * col;

            dir.coords[row_dim] = (tdir.coords[row_dim] * (1 + cos (row_angle))
                                   + tdir.coords[dir_dim] * sin (row_angle));

            dir.coords[col_dim] = (tdir.coords[col_dim] * (1 + cos (col_angle))
                                   + tdir.coords[dir_dim] * sin (col_angle));

            dir.coords[dir_dim] = (tdir.coords[dir_dim] * (cos (row_angle) + cos (col_angle))
                                   - tdir.coords[row_dim] * sin (row_angle)
                                   - tdir.coords[col_dim] * sin (col_angle));

            normalize_Point (&dir, &dir);
            hitline[col] = cast_ray (&origin, &dir, space, inside_box);
        }
    }
}

void rays_to_hits_perspective (uint* hits, uint nrows, uint ncols,
                               const RaySpace* space, real zpos)
{
    uint row;
    bool inside_box;
    const uint dir_dim = 2;
    const uint row_dim = 1;
    const uint col_dim = 0;
    Point origin, tdir;
    real col_start, row_start;
    real col_delta, row_delta;
    const BoundingBox* box;

    box = &space->scene.box;

    row_start = space->scene.box.min_corner.coords[row_dim];
    row_delta = (box->max_corner.coords[row_dim] - row_start) / nrows;
    row_start += row_delta / 2;

    col_start = box->min_corner.coords[col_dim];
    col_delta = (box->max_corner.coords[col_dim] - col_start) / ncols;
    col_start += col_delta / 2;

    inside_box = (zpos > box->min_corner.coords[dir_dim] &&
                  zpos < box->max_corner.coords[dir_dim]);

    origin.coords[dir_dim] = zpos;
    origin.coords[row_dim] = 50;
    origin.coords[col_dim] = 50;
    
    tdir.coords[dir_dim] = 1;
    tdir.coords[row_dim] = 0;
    tdir.coords[col_dim] = 0;

    inside_box = inside_BoundingBox (box, &origin);

#pragma omp parallel for
    UFor( row, nrows )
    {
        uint col;
        uint* hitline;

        hitline = &hits[row * ncols];

        UFor( col, ncols )
        {
            Point dir;

                /* if (! (row == 333 && col == 322))  continue; */

            dir.coords[dir_dim] = 0;
            dir.coords[row_dim] = row_start + (nrows - row -1) * row_delta;
            dir.coords[col_dim] = col_start + col * col_delta;

            diff_Point (&dir, &dir, &origin);
            normalize_Point (&dir, &dir);

            hitline[col] = cast_ray (&origin, &dir, space, inside_box);

                /* if (row == 333 && col == 322)  puts (elem ? "hit" : "miss"); */
        }
    }
}

void rays_to_hits_plane (uint* hits, uint nrows, uint ncols,
                         const RaySpace* space, real zpos)
{
    uint row;
    bool inside_box;
    const uint dir_dim = 2;
    const uint row_dim = 1;
    const uint col_dim = 0;
    real col_start, row_start;
    real col_delta, row_delta;
    const BoundingBox* box;

    box = &space->scene.box;

    row_start = box->min_corner.coords[row_dim];
    row_delta = (box->max_corner.coords[row_dim] - row_start) / nrows;
    row_start += row_delta / 2;

    col_start = box->min_corner.coords[col_dim];
    col_delta = (box->max_corner.coords[col_dim] - col_start) / ncols;
    col_start += col_delta / 2;

    inside_box = (zpos > box->min_corner.coords[dir_dim] &&
                  zpos < box->max_corner.coords[dir_dim]);

#pragma omp parallel for
    UFor( row, nrows )
    {
        uint col;
        Point origin, dir;
        uint* hitline;

        hitline = &hits[row * ncols];

        dir.coords[dir_dim] = 1;
        dir.coords[row_dim] = 0;
        dir.coords[col_dim] = 0;
        normalize_Point (&dir, &dir);

        origin.coords[dir_dim] = zpos;
        origin.coords[row_dim] = row_start + (nrows - row -1) * row_delta;

        UFor( col, ncols )
        {
            origin.coords[col_dim] = col_start + col * col_delta;
            hitline[col] = cast_ray (&origin, &dir, space, inside_box);
        }
    }
}

void rays_to_hits (uint* hits, uint nrows, uint ncols,
                   const RaySpace* space,
                   const Point* origin,
                   const PointXfrm* view_basis)
{
    uint row;
    bool inside_box;
    Point dir_start, row_delta, col_delta;
    const uint dir_dim = 2, row_dim = 1, col_dim = 0;
    const BoundingBox* box;

    box = &space->scene.box;

    {
        Point dstart, rdelta, cdelta;
        real tcos;
        tcos = cos (M_PI / 3); /* cos (view_angle / 2) */

        zero_Point (&dstart);
        zero_Point (&rdelta);
        zero_Point (&cdelta);

        dstart.coords[dir_dim] = 1;
        dstart.coords[row_dim] = - tcos;
        dstart.coords[col_dim] = - tcos;

        rdelta.coords[row_dim] = -2 * dstart.coords[row_dim] / nrows;
        cdelta.coords[col_dim] = -2 * dstart.coords[col_dim] / ncols;

        dstart.coords[row_dim] -= dstart.coords[row_dim] / nrows;
        dstart.coords[col_dim] -= dstart.coords[col_dim] / ncols;

        trxfrm_Point (&dir_start, view_basis, &dstart);
        trxfrm_Point (&row_delta, view_basis, &rdelta);
        trxfrm_Point (&col_delta, view_basis, &cdelta);
    }

    inside_box = inside_BoundingBox (box, origin);

#pragma omp parallel for
    UFor( row, nrows )
    {
        uint col;
        uint* hitline;
        Point partial_dir;

        hitline = &hits[row * ncols];
        scale_Point (&partial_dir, &row_delta, nrows - row -1);
        summ_Point (&partial_dir, &partial_dir, &dir_start);

        UFor( col, ncols )
        {
            Point dir;

                /* if (! (row == 333 && col == 322))  continue; */

            scale_Point (&dir, &col_delta, col);
            summ_Point (&dir, &dir, &partial_dir);
            normalize_Point (&dir, &dir);

            hitline[col] = cast_ray (origin, &dir, space, inside_box);

                /* if (row == 333 && col == 322)  puts (elem ? "hit" : "miss"); */
        }
    }
}

