
#include "raytrace.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

void cleanup_RaySpace (RaySpace* space)
{
    cleanup_KDTree (&space->tree);
    if (space->nelems > 0)
    {
        free (space->elems);
        free (space->selems);
    }
}

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
hit_tri (real* mag,
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

    *mag = dot_Point (&edge2, &qvec) * inv_det;

        /* u *= inv_det; */
        /* v *= inv_det; */

    return true;
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

const Triangle* cast_ray (const Point* origin,
                          const Point* dir,
                          const KDTree* tree)
{
    Point salo_entrance;
    const KDTreeNode* parent = 0;

    const BoundingBox* box;
    Point* entrance;

    const Triangle* elem = 0;
    const KDTreeNode* node;

    entrance = &salo_entrance;

    if (! hit_BoundingBox (entrance, &tree->box, origin, dir))  return 0;
    box = &tree->box;
    node = &tree->root;

    while (node)
    {
        if (leaf_KDTreeNode (node))
        {
            uint i;
            const KDTreeLeaf* leaf;
            Point close_hit;

            leaf = &node->as.leaf;
            box = &leaf->box;
                /* output_BoundingBox (stdout, box); */
                /* fputc ('\n', stdout); */
            elem = 0;

            UFor( i, leaf->nelems )
            {
                Point hit;
                real mag;
                if (!hit_tri (&mag, origin, dir, leaf->elems[i]))  continue;

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
                if (elem)
                {
                    if (closer_hit (&hit, &close_hit, dir))
                    {
                        elem = leaf->elems[i];
                        set_Point (&close_hit, &hit);
                    }
                }
                else
                {
                    elem = leaf->elems[i];
                    set_Point (&close_hit, &hit);
                }
            }
            if (elem)  break;
            node = upnext_KDTreeNode (entrance, &parent, origin, dir, node);
        }
        else
        {
            const KDTreeInner* inner;
            inner = &node->as.inner;
            parent = node;

                /* Subtlety: Inclusive case here must be opposite of
                 * inclusive case in upnext_KDTreeNode to avoid infinite
                 * iteration on rays in the splitting plane's subspace.
                 */
            if (entrance->coords[node->split_dim] <= inner->split_pos)
                node = inner->children[0];
            else
                node = inner->children[1];
        }
    }
    return elem;
}

#if 0
void rays_to_hits_fish (uint* hits, uint nrows, uint ncols,
                        uint nelems, const Triangle* elems,
                        const KDTree* space,
                        real vert_angle, real horiz_angle)
{
    uint row;
    UFor( row, nrows )
    {
        uint col, offset;
        UFor( col, ncols )
        {
        }
    }
}
#endif

void rays_to_hits (uint* hits, uint nrows, uint ncols,
                   uint nelems, const Triangle* elems,
                   const KDTree* space)
{
    uint row;
    const uint dir_dim = 2;
    const uint row_dim = 1;
    const uint col_dim = 0;
    real col_start, row_start;
    real col_delta, row_delta;

    row_start = space->box.min_corner.coords[row_dim];
    row_delta = (space->box.max_corner.coords[row_dim] - row_start) / nrows;
    row_start += row_delta / 2;

    col_start = space->box.min_corner.coords[col_dim];
    col_delta = (space->box.max_corner.coords[col_dim] - col_start) / ncols;
    col_start += col_delta / 2;

    UFor( row, nrows )
    {
        uint col, offset;
        Point origin, dir;
        dir.coords[dir_dim] = 1;
        dir.coords[row_dim] = 0;
        dir.coords[col_dim] = 0;
        normalize_Point (&dir);

        origin.coords[dir_dim] = -1;
        origin.coords[row_dim] = row_start + (nrows - row -1) * row_delta;

        offset = row * ncols;
        UFor( col, ncols )
        {
            const Triangle* elem;
            origin.coords[col_dim] = col_start + col * col_delta;
            elem = cast_ray (&origin, &dir, space);
            if (elem)
                hits[offset + col] = index_of (elem, elems, sizeof (Triangle));
            else
                hits[offset + col] = nelems;
        }
    }
}
