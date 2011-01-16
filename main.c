
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define UFor( i, bel )  for (i = 0; i < bel; ++i)

#define NDimensions 3
#define MaxKDTreeDepth 3*NDimensions
    /* #define MaxKDTreeDepth 0 */

    /* typedef double real; */
typedef float real;
typedef unsigned uint;
typedef char tristate;
typedef unsigned char byte;

#ifndef COMPILER_HAS_BOOL
typedef byte bool;
#define true 1
#define false 0
#endif

struct point_struct
{
    real coords[NDimensions];
};
typedef struct point_struct Point;

#define NTrianglePoints 3
struct triangle_struct
{
    Point pts[NTrianglePoints];
};
typedef struct triangle_struct Triangle;

struct bounding_box_struct
{
    Point min_corner;
    Point max_corner;
};
typedef struct bounding_box_struct BoundingBox;

struct kd_tree_leaf_struct;
struct kd_tree_inner_struct;
struct kd_tree_node_struct;
typedef struct kd_tree_leaf_struct  KDTreeLeaf;
typedef struct kd_tree_inner_struct KDTreeInner;
typedef struct kd_tree_node_struct  KDTreeNode;

struct kd_tree_leaf_struct
{
    uint nelems;
    BoundingBox box;
    const Triangle** elems;
};
struct kd_tree_inner_struct
{
    real split_pos;
    KDTreeNode* parent;
    KDTreeNode* children[2];
};
struct kd_tree_node_struct
{
    uint split_dim;
    union kd_tree_node_struct_union
    {
        KDTreeLeaf leaf;
        KDTreeInner inner;
    } as;
};

struct kd_tree_struct
{
    BoundingBox box;
    KDTreeNode root;
};
typedef struct kd_tree_struct KDTree;


bool leaf_KDTreeNode (const KDTreeNode* node)
{
    return node->split_dim == NDimensions;
}

uint index_of (const void* e, const void* arr, size_t size)
{
    return ((size_t) e - (size_t) arr) / size;
}

void output_Point (FILE* out, const Point* point)
{
    uint ci;
    const char* delim = "";
    fputc ('(', out);
    UFor( ci, NDimensions )
    {
            /* fprintf (out, "%s%4.1f", delim, point->coords[ci]); */
        fprintf (out, "%s%.1f", delim, point->coords[ci]);
            /* delim = ", "; */
        delim = " ";
    }
    fputc (')', out);
}

void output_BoundingBox (FILE* out, const BoundingBox* box)
{
    fputs ("BoundingBox: ", out);
    output_Point (out, &box->min_corner);
    fputs (" to ", out);
    output_Point (out, &box->max_corner);
}

void output_Triangle (FILE* out, const Triangle* elem)
{
    uint pi;
    const char* delim = "";
    fputc ('[', out);
    UFor( pi, NTrianglePoints )
    {
        fputs (delim, out);
        output_Point (out, &elem->pts[pi]);
        delim = " ";
    }
    fputc (']', out);
}

    void
output_KDTreeNode (FILE* out, const KDTreeNode* node,
                   uint depth,
                   uint id,
                   const Triangle* elems)
{
    fprintf (out, " %*s%-*u depth:%u   ", depth, "", depth+1, id, depth);
    if (leaf_KDTreeNode (node))
    {
        uint ei;
        const KDTreeLeaf* leaf = &node->as.leaf;
        output_BoundingBox (out, &leaf->box);
        fprintf (out, "\n %*selems:", 2*depth+2, "");
        UFor( ei, leaf->nelems )
        {
            uint index;
            index = index_of (leaf->elems[ei], elems, sizeof (Triangle));
            fprintf (out, " %u", index);
        }
        fputc ('\n', out);

    }
    else
    {
        uint i;
        const KDTreeInner* inner;
        inner = &node->as.inner;
        fprintf (out, "dim:%u  pos:%.1f\n",
                 node->split_dim, inner->split_pos);
        UFor( i, 2 )
            output_KDTreeNode (out, inner->children[i],
                               1+depth,
                               2*(id+1)-1 + i,
                               elems);
    }
}

    void
output_KDTree (FILE* out, const KDTree* tree,
               uint nelems, const Triangle* elems)
{
    uint i;
    fputs ("-- KDTree --\n", out);
    fputs ("- Elements -", out);
    UFor( i, nelems )
    {
        fprintf (out, "\n %u  ", i);
        output_Triangle (out, &elems[i]);
    }
    fputs ("\n- Nodes -\n", out);
    output_KDTreeNode (out, &tree->root, 0, 0, elems);
}


    /* a - b */
void diff_Point (Point* dst, const Point* a, const Point* b)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = a->coords[i] - b->coords[i];
}

real dot_Point (const Point* a, const Point* b)
{
    uint i;
    real sum = 0;
    UFor( i, NDimensions )
        sum += a->coords[i] * b->coords[i];
    return sum;
}

    /* a + b */
void summ_Point (Point* dst, const Point* a, const Point* b)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = a->coords[i] + b->coords[i];
}

void scale_Point (Point* dst, const Point* a, real k)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = k * a->coords[i];
}

void zero_Point (Point* a)
{
    uint i;
    UFor( i, NDimensions )
        a->coords[i] = 0;
}

void set_Point (Point* dst, const Point* src)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = src->coords[i];
}

real magnitude_Point (const Point* a)
{
    return sqrt (dot_Point (a, a));
}

void normalize_Point (Point* a)
{
    scale_Point (a, a, 1 / magnitude_Point (a));
}

tristate compare_real (real a, real b)
{
    if (a > b)  return  1;
    if (a < b)  return -1;
    return 0;
}

tristate signum_real (real a)
{
    return compare_real (a, 0);
}

bool facing_plane (uint dim, real plane,
                   const Point* origin, const Point* dir)
{
    tristate sign;
    sign = compare_real (plane, origin->coords[dim]);
    sign *= signum_real (dir->coords[dim]);
    return sign >= 0;
}

bool hit_plane (Point* entrance,
                uint dim, real plane,
                const BoundingBox* box,
                const Point* origin, const Point* dir)
{
    uint i;
    real coeff;

    if (! facing_plane (dim, plane, origin, dir))  return false;

    coeff = (plane - origin->coords[dim]) / dir->coords[dim];
    
    UFor( i, NDimensions )
    {
        real x;
        if (i == dim)
        {
            x = plane;
        }
        else
        {
            x = origin->coords[i] + coeff * dir->coords[i];
            if (x < box->min_corner.coords[i])  return false;
            if (x > box->max_corner.coords[i])  return false;
        }
        entrance->coords[i] = x;
    }
    return true;
}

bool hit_box (Point* entrance,
              const BoundingBox* box,
              const Point* origin, const Point* dir)
{
    bool inside = true;
    uint dim;
    UFor( dim, NDimensions )
    {
        const Point* corner = 0;
        if (origin->coords[dim] < box->min_corner.coords[dim])
            corner = &box->min_corner;
        else if (origin->coords[dim] > box->max_corner.coords[dim])
            corner = &box->max_corner;

        if (corner)
        {
            if (hit_plane (entrance, dim, corner->coords[dim],
                           box, origin, dir))
                return true;
            inside = false;
        }
    }
    return inside;
}

#if 0
bool hit_tri (Point* hit,
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
    if (inbounds)     j
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


void cross_Point (Point* dst, const Point* a, const Point* b)
{
    dst->coords[0] = a->coords[1] * b->coords[2] - a->coords[2] * b->coords[1];
    dst->coords[1] = a->coords[2] * b->coords[0] - a->coords[0] * b->coords[2];
    dst->coords[2] = a->coords[0] * b->coords[1] - a->coords[1] * b->coords[0];
}


    /* code rewritten to do tests on the sign of the determinant */
    /* the division is before the test of the sign of the det    */
    /* and one CROSS has been moved out from the if-else if-else */
bool hit_tri (real* mag,
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


void adjust_BoundingBox (BoundingBox* box, const Point* point)
{
    uint i;
    UFor( i, NDimensions )
    {
        if (point->coords[i] < box->min_corner.coords[i])
            box->min_corner.coords[i] = point->coords[i];
        else if (point->coords[i] > box->max_corner.coords[i])
            box->max_corner.coords[i] = point->coords[i];
    }
}

bool inside_BoundingBox (const BoundingBox* box, const Point* point)
{
    uint i;
    UFor( i, NDimensions )
    {
        if (!(box->min_corner.coords[i] <= point->coords[i] &&
              box->max_corner.coords[i] >= point->coords[i]))
            return false;
    }
    return true;
}

    /* -1 strictly below, 0 in both, 1 strictly greater or equal */
tristate splitting_plane (const Triangle* elem,
                          uint split_dim, real split_pos)
{
    bool below = false;
    bool above = false;
    uint pi;
    UFor( pi, NTrianglePoints )
        if (elem->pts[pi].coords[split_dim] < split_pos)
            below = true;
        else
            above = true;
    if (below && above)  return 0;
    if (below)           return -1;
    if (above)           return 1;
    assert (0);
    return 0;
}

void partition_split (uint nelems, const Triangle** elems,
                      uint* nbelow, uint* nabove,
                      const KDTreeNode* node)
{
    uint ei, l, r;
    ei = 0;
    l = 0;
    r = nelems;

    while (ei < r)
    {
        tristate pos;
        pos = splitting_plane (elems[ei],
                               node->split_dim, node->as.inner.split_pos);
        if (pos < 0)
        {
            const Triangle* tmp;
            tmp = elems[l];
            elems[l] = elems[ei];
            elems[ei] = tmp;
            ++ l;
            ++ ei;
        }
        if (pos > 0)
        {
            const Triangle* tmp;
            -- r;
            tmp = elems[r];
            elems[r] = elems[ei];
            elems[ei] = tmp;
        }
        if (pos == 0)
        {
            ++ ei;
        }
    }
    *nbelow = r;
    *nabove = nelems - l;
}

void build_KDTreeNode (KDTreeNode* node,
                       KDTreeNode* parent, BoundingBox* box,
                       uint nelems, const Triangle** elems,
                       uint depth)
{
        /* printf ("%*sdepth=%u, nelems=%u\n", depth, "", depth, nelems); */
    if (depth < MaxKDTreeDepth)
    {
        uint nbelow, nabove;
        KDTreeNode** children;
        KDTreeInner* inner;
        inner = &node->as.inner;
        children = inner->children;
        node->split_dim = depth % NDimensions;
            /* node->split_dim = depth % 2; */
        inner->split_pos = 0.5 * (box->min_corner.coords[node->split_dim] +
                                  box->max_corner.coords[node->split_dim]);
        partition_split (nelems, elems, &nbelow, &nabove, node);

#if 0
        {
            uint i;
            FILE* out = stderr;
            fprintf (out, "%*sdim:%u pos:%.1f  blw:%u abv:%u\n",
                     2*depth, "",
                     node->split_dim, inner->split_pos,
                     nbelow, nabove);
            fprintf (out, "%*s", 2*depth+1, "");
            output_BoundingBox (out, box);
            UFor( i, nelems )
            {
                fprintf (out, "\n%*s", 2*depth+1, "");
                output_Triangle (out, elems[i]);
            }
            fputc ('\n', out);
        }
#endif

        children[0] = (KDTreeNode*) malloc (2 * sizeof (KDTreeNode));
        children[1] = &children[0][1];

        inner->parent = parent;

        {
            real tmp;
            const Triangle** blw_elems;

            blw_elems = (const Triangle**)
                malloc (nbelow * sizeof (Triangle*));
            memcpy (blw_elems, elems, nbelow * sizeof (Triangle*));

            tmp = box->max_corner.coords[node->split_dim];
            box->max_corner.coords[node->split_dim] = inner->split_pos;
            build_KDTreeNode (children[0], node, box, nbelow,
                              blw_elems, 1+depth);
            box->max_corner.coords[node->split_dim] = tmp;

            free (blw_elems);

            tmp = box->min_corner.coords[node->split_dim];
            box->min_corner.coords[node->split_dim] = inner->split_pos;
            build_KDTreeNode (children[1], node, box, nabove,
                              &elems[nelems-nabove], 1+depth);
            box->min_corner.coords[node->split_dim] = tmp;
        }

    }
    else
    {
        KDTreeLeaf* leaf;
        leaf = &node->as.leaf;
        node->split_dim = NDimensions;
        memcpy (&leaf->box, box, sizeof (BoundingBox));
        leaf->nelems = nelems;
        if (0 != nelems)
        {
            leaf->elems = (const Triangle**) malloc (nelems * sizeof (Triangle*));
            memcpy (leaf->elems, elems, nelems * sizeof (Triangle*));
        }
    }
}

void build_KDTree (KDTree* tree, uint nelems, const Triangle** elems)
{
    uint i, ei;
    assert (nelems > 0);
    UFor( i, NDimensions )
    {
        real x;
        x = elems[0]->pts[0].coords[i];
        tree->box.min_corner.coords[i] = x;
        tree->box.max_corner.coords[i] = x;
        tree->box.min_corner.coords[i] = 0;
        tree->box.max_corner.coords[i] = 100;
    }

    UFor( ei, nelems )
    {
        uint pi;
        UFor( pi, NTrianglePoints )
            adjust_BoundingBox (&tree->box, &elems[ei]->pts[pi]);
    }

    UFor( ei, nelems )
    {
        uint pi;
        UFor( pi, NTrianglePoints )
            assert (inside_BoundingBox (&tree->box, &elems[ei]->pts[pi]));
    }

    build_KDTreeNode (&tree->root, 0, &tree->box, nelems, elems, 0);
}

const KDTreeNode* upnext_KDTreeNode (Point* entrance,
                                     const KDTreeNode** parent_ptr,
                                     const Point* origin,
                                     const Point* dir,
                                     const KDTreeNode* node)
{
    const BoundingBox* box;
    const KDTreeNode* child;

    assert (leaf_KDTreeNode (node));
    box = &node->as.leaf.box;
    child = node;
    assert (parent_ptr);
    node = *parent_ptr;

    while (1)
    {
        const KDTreeInner* inner;
        tristate facing;
        uint ni;

            /* Backtracked from root node => no possible next leaf.*/
        if (!node)
        {
            child = 0;
            break;
        }

        assert (! leaf_KDTreeNode (node));
        inner = &node->as.inner;

        facing = signum_real (dir->coords[node->split_dim]);

            /* Subtlety: Inclusive case opposite when descending tree.*/
        ni = (facing < 0) ? 0 : 1;

            /* Ray pointing to previously visited node => keep backtracking.*/
        if (inner->children[ni] != child)
        {
            if (hit_plane (entrance, node->split_dim, inner->split_pos,
                           box, origin, dir))
            {
                *parent_ptr = node;
                child = inner->children[ni];
                break;
            }
        }

        child = node;
        node = inner->parent;
    }
    return child;
}

bool closer_hit (const Point* newhit, const Point* oldhit, const Point* dir)
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

    if (! hit_box (entrance, &tree->box, origin, dir))  return 0;
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


void cleanup_KDTreeNode (KDTreeNode* node)
{
    if (leaf_KDTreeNode (node))
    {
        KDTreeLeaf* leaf;
        leaf = &node->as.leaf;
        if (0 != leaf->nelems)
            free (leaf->elems);
    }
    else
    {
        KDTreeInner* inner;
        inner = &node->as.inner;
        cleanup_KDTreeNode (inner->children[0]);
        cleanup_KDTreeNode (inner->children[1]);
        free (inner->children[0]);
    }
}

void cleanup_KDTree (KDTree* tree)
{
    cleanup_KDTreeNode (&tree->root);
}

void random_Triangle (Triangle* elem)
{
    uint pi, ci;
    UFor( pi, NTrianglePoints )
    {
        UFor( ci, NDimensions )
        {
            real x;
            x = 100 * ((real) rand () / RAND_MAX);
                /* printf ("%f\n", x); */
            elem->pts[pi].coords[ci] = x;
        }
    }
}

void output_PBM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems)
{
    uint row, i;
    FILE* out;

    out = fopen (filename, "w+");
    fputs ("P1\n", out);
    fprintf (out, "%u %u\n", ncols, nrows);

    i = 0;
    UFor( row, nrows )
    {
        uint col;
        UFor( col, ncols )
        {
            if (nelems != hits[i++])
                fputs (" 1", out);
            else
                fputs (" 0", out);
        }
        fputc ('\n', out);
    }
    fclose (out);
}

void output_PGM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems)
{
    uint row, i;
    FILE* out;

    out = fopen (filename, "w+");
    fputs ("P2\n", out);
    fprintf (out, "%u %u\n", ncols, nrows);
    fprintf (out, "%u\n", nelems);

    i = 0;
    UFor( row, nrows )
    {
        uint col;
        UFor( col, ncols )
        {
            fprintf (out, " %u", nelems - hits[i++]);
        }
        fputc ('\n', out);
    }
    fclose (out);
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

int main ()
{
    uint i;
    KDTree tree;
    FILE* out;
#define NELEMS 20
    uint nelems   = NELEMS;
    Triangle selems[NELEMS];
    const Triangle* elems[NELEMS];
#undef NELEMS

    out = stdout;
    {
        unsigned seed = 1294785237;
            /* unsigned seed = 1294968341; */
            /* seed = time (0); */
        fprintf (out, "Using seed: %u\n", seed);
        srand (seed);
    }

    UFor( i, nelems )
    {
        uint pi, ci;
        Triangle* elem;
        elem = &selems[i];

        UFor( pi, NTrianglePoints )
        {
            UFor( ci, NDimensions )
            {
                elem->pts[pi].coords[ci] = 0;
            }
        }

        random_Triangle (elem);

        /*
        elem->pts[0].coords[0] = 10*i;
        elem->pts[0].coords[1] = 10;
        elem->pts[0].coords[2] = 0;

        elem->pts[1].coords[0] = 10*i;
        elem->pts[1].coords[1] = 0;
        elem->pts[1].coords[2] = 0;

        elem->pts[2].coords[0] = 10*i+10;
        elem->pts[2].coords[1] = 10;
        elem->pts[2].coords[2] = 0;
        */

        elems[i] = elem;
    }

    build_KDTree (&tree, nelems, elems);
        /* output_KDTree (stdout, &tree, nelems, selems); */
    {
        uint* hits;
        const uint nrows = 50;
        const uint ncols = 50;
        hits = (uint*) malloc (nrows * ncols * sizeof (uint));
        rays_to_hits (hits, nrows, ncols, nelems, selems, &tree);
        output_PBM_image ("out.pbm", nrows, ncols, hits, nelems);
        output_PGM_image ("out.pgm", nrows, ncols, hits, nelems);
        free (hits);
    }
#if 0
    {
        const Triangle* elem;
        Point origin;
        Point dir;
        UFor( i, NDimensions )
        {
            dir.coords[i] = tree.box.max_corner.coords[i];
            origin.coords[i] = - dir.coords[i];
                /* origin.coords[i] = 0; */
        }
        puts ("");
        elem = cast_ray (&origin, &dir, &tree);
        if (elem)
        {
            fputs ("Found element: ", out);
            output_Triangle (out, elem);
            fputc ('\n', out);
        }
        else
        {
            fputs ("No element found.\n", out);
        }
    }
#endif

    cleanup_KDTree (&tree);

    return 0;
}

