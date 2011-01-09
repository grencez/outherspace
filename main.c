
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define UFor( i, bel )  for (i = 0; i < bel; ++i)

#define NDimensions 3
#define MaxKDTreeDepth NDimensions

typedef float real;
typedef unsigned uint;
typedef char tristate;

#ifndef COMPILER_HAS_BOOL
typedef unsigned char bool;
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


void output_Point (FILE* out, const Point* point)
{
    uint ci;
    const char* delim = "";
    fputc ('(', out);
    UFor( ci, NDimensions )
    {
        fprintf (out, "%s%4.1f", delim, point->coords[ci]);
        delim = ", ";
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

void output_KDTreeNode (FILE* out, const KDTreeNode* node, uint depth)
{
    fprintf (out, "%*sdepth=%u", 2*depth, "", depth);
    if (leaf_KDTreeNode (node))
    {
        uint ei;
        fputs ("  elems:", out);
        UFor( ei, node->as.leaf.nelems )
            output_Triangle (out, node->as.leaf.elems[ei]);
        fputs ("\n", out);

    }
    else
    {
        uint i;
        const KDTreeInner* inner;
        inner = &node->as.inner;
        fprintf (out, " split_dim=%u split_pos=%f\n",
                 node->split_dim, inner->split_pos);
        UFor( i, 2 )
            output_KDTreeNode (out, inner->children[i], 1+depth);
    }
}

void output_KDTree (FILE* out, const KDTree* tree)
{
    output_KDTreeNode (out, &tree->root, 0);
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

bool hit_tri (const Point* origin, const Point* dir,
              const Triangle* elem)
{
    uint i, j, k;
    Point v[NTrianglePoints];
    real tdots[NTrianglePoints];
    real dirdot;

    dirdot = dot_Point (dir, dir);
    UFor( i, NTrianglePoints )
        diff_Point (&v[i], &elem->pts[i], origin);
    UFor( i, NTrianglePoints )
    {
        j = (1+i) % NTrianglePoints;
        k = (2+i) % NTrianglePoints;
        tdots[i] = dirdot * dot_Point (&v[j], &v[k])
            - dot_Point (dir, &v[j]) * dot_Point (dir, &v[k]);
    }
    UFor( i, NTrianglePoints )
    {
        j = (1+i) % NTrianglePoints;
        k = (2+i) % NTrianglePoints;
        if (tdots[j] <= 0 && tdots[k] <= 0)
        {
            tristate sign;
            real x;
            x = dot_Point (dir, &v[i]);
            x = dirdot * dot_Point (&v[i], &v[i]) - x * x;
            sign = compare_real (x * tdots[i], tdots[j] * tdots[k]);
            return sign <= 0;
        }
    }
    return false;
}

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
        inner->split_pos = 0.5 * (box->min_corner.coords[node->split_dim] +
                                  box->max_corner.coords[node->split_dim]);
            /* printf ("%*ssplitting: %f\n", depth, "", node->split_pos); */
            /* printf ("%*sbox: x=%f y=%f z=%f\n", depth, "", box->lengths.coords[0], box->lengths.coords[1], box->lengths.coords[2]); */
        partition_split (nelems, elems, &nbelow, &nabove, node);
        children[0] = (KDTreeNode*) malloc (2 * sizeof (KDTreeNode));
        children[1] = &children[0][1];

        inner->parent = parent;

        {
            real tmp;
            tmp = box->max_corner.coords[node->split_dim];
            box->max_corner.coords[node->split_dim] = inner->split_pos;
            build_KDTreeNode (children[0], node, box, nbelow,
                              elems, 1+depth);
            box->max_corner.coords[node->split_dim] = tmp;

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
        leaf->nelems = nelems;
        leaf->elems = elems;
        memcpy (&leaf->box, box, sizeof (BoundingBox));
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

const Triangle* cast_ray_iter (const Point* origin,
                               const Point* dir,
                               const KDTree* tree)
{
    Point salo_entrance;
    const KDTreeNode* parent;

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
            leaf = &node->as.leaf;
            box = &leaf->box;
            output_BoundingBox (stdout, box);
            fputc ('\n', stdout);
            elem = 0;
            UFor( i, leaf->nelems )
            {
                if (hit_tri (origin, dir, leaf->elems[i]))
                {
                    elem = leaf->elems[i];
                    break;
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

const Triangle* cast_ray_rec (const Point* origin,
                              const Point* entrance,
                              const Point* dir,
                              const KDTreeNode* node,
                              const BoundingBox* box)
{
    const Triangle* elem;
    if (leaf_KDTreeNode (node))
    {
            /* TODO: Intersection tests. */
        elem = 0;
        output_BoundingBox (stdout, box);
        fputc ('\n', stdout);
    }
    else
    {
        uint child;
        BoundingBox tbox;
        const KDTreeInner* inner;
        Point desc_entrance;
        inner = &node->as.inner;
        memcpy (&tbox, box, sizeof (BoundingBox));
        if (entrance->coords[node->split_dim] <= inner->split_pos)
        {
            child = 0;
            tbox.max_corner.coords[node->split_dim] = inner->split_pos;
        }
        else
        {
            child = 1;
            tbox.min_corner.coords[node->split_dim] = inner->split_pos;
        }
        elem = cast_ray_rec (origin, entrance, dir,
                             inner->children[child], &tbox);
        if (elem)  return elem;
        if (hit_plane (&desc_entrance, node->split_dim, inner->split_pos,
                       box, origin, dir))
        {
                /* fputs ("HIT\n", stdout); */
            child = (1+child) % 2;
            if (child == 0)
            {
                tbox.max_corner.coords[node->split_dim] = inner->split_pos;
                tbox.min_corner.coords[node->split_dim] =
                    box->min_corner.coords[node->split_dim];
            }
            else
            {
                tbox.max_corner.coords[node->split_dim] =
                    box->max_corner.coords[node->split_dim];
                tbox.min_corner.coords[node->split_dim] = inner->split_pos;
            }
            elem = cast_ray_rec (origin, &desc_entrance, dir,
                                 inner->children[child], &tbox);
        }
        else
        {
                /* fputs ("NOHIT\n", stdout); */
        }
    }
    return elem;
}

const Triangle* cast_ray (const Point* origin,
                          const Point* dir,
                          const KDTree* tree)
{
    Point entrance;
    if (! hit_box (&entrance, &tree->box, origin, dir))  return 0;
    return cast_ray_rec (origin, &entrance, dir, &tree->root, &tree->box);
}


void cleanup_KDTreeNode (KDTreeNode* node)
{
    if (! leaf_KDTreeNode (node))
    {
        KDTreeNode** children;
        children = node->as.inner.children;
        cleanup_KDTreeNode (children[0]);
        cleanup_KDTreeNode (children[1]);
        free (children[0]);
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

int main ()
{
    uint i;
    KDTree tree;
    FILE* out;
#define NELEMS 10
    uint nelems   = NELEMS;
    Triangle selems[NELEMS];
    const Triangle* elems[NELEMS];
#undef NELEMS

    out = stdout;
    srand (time (0));

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

        elem->pts[0].coords[0] = 10*i;
        elem->pts[0].coords[1] = 10;
        elem->pts[0].coords[2] = 10;
        elem->pts[1].coords[0] = 10*i;
        elem->pts[1].coords[1] = 0;
        elem->pts[1].coords[2] = 0;
        elem->pts[2].coords[0] = 10*i+10;
        elem->pts[2].coords[1] = 10;
        elem->pts[2].coords[2] = 0;

        random_Triangle (elem);
        elems[i] = elem;
    }

    build_KDTree (&tree, nelems, elems);
        /* output_KDTree (stdout, &tree); */
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
        cast_ray (&origin, &dir, &tree);
        puts ("");
        elem = cast_ray_iter (&origin, &dir, &tree);
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

    cleanup_KDTree (&tree);
    return 0;
}

