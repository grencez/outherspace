
#include "pack.h"

#define RayPacketDimSz realPackSz
#define RayPacketSz (RayPacketDimSz*RayPacketDimSz)

struct ray_packet_struct
{
    PointPack origins[RayPacketDimSz];
    PointPack directs[RayPacketDimSz];
};
typedef struct ray_packet_struct RayPacket;

struct ray_hit_packet_struct
{
    uintPack inds[RayPacketDimSz];
    uintPack objs[RayPacketDimSz];
    realPack mags[RayPacketDimSz];
};
typedef struct ray_hit_packet_struct RayHitPacket;


static
    void
first_pack_KDTreeNode (uintPack* ret_nodes,
                       uintPack* ret_parents,
                       const PointPack* origins,
                       const PointPack* directs,
                       const KDTreeNode* nodes,
                       const BoundingBox* box,
                       const boolPack* inside_box_flags)
{
    uint i;
    UFor( i, RayPacketDimSz )
    {
        uint j;
            /* Inputs.*/
        bool inside_box[RayPacketDimSz];
        Point origin[RayPacketDimSz];
        Point direct[RayPacketDimSz];
            /* Outputs.*/
        uint node[RayPacketDimSz];
        uint parent[RayPacketDimSz];

        scat_boolPack (inside_box, inside_box_flags[i]);
        scat_PointPack (origin, &origins[i]);
        scat_PointPack (direct, &directs[i]);

        UFor( j, RayPacketDimSz )
            node[j] = first_KDTreeNode (&parent[j],
                                        &origin[j], &direct[j],
                                        nodes, box, inside_box[j]);

        ret_nodes[i] = pack_uintPack (node);
        ret_parents[i] = pack_uintPack (parent);
    }
}

static
    bool
contains_uint (uint n, const uint* a, uint x)
{
    uint i;
    UFor( i, n )
        if (a[i] == x)  return true;
    return false;
}

static
    void
next_pack_KDTreeNode (uintPack* ret_nodes,
                      uintPack* ret_parents,
                      boolPack* hit_in_boxen,
                      const PointPack* origins,
                      const PointPack* directs,
                      const PointPack* rdirects,
                      const realPack* hit_mags,
                      const KDTreeNode* nodes,
                      uint nprev_nodes,
                      const uint* prev_nodes)
{
    uint i;
    UFor( i, RayPacketDimSz )
    {
        bool hit_in_box[RayPacketDimSz];
        uint node_idx[RayPacketDimSz];
        uint parent_idx[RayPacketDimSz];
        real hit_mag[RayPacketDimSz];
        uint j;

        scat_boolPack (hit_in_box, hit_in_boxen[i]);
        scat_uintPack (node_idx, ret_nodes[i]);
        scat_uintPack (parent_idx, ret_parents[i]);
        scat_realPack (hit_mag, hit_mags[i]);

        UFor( j, RayPacketDimSz )
        {
            uint dim;
            Point origin, direct, rdirect;

            if (hit_in_box[j])  continue;

            UFor( dim, NDimensions )
            {
                origin.coords[dim] = origins[i].vcoords[dim][j];
                direct.coords[dim] = directs[i].vcoords[dim][j];
                rdirect.coords[dim] = rdirects[i].vcoords[dim][j];
            }

            do
            {
                node_idx[j] = next_KDTreeNode (&parent_idx[j],
                                               &origin, &direct, &rdirect,
                                               hit_mag[j],
                                               node_idx[j], nodes);
            } while (node_idx[j] != Max_uint &&
                     contains_uint (nprev_nodes, prev_nodes, node_idx[j]));

                /* If exiting search space, flag it!*/
            if (node_idx[j] == Max_uint)
                hit_in_box[j] = true;
        }

        ret_nodes[i] = pack_uintPack (node_idx);
        ret_parents[i] = pack_uintPack (parent_idx);
        hit_in_boxen[i] = pack_boolPack (hit_in_box);
    }
}

static
    uint
unique_pack_uints (uint* a)
{
    uint i, n = 0;
    UFor( i, RayPacketDimSz )
    {
        uint j;
        bool match = false;
        if (a[i] == Max_uint)  continue;
        UFor( j, n )
        {
            if (a[i] == a[j])
                match = true;
        }
        if (!match)  a[n++] = a[i];
    }
    return n;
}

static
    void
unique_node_pack_indices (uint* ret_nnodes, uint* ret_nodes,
                          const uintPack* node_indices)
{
    uint i, n = 0;
    UFor( i, RayPacketDimSz )
    {
        uint x[RayPacketDimSz];
        uint j;
        uintPack node_idx;
        boolPack match;

        node_idx = node_indices[i];

        match = false_boolPack ();
        UFor( j, n )
        {
            boolPack tmp_match;
            tmp_match = eql1_uintPack (node_idx, ret_nodes[j]);
            match = or_boolPack (match, tmp_match);
        }

        cfill_uintPack (&node_idx, Max_uint, match);
        scat_uintPack (x, node_idx);
        j = unique_pack_uints (x);
        Op_0( uint, j, &ret_nodes[n] , x );
        n += j;
    }

    *ret_nnodes = n;
}


static
    void
cast_RayPacket (RayHitPacket* hit, const RayPacket* pkt,
                const ObjectRaySpace* space,
                const boolPack* inside_box_flags)
{
    PointPack rdirects[RayPacketDimSz];
    boolPack hit_in_boxen[RayPacketDimSz];
    uintPack node_indices[RayPacketDimSz];
    uintPack parent_indices[RayPacketDimSz];
    uint explore_nodes[RayPacketSz];
    uint nexplore_nodes;
    uint i;

    first_pack_KDTreeNode (node_indices,
                           parent_indices,
                           pkt->origins,
                           pkt->directs,
                           space->tree.nodes,
                           &space->box,
                           inside_box_flags);

    UFor( i, RayPacketDimSz )
    {
        hit_in_boxen[i] = eql1_uintPack (node_indices[i], Max_uint);
        reci_PointPack (&rdirects[i], &pkt->directs[i]);
    }

    unique_node_pack_indices (&nexplore_nodes, explore_nodes,
                              node_indices);

    while (nexplore_nodes > 0)
    {
        uint j;

        UFor( j, nexplore_nodes )
        {
            const KDTreeLeaf* restrict leaf;
            const uint* restrict elemidcs;

            leaf = &space->tree.nodes[explore_nodes[j]].as.leaf;
            elemidcs = &space->tree.elemidcs[leaf->elemidcs];

            UFor( i, RayPacketDimSz )
            {
                test_pack_intersections (&hit->inds[i],
                                         &hit->mags[i],
                                         &pkt->origins[i],
                                         &pkt->directs[i],
                                         leaf->nelems, elemidcs,
                                         space->simplices);
            }
        }

        next_pack_KDTreeNode (node_indices,
                              parent_indices,
                              hit_in_boxen,
                              pkt->origins,
                              pkt->directs,
                              rdirects,
                              hit->mags,
                              space->tree.nodes,
                              nexplore_nodes,
                              explore_nodes);

        unique_node_pack_indices (&nexplore_nodes, explore_nodes,
                                  node_indices);
    }
}

static
    void
ray_packet_to_basis (RayPacket* ret_pkt,
                     const PointXfrm* basis,
                     const RayPacket* pkt,
                     const Point* old_centroid)
{
    uint i;
    UFor( i, RayPacketDimSz )
    {
        Point origin[RayPacketDimSz], direct[RayPacketDimSz];
        uint j;

        scat_PointPack (origin, &pkt->origins[i]);
        scat_PointPack (direct, &pkt->directs[i]);

        UFor( j, RayPacketDimSz )
            ray_to_basis (&origin[j], &direct[j], basis,
                          &origin[j], &direct[j], old_centroid);

        pack_PointPack (&ret_pkt->origins[i], origin);
        pack_PointPack (&ret_pkt->directs[i], direct);
    }
}

static
    const ObjectRaySpace*
ray_packet_to_ObjectRaySpace (RayPacket* ret_pkt,
                              const RayPacket* pkt,
                              const RaySpace* space,
                              uint objidx)
{
    const ObjectRaySpace* object;
    assert (objidx < space->nobjects);

    object = &space->objects[objidx];
    ray_packet_to_basis (ret_pkt, &object->orientation,
                         pkt, &object->centroid);

    return object;
}

static
    void
cast_packet_nopartition (RayHitPacket* hit,
                         const RayPacket* pkt,
                         const RaySpace* restrict space,
                         const boolPack* inside_box,
                         uint ignore_object)
{
    uint i, objidx;
    realPack prev_mags[RayPacketDimSz];

    if (space->main.visible)
        cast_RayPacket (hit, pkt, &space->main, inside_box);

    UFor( i, RayPacketDimSz )
    {
        boolPack didhit;
        prev_mags[i] = hit->mags[i];
        didhit = less_realPack (hit->mags[i], fill_realPack (Max_real));
        cfill_uintPack (&hit->objs[i], space->nobjects, didhit);
    }

    UFor( objidx, space->nobjects )
    {
        RayPacket rel_pkt;
        boolPack rel_inside_box[RayPacketDimSz];
        const ObjectRaySpace* object;

        if (objidx == ignore_object || !space->objects[objidx].visible)
            continue;

        object = ray_packet_to_ObjectRaySpace (&rel_pkt, pkt, space, objidx);

        UFor( i, RayPacketDimSz )
        {
            rel_inside_box[i] =
                inside_pack_BoundingBox (&object->box, &rel_pkt.origins[i]);
        }

        cast_RayPacket (hit, &rel_pkt, object, rel_inside_box);

        UFor( i, RayPacketDimSz )
        {
            boolPack didhit;
            didhit = less_realPack (hit->mags[i], prev_mags[i]);
            prev_mags[i] = hit->mags[i];
            cfill_uintPack (&hit->objs[i], objidx, didhit);
        }
    }
}

static
    void
RayCastAPriori_to_RayPacket (RayPacket* pkt,
                             const RayCastAPriori* known,
                             uint row_off, uint col_off,
                             const RayImage* image)
{
    uint i, j;
    PointPack col_directs;
    PointPack directs[RayPacketDimSz];

    UUFor( i, NDimensions, j, RayPacketDimSz )
        col_directs.vcoords[i][j] =  known->col_delta.coords[i] * (j+col_off);

    UFor( i, RayPacketDimSz )
    {
        Point tmp;
        scale_Point (&tmp, &known->row_delta, i+row_off);
        summ_Point (&tmp, &tmp, &known->dir_start);
        fill_PointPack (&directs[i], &tmp);
        summ_PointPack (&directs[i], &directs[i], &col_directs);
    }

    UFor( i, RayPacketDimSz )
    {
        if (image->perspective)
        {
            fill_PointPack (&pkt->origins[i], &known->origin);
            normalize_PointPack (&pkt->directs[i], &directs[i]);
        }
        else
        {
            copy_PointPack (&pkt->origins[i], &directs[i]);
            fill_PointPack (&pkt->directs[i], &known->origin);
        }
    }
}

static
    void
cast_packet_RayImage (RayImage* image, uint row_off, uint col_off,
                      const RaySpace* space,
                      const RayCastAPriori* known)
{
    uint i;
    boolPack inside_box[RayPacketDimSz];
    RayPacket pkt;
    RayHitPacket hit;

    RayCastAPriori_to_RayPacket (&pkt, known, row_off, col_off, image);

    UFor( i, RayPacketDimSz )
    {
        hit.inds[i] = fill_uintPack (Max_uint);
        hit.objs[i] = fill_uintPack (Max_uint);
        hit.mags[i] = fill_realPack (Max_real);

#if 0
        inside_box[i] =
            (image->perspective
             ? inside_box[i] = fill_boolPack (known->inside_box)
             : inside_pack_BoundingBox (&space->main.box, &pkt.origins[i]));
#else
        inside_box[i] = inside_pack_BoundingBox (&space->main.box,
                                                 &pkt.origins[i]);
#endif

    }

    cast_packet_nopartition (&hit, &pkt, space, inside_box, Max_uint);

    UFor( i, RayPacketDimSz )
    {
        uint hit_inds[RayPacketDimSz];
        uint hit_objs[RayPacketDimSz];
        real hit_mags[RayPacketDimSz];
        uint row, j;

        row = i + row_off;
        if (row >= image->nrows)  break;

        scat_uintPack (hit_inds, hit.inds[i]);
        scat_uintPack (hit_objs, hit.objs[i]);
        scat_realPack (hit_mags, hit.mags[i]);

        UFor( j, RayPacketDimSz )
        {
            uint img_idx;
            img_idx = j + col_off + row * image->stride;

            if (image->hits)  image->hits[img_idx] = hit_inds[j];
            if (image->mags)  image->mags[img_idx] = hit_mags[j];
            if (image->pixels)
            {
                uint dim;
                real colors[NColors];
                uint color_idx;
                Point origin, direct;

                UFor( dim, NDimensions )
                {
                    origin.coords[dim] = pkt.origins[i].vcoords[dim][j];
                    direct.coords[dim] = pkt.directs[i].vcoords[dim][j];
                }

                UFor( color_idx, NColors )  colors[color_idx] = 0;
                fill_pixel (colors,
                            hit_inds[j], hit_mags[j], hit_objs[j],
                            image, &origin, &direct,
                            space, 0);
                UFor( color_idx, NColors )
                {
                    image->pixels[3*img_idx + color_idx] = (byte)
                        clamp_real (255.5 * colors[color_idx], 0, 255.5);
                }
            }
        }
    }
}

