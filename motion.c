
#ifndef __OPENCL_VERSION__
#include "motion.h"

#include "bbox.h"
#include "bitstring.h"
#include "order.h"
#include "point.h"
#include "serial.h"
#include "simplex.h"
#include "space-junk.h"
#include "xfrm.h"

#include <assert.h>
#include <math.h>
#include <string.h>
#endif  /* #ifndef __OPENCL_VERSION__ */

const bool UseNNCollision = false;

static void
zero_rotations (ObjectMotion* motion);
static void
move_object (RaySpace* space, ObjectMotion* motions,
             BitString* collisions, Point* refldirs,
             uint objidx, real dt);
static void
apply_track_gravity (ObjectMotion* motion, const RaySpace* space,
                     uint objidx, real dt,
                     const Point* centroid,
                     const PointXfrm* orientation);
static void
apply_gravity (ObjectMotion* motion, real dt);
static void
apply_thrust (Point* veloc,
              PointXfrm* orientation,
              real* drift,
              const ObjectMotion* motion,
              real dt);
static void
mark_colliding (BitString* collisions,
                Point* refldirs,
                ObjectMotion* motions,
                uint objidx, const RaySpace* space);
static bool
detect_collision (ObjectMotion* motions,
                  BitString* collisions,
                  const RaySpace* space,
                  uint objidx,
                  const Point* new_centroid,
                  const PointXfrm* new_orientation,
                  const Point* displacement,
                  real dt);
static void
remove_4d_rotation (PointXfrm* basis);

static void
set_pts_ObjectMotion (ObjectMotion* motion, const Scene* scene)
{
    uint i;
    uint n = scene->nverts;
    uint* jumps = AllocT( uint, n );
    uint* indices = AllocT( uint, n );
    real* coords = AllocT( real, n );
    real* lexis = AllocT( real, n * NDims );

    if (n == 0)  return;

    UFor( i, n )
    {
        uint j;
        const Point* p = &scene->verts[i];
        UFor( j, NDims )
            lexis[j + i * NDims] = p->coords[j];
    }

    motion->npts =
        condense_lexi_reals (jumps, indices, coords, n, NDims, lexis);
    motion->pts = AllocT( Point, motion->npts );

    UFor( i, motion->npts )
    {
        uint j;
        Point* p = &motion->pts[i];
        UFor( j, NDims )
            p->coords[j] = lexis[j + i * NDims];
    }

    free (jumps);
    free (indices);
    free (coords);
    free (lexis);
}

    void
init_ObjectMotion (ObjectMotion* motion, const ObjectRaySpace* object)
{
    uint i;
    motion->mass = 1;
    zero_Point (&motion->veloc);
    UFor( i, 2 )  motion->thrust[i] = 0;
    motion->boost = false;
    UFor( i, N2DimRotations )
        motion->rots[i] = 0;
    motion->collide = true;
    motion->gravity = true;
    motion->stabilize = true;

    motion->flying = true;
    motion->hover_height = 0;
    UFor( i, NDimensions )
    {
        real x;
        x = object->box.max.coords[i] - object->box.min.coords[i];
        if (motion->hover_height < x)
            motion->hover_height = x;
    }
    motion->hover_height *= .4;
    motion->escape_height = 5 * motion->hover_height;
    zero_Point (&motion->track_normal);
    motion->track_normal.coords[UpDim] = 1;
    motion->laps = 0;
    motion->checkpoint_idx = 0;
    motion->lock_drift = false;

    set_pts_ObjectMotion (motion, &object->scene);
}

    void
lose_ObjectMotion (ObjectMotion* motion)
{
    if (motion->npts > 0)  free (motion->pts);
}

    /** Rotate the (/xdim/,/ydim/)-plane by /angle/ counterclockwise,
     * or more intuitively, the positive /xdim/-axis moves toward the
     * positive /ydim/-axis for the first quarter-rotation.
     **/
    void
rotate_object (ObjectMotion* motion, uint xdim, uint ydim, real angle)
{
    assert (xdim != ydim);
    if (xdim < ydim)
        motion->rots[assoc_index (NDimensions, xdim, ydim)] += angle;
    else
        motion->rots[assoc_index (NDimensions, ydim, xdim)] -= angle;
}

    void
move_objects (RaySpace* space, ObjectMotion* motions, real dt,
              uint ncheckplanes, const Plane* checkplanes)
{
    uint nincs = 10;
    real inc;
    uint i, nobjects;
    Point* prev_centroids;
    BitString* collisions;
    Point* refldirs;

        /* Make our time increment about 1/200 sec.*/
    nincs = (uint) (dt * 200);
    nincs = clamp_real (nincs, 10, 200);

    nobjects = space->nobjects;
    prev_centroids = AllocT( Point, nobjects );

    UFor( i, nobjects )
        copy_Point (&prev_centroids[i], &space->objects[i].centroid);

    i = 1 + space->nobjects;
    i *= i;
    collisions = alloc_BitString (i, false);
    refldirs = AllocT( Point, i );

    UFor( i, nobjects )
        mark_colliding (collisions, refldirs, motions, i, space);

    inc = dt / nincs;

    UFor( i, nincs )
    {
        uint j;
        UFor( j, nobjects )
            move_object (space, motions, collisions, refldirs, j, inc);
    }

    UFor( i, nobjects )
    {
        ObjectMotion* mot;
        mot = &motions[i];

        zero_rotations (mot);
        mot->thrust[0] = 0;
        mot->thrust[1] = 0;

        if (ncheckplanes > 0)
        {
            uint idx;
            const Point* prev_centroid;
            const Point* centroid;
            prev_centroid = &prev_centroids[i];
            centroid = &space->objects[i].centroid;
            idx = mot->checkpoint_idx;

                /* See which checkplanes are passed.*/
            while ((0 > dist_Plane (&checkplanes[idx], prev_centroid)) ==
                   (0 < dist_Plane (&checkplanes[idx], centroid)))
            {
                FILE* out = stderr;
                fprintf (out, "Racer:%u passed checkplane:%u.\n", i, idx);
                idx = incmod_uint (idx, 1, ncheckplanes);
                if (idx == 0)
                {
                    mot->laps += 1;
                    fprintf (out, "Racer:%u gets lap:%u.\n", i, mot->laps);
                }
            }
            mot->checkpoint_idx = idx;
        }
    }

    free (prev_centroids);
    free_BitString (collisions);
    free (refldirs);
}

    void
zero_rotations (ObjectMotion* motion)
{
#ifdef __OPENCL_VERSION__
    uint i;
    UFor( i, N2DimRotations )
        motion->rots[i] = 0;
#else
    memset (motion->rots, 0, N2DimRotations * sizeof (real));
#endif
}

    void
move_object (RaySpace* space, ObjectMotion* motions,
             BitString* collisions, Point* refldirs,
             uint objidx, real dt)
{
    bool commit_move = true;
    uint i;
    real drift;
    Point veloc;
    Point new_centroid;
        /* PointXfrm basis; */
    PointXfrm new_orientation, rotation;
    ObjectRaySpace* object;
    ObjectMotion* motion;

    object = &space->objects[objidx];
    motion = &motions[objidx];
    identity_PointXfrm (&rotation);

        /* Rotate object.*/
    UFor( i, NDimensions )
    {
        uint j;
        UFor( j, i )
        {
            uint idx;
            real angle;
            idx = assoc_index (NDimensions, j, i);
            angle = motion->rots[idx] * dt;
            assert (angle < + M_PI / 2);
            assert (angle > - M_PI / 2);
            rotate_PointXfrm (&rotation, j, i, angle);
        }
    }

    trxfrm_PointXfrm (&new_orientation, &rotation, &object->orientation);
    apply_thrust (&veloc, &new_orientation, &drift, motion, dt);

    copy_PointXfrm (&object->orientation, &new_orientation);
    mark_colliding (collisions, refldirs, motions, objidx, space);

    if (NDimensions == 4)
    {
        veloc.coords[DriftDim] =
            drift - object->centroid.coords[DriftDim];
        if (motion->lock_drift)
            veloc.coords[DriftDim] = 0;
    }

    copy_Point (&motion->veloc, &veloc);
    scale_Point (&veloc, &veloc, dt);
    summ_Point (&new_centroid, &object->centroid, &veloc);

    if (NDimensions == 4)
    {
        real offset, lo, hi;
        offset = .5 * (object->box.max.coords[DriftDim] -
                       object->box.min.coords[DriftDim]);
        lo = space->main.box.min.coords[DriftDim] + offset;
        hi = space->main.box.max.coords[DriftDim] - offset;
#if 0
        drift = clamp_real (drift, lo, hi);
        new_centroid.coords[DriftDim] = drift;
#else
        new_centroid.coords[DriftDim] =
            clamp_real (new_centroid.coords[DriftDim], lo, hi);
#endif
    }

    apply_track_gravity (motion, space, objidx, dt,
                         &new_centroid, &new_orientation);
    if (motion->collide)
    {
        bool hit;
        hit = detect_collision (motions, collisions,
                                space, objidx,
                                &new_centroid,
                                &new_orientation,
                                &veloc,
                                dt);
        if (hit)
        {
            commit_move = false;
        }

    }

    if (commit_move)
    {
        copy_Point (&object->centroid, &new_centroid);
        copy_PointXfrm (&object->orientation, &new_orientation);
        apply_gravity (motion, dt);
    }
}

    void
apply_track_gravity (ObjectMotion* motion, const RaySpace* space,
                     uint objidx, real dt,
                     const Point* centroid,
                     const PointXfrm* orientation)
{
    bool inside_box;
    uint hit_idx;
    real hit_mag;
    Point origin, direct;
    const ObjectRaySpace* object;

    (void) dt;

    if (!motion->gravity)  return;

    object = &space->objects[objidx];

    if (motion->flying)
        negate_Point (&direct, &orientation->pts[UpDim]);
    else
        negate_Point (&direct, &motion->track_normal);
    copy_Point (&origin, centroid);
    inside_box = inside_BBox (&space->main.box, &origin);

    hit_idx = Max_uint;
    hit_mag = motion->escape_height;
    cast1_ObjectRaySpace (&hit_idx, &hit_mag,
                          &origin, &direct,
                          &space->main, inside_box, May);

    motion->flying = hit_idx >= space->main.nelems;
    if (!motion->flying)
    {
        Point normal;
        const BarySimplex* simplex;
        real cos_normal;
        simplex = &space->main.simplices[hit_idx];

            /* TODO: Add a case for normal calculated from bump map.*/
        if (space->main.scene.nvnmls > 0)
        {
            Point isect, bpoint;
            Op_Point_2010( &isect ,+, &origin ,hit_mag*, &direct );
            barycentric_Point (&bpoint, &isect, simplex);

            map_vertex_normal (&normal,
                               space->main.scene.vnmls,
                               &space->main.scene.elems[hit_idx],
                               &bpoint);
        }
        else
        {
            copy_Point (&normal, &simplex->plane.normal);
        }

            /* Neither of the above cases give a normalized vector!*/
        normalize_Point (&normal, &normal);

        cos_normal = dot_Point (&simplex->plane.normal, &direct);
        if (0 > cos_normal)
            cos_normal = - cos_normal;
        else
            negate_Point (&normal, &normal);

        if (cos_normal < .1)
            motion->flying = true;
        else
            copy_Point (&motion->track_normal, &normal);
    }

    if (!motion->flying)
    {
        real x0, v0;
        real v1, x1;
        real v2, x2;
        x0 = hit_mag + object->box.min.coords[UpDim];
        v0 = dot_Point (&motion->track_normal, &motion->veloc);

        if (x0 <= motion->hover_height)
            v1 = v0 + motion->hover_height - x0;
        else
            v1 = v0 + ((x0 - motion->hover_height)
                       / (motion->escape_height - motion->hover_height));

            /* Note: this dt is optional?*/
            /* x1 = x0 + dt * v1; */
        x1 = x0 + v1;

        if (x1 <= motion->hover_height)
            x2 = motion->hover_height * exp (x1 / motion->hover_height);
        else
            x2 = motion->hover_height;

        v2 = x2 - x0;

        Op_Point_2010( &motion->veloc
                       ,+, &motion->veloc
                       ,   (v2-v0)*, &motion->track_normal );

        if (NDimensions == 4)
        {
            motion->track_normal.coords[DriftDim] = 0;
            normalize_Point (&motion->track_normal, &motion->track_normal);
        }
    }
}

    void
apply_gravity (ObjectMotion* motion, real dt)
{

    Point dv;
    real accel;
        /* TODO: Totally arbitrary gravity!*/
    accel = -1980;
    zero_Point (&dv);
    dv.coords[UpDim] = accel * dt;

    if (!motion->flying)
        orth_unit_Point (&dv, &dv, &motion->track_normal);

    summ_Point (&motion->veloc, &motion->veloc, &dv);
}

    void
apply_thrust (Point* veloc,
              PointXfrm* orientation,
              real* drift,
              const ObjectMotion* motion,
              real dt)
{
    const real alpha = 3;  /* Arbitrary coefficient.*/
        /* TODO: Totally arbitrary thrusters!*/
    real vthrust = 3 * 733;  /* Velocity of thrusters.*/
    real accel;
    real forward_mag, up_mag, drift_mag;
    Point forward_veloc, up_veloc, drift_veloc;
    Point tmp;
    PointXfrm basis;

    copy_PointXfrm (&basis, orientation);

    if (motion->boost)  vthrust *= 2;

    if (motion->thrust[0] > motion->thrust[1])  vthrust *= motion->thrust[0];
    else                                        vthrust *= motion->thrust[1];

    if (motion->stabilize && !motion->flying)
    {
        if (true)
        {
            real dot;
            Point normal;
            dot = dot_Point (&basis.pts[UpDim], &motion->track_normal);
            Op_Point_2010( &normal
                           ,+, &basis.pts[UpDim]
                           ,   .01*(1-dot)*dt*, &motion->track_normal );
            
            orthorotate_PointXfrm (&basis, &basis, &normal, UpDim);
        }
        else
        {
            Point normal;
            proj_unit_Point (&normal, &basis.pts[UpDim],
                             &motion->track_normal);
            orthorotate_PointXfrm (&basis, &basis, &normal, UpDim);
        }
    }
    else
    {
        real roll;
        roll = motion->thrust[1] - motion->thrust[0];
        rotate_PointXfrm (&basis, RightDim, UpDim, roll * dt * (M_PI / 2));
    }
    remove_4d_rotation (&basis);
    copy_PointXfrm (orientation, &basis);

    copy_Point (veloc, &motion->veloc);

        /* Forward component.*/
    forward_mag = dot_Point (veloc, &orientation->pts[ForwardDim]);
    scale_Point (&forward_veloc, &orientation->pts[ForwardDim], forward_mag);

    diff_Point (&drift_veloc, veloc, &forward_veloc);
        /* Up component.*/
    if (motion->stabilize && !motion->flying)
    {
        up_mag = dot_Point (&drift_veloc, &motion->track_normal);
        scale_Point (&up_veloc, &motion->track_normal, up_mag);
    }
    else
    {
        up_mag = dot_Point (&drift_veloc, &orientation->pts[UpDim]);
        scale_Point (&up_veloc, &orientation->pts[UpDim], up_mag);
    }
        /* Drift component.*/
    diff_Point (&drift_veloc, &drift_veloc, &up_veloc);
    drift_mag = magnitude_Point (&drift_veloc);

    *drift = drift_mag;


    accel = alpha * (vthrust - forward_mag);
    scale_Point (&tmp, &orientation->pts[ForwardDim], accel * dt);
    summ_Point (&forward_veloc, &forward_veloc, &tmp);

    if (up_mag != 0)
    {
        accel = - alpha;
        scale_Point (&tmp, &up_veloc, accel * dt);
        summ_Point (&up_veloc, &up_veloc, &tmp);
    }

    if (drift_mag != 0)
    {
        const real grip_min = .2;
        real grip = 5;

        grip -= grip_min;
        if (0 < dot_Point (&drift_veloc, &orientation->pts[RightDim]))
            grip *= motion->thrust[0];
        else
            grip *= motion->thrust[1];

        if (grip < 0)  grip = grip_min - grip;
        else           grip = grip_min + grip;

        accel = -grip;
        scale_Point (&tmp, &drift_veloc, accel * dt);
        summ_Point (&drift_veloc, &drift_veloc, &tmp);
    }

    Op_Point_20200( veloc
                    ,+, &forward_veloc
                    ,   +, &up_veloc
                    ,      &drift_veloc );

    if (motion->stabilize && !motion->flying)
    {
        Point up;
        Op_Point_2100( &up
                       ,-, (500+drift_mag)*, &motion->track_normal
                       ,   &drift_veloc );

        normalize_Point (&up, &up);
#if 0
        Op_Point_2100( &up
                        ,+, 10*dt*, &up
                        ,   &basis.pts[UpDim] );
#endif
        orthorotate_PointXfrm (&basis, &basis, &up, UpDim);
    }
    remove_4d_rotation (&basis);
    orthonormalize_PointXfrm (orientation, &basis);
        /* copy_PointXfrm (orientation, &basis); */
    assert (0 < det_PointXfrm (orientation));
}

    /** Mark which objects are colliding with which.
     * I don't much like this function.
     **/
    void
mark_colliding (BitString* collisions,
                Point* refldirs,
                ObjectMotion* motions,
                uint objidx, const RaySpace* space)
{
    uint i, off, tmp_off, n;
    const ObjectRaySpace* object;
    const ObjectMotion* motion;
    FILE* out = stderr;

    if (!UseNNCollision)  return;

    object = &space->objects[objidx];
    motion = &motions[objidx];

    n = space->nobjects+1;
    off = n * objidx;
    tmp_off = n * space->nobjects;

    UFor( i, n )
    {
        uint idx;
        idx = tmp_off + i;
        set0_BitString (collisions, idx);
        zero_Point (&refldirs[idx]);
    }

    UFor( i, motion->npts )
    {
        bool inside_box;
        uint hit_idx = Max_uint;
        real hit_mag;
        uint hit_objidx = Max_uint;
        Point origin, direct;

        copy_Point (&origin, &motion->pts[i]);
        hit_mag = magnitude_Point (&origin);
        scale_Point (&direct, &origin, - 1 / hit_mag);

        ray_from_basis (&origin, &direct, &object->orientation,
                        &origin, &direct, &object->centroid);

        inside_box = inside_BBox (&space->main.box, &origin);
        cast_nopartition (&hit_idx, &hit_mag, &hit_objidx,
                          space, &origin, &direct,
                          inside_box, May, objidx);

        if (hit_objidx < n)
        {
            if (false)
            {
                output_Point (out, &origin);
                fputc (' ', out);
                output_Point (out, &direct);
                fputc ('\n', out);
            }

            if (!test_BitString (collisions, off + hit_objidx) &&
                !set1_BitString (collisions, tmp_off + hit_objidx))
            {
                Point veloc, normal;

                Op_Point_1200( &veloc
                               ,-, +, &motion->veloc
                               ,      &direct );

                Op_Point_202100( &normal
                                 ,-, &object->centroid
                                 ,   +, hit_mag *, &direct
                                 ,      &origin );

                normalize_Point (&normal, &normal);
                reflect_Point (&veloc, &veloc, &normal,
                               dot_Point (&veloc, &normal));

                copy_Point (&refldirs[tmp_off + hit_objidx], &veloc);
                    /* TODO: Need more samples than the first!
                     * Do velocity stuff in loop below!
                     */
                summ_Point (&refldirs[tmp_off + hit_objidx],
                            &refldirs[tmp_off + hit_objidx],
                            &normal);
            }
        }
    }

    UFor( i, n )
    {
        bool collide;
        collide = set0_BitString (collisions, tmp_off + i);

        if (collide)
        {
            Point* veloc;

            assert (i != objidx);
            veloc = &motions[objidx].veloc;

            if (set1_BitString (collisions, off + i))
            {
                real arbitrary_spring = 10;
                real dot;
                const Point* direct;
                direct = &refldirs[off + i];

                dot = dot_Point (veloc, direct);
                if (dot < 0)
                    dot = - dot;
                
                scale_Point (veloc, direct, dot + arbitrary_spring);
            }
            else
            {
                copy_Point (veloc, &refldirs[tmp_off + i]);
                normalize_Point (&refldirs[off + i], &refldirs[tmp_off + i]);
            }
        }
        else
        {
            set0_BitString (collisions, off + i);
        }
    }
}

    bool
detect_collision (ObjectMotion* motions,
                  BitString* collisions,
                  const RaySpace* space,
                  uint objidx,
                  const Point* new_centroid,
                  const PointXfrm* new_orientation,
                  const Point* displacement,
                  real dt)
{
    uint i;
    uint hit_idx = Max_uint;
    real hit_mag = Max_real;
    uint hit_objidx = Max_uint;
    uint bs_offset;
    real hit_dx = 0;
    Point hit_dir, reflveloc;
    const ObjectRaySpace* object;
    ObjectMotion* motion;
    bool colliding;


    assert (objidx < space->nobjects);
    object = &space->objects[objidx];
    motion = &motions[objidx];
    zero_Point (&hit_dir);

    bs_offset = objidx * (space->nobjects+1);

        /* Detect if we run into other objects' points.*/
    if (UseNNCollision)
    UFor( i, space->nobjects )
    {
        uint j;
        uint eff_objidx;
        BBox box, tmpbox;
        PointXfrm basis;
        Point cent, centoff;
        const ObjectRaySpace* query_object;

        if (i == objidx)  eff_objidx = space->nobjects;
        else              eff_objidx = i;

            /* set0_BitString (collisions, bs_offset + eff_objidx); */

        if (eff_objidx == space->nobjects)
        {
            query_object = &space->main;
            zero_Point (&centoff);
        }
        else
        {
            query_object = &space->objects[i];
            negate_Point (&centoff, &query_object->centroid);
        }

        xfrm_PointXfrm (&basis, &query_object->orientation,
                        &object->orientation);
        summ_Point (&cent, &centoff, &object->centroid);
        trxfrm_BBox (&box, &basis, &object->box, &cent);


        xfrm_PointXfrm (&basis, &query_object->orientation,
                        new_orientation);
        summ_Point (&cent, &centoff, new_centroid);
        trxfrm_BBox (&tmpbox, &basis, &object->box, &cent);

        merge_BBox (&box, &box, &tmpbox);

        j = inside_BBox_KPTree (&query_object->verttree, &box, Max_uint);
        while (j != Max_uint)
        {
            bool inside_box;
            bool inside_object = false;
            uint tmp_hit;
            real tmp_mag;
            Point p0, p1, direct;
            Point tmp;
            FILE* out = stderr;
            if (eff_objidx == space->nobjects)
            {
                copy_Point (&p1,
                            &query_object->verttree.nodes[j].loc);
                copy_Point (&p0, &p1);
            }
            else
            {
                xfrm_Point (&p1, &query_object->orientation,
                            &query_object->verttree.nodes[j].loc);
                summ_Point (&p1, &p1, &query_object->centroid);
                diff_Point (&p0, &p1, &object->centroid);
            }

            diff_Point (&p0, &p0, displacement);
            
            if (eff_objidx < space->nobjects)
                summ_Point (&p0, &p0, &object->centroid);
                /* /p1/ and /p0/ are now in global coordinates.*/
            
            diff_Point (&tmp, &p1, &object->centroid);
            trxfrm_Point (&p1, &object->orientation, &tmp);

            diff_Point (&tmp, &p0, &object->centroid);
            trxfrm_Point (&p0, &object->orientation, &tmp);
                /* /p1/ and /p0/ are now in local coordinates.*/
            
            if (false)
            {
                output_Point (out, &p1);
                fputs (" <- ", out);
                output_Point (out, &p0);
                fputc ('\n', out);
            }


            inside_box = inside_BBox (&object->box, &p0);
            if (inside_box)
            {
                tmp_hit = Max_uint;
                tmp_mag = Max_real;
                normalize_Point (&direct, &p0);

                cast1_ObjectRaySpace (&tmp_hit, &tmp_mag, &p0, &direct,
                                      object, inside_box, May);

                if (tmp_hit == Max_uint)
                {
                    inside_object = true;
                    set1_BitString (collisions, bs_offset + eff_objidx);
                }
            }

            if (!inside_object)
            {
                diff_Point (&direct, &p0, &p1);
                normalize_Point (&direct, &direct);
                inside_box = inside_BBox (&object->box, &p1);
                tmp_hit = Max_uint;
                tmp_mag = Max_real;
                cast1_ObjectRaySpace (&tmp_hit, &tmp_mag, &p1, &direct,
                                      object, inside_box, May);

                if (tmp_mag < hit_mag)
                {
                    hit_idx = tmp_hit;
                    hit_mag = tmp_mag;
                    hit_objidx = eff_objidx;
                }

            }
            j = inside_BBox_KPTree (&query_object->verttree,
                                           &box, j);
        }
    }

    if (true)
    UFor( i, motion->npts )
    {
        Point diff, origin, destin;
        real distance;
        Point unit_dir;
        real tmp_mag;
        uint tmp_hit = Max_uint, tmp_objidx = Max_uint;
        bool inside_box;

        trxfrm_Point (&origin, &object->orientation, &motion->pts[i]);
        trxfrm_Point (&destin, new_orientation, &motion->pts[i]);

        summ_Point (&origin, &origin, &object->centroid);
        summ_Point (&destin, &destin, new_centroid);

        inside_box = inside_BBox (&space->main.box, &origin);

        diff_Point (&unit_dir, &object->centroid, &origin);
        distance = magnitude_Point (&unit_dir);
        tmp_mag = distance;
        scale_Point (&unit_dir, &unit_dir, 1 / distance);

        cast_nopartition (&tmp_hit, &tmp_mag, &tmp_objidx,
                          space, &origin, &unit_dir,
                          inside_box, May, objidx);
        if (tmp_mag < distance)  continue;


        diff_Point (&diff, &destin, &origin);
        distance = magnitude_Point (&diff);


        if (0 < distance)
        {

            scale_Point (&unit_dir, &diff, 1 / distance);

            tmp_hit = Max_uint;
            tmp_mag = distance;
            tmp_objidx = Max_uint;
            cast_nopartition (&tmp_hit, &tmp_mag, &tmp_objidx,
                              space, &origin, &unit_dir,
                              inside_box, May, objidx);
            if (tmp_mag < distance &&
                tmp_mag < hit_mag &&
                !test_BitString (collisions, bs_offset + tmp_objidx) &&
                !test_BitString (collisions, tmp_objidx * (space->nobjects+1) + objidx))
            {
                hit_idx = tmp_hit;
                hit_mag = tmp_mag;
                hit_objidx = tmp_objidx;
                hit_dx = distance;
                copy_Point (&hit_dir, &unit_dir);
            }
        }
    }

    colliding = false;
    if (hit_objidx <= space->nobjects)
    {
        Point normal;
        real cos_normal;
        real hit_speed;
        const ObjectRaySpace* hit_object;

            /* OVERWRITE values!*/
        hit_dx = magnitude_Point (displacement);
        scale_Point (&hit_dir, displacement, 1 / hit_dx);

        hit_speed = hit_dx / dt;
        colliding = true;

        if (hit_objidx < space->nobjects)
        {
            hit_object = &space->objects[hit_objidx];
#if 0
            xfrm_Point (&normal, &hit_object->orientation,
                        &hit_object->simplices[hit_idx].plane.normal);
#else
            diff_Point (&normal, &object->centroid, &hit_object->centroid);
            normalize_Point (&normal, &normal);
#endif
        }
        else
        {
            hit_object = &space->main;
            copy_Point (&normal,
                        &hit_object->simplices[hit_idx].plane.normal);
        }

        cos_normal = dot_Point (&hit_dir, &normal);
        if (cos_normal < 0)
            cos_normal = - cos_normal;
        else
            negate_Point (&normal, &normal);

        if (hit_objidx < space->nobjects)
        {
            real m1, m2, invavg;
            Point u1, u2;  /* Initial velocities.*/
            Point w1, w2;
            Point v1, v2;  /* Final velocities.*/
            Point v;  /* Average velocity.*/


            m1 = motion->mass;
            m2 = motions[hit_objidx].mass;
            invavg = 2 / (m1+m2);
            scale_Point (&u1, &normal, - cos_normal * hit_speed);
            copy_Point (&u2, &motions[hit_objidx].veloc);

            scale_Point (&w1, &u1, m1 * invavg);
            scale_Point (&w2, &u2, m2 * invavg);

            summ_Point (&v, &w1, &w2);
            diff_Point (&v1, &v, &u1);
            diff_Point (&v2, &v, &u2);

            if (false)
            {
                FILE* out = stderr;
                fprintf (out, "objidx:%u\n", objidx);
                fputs ("v1:", out);
                output_Point (out, &v1);
                fputs ("  v2:", out);
                output_Point (out, &v2);
                fputc ('\n', out);
            }

            copy_Point (&motions[hit_objidx].veloc, &v2);
            
                /* Project reflection onto plane.*/
            scale_Point (&reflveloc, &hit_dir, hit_speed);
            diff_Point (&reflveloc, &reflveloc, &u1);
                /* Add in the velocity. Reflection probably
                 * does not lie on the plane after this point.
                 */
            summ_Point (&reflveloc, &reflveloc, &v1);
        }
        else
        {
            scale_Point (&reflveloc, &normal, 2 * cos_normal);
            summ_Point (&reflveloc, &hit_dir, &reflveloc);
            scale_Point (&reflveloc, &reflveloc, hit_speed);
        }
        scale_Point (&motion->veloc, &reflveloc, 1);
    }

    return colliding;
}

    void
remove_4d_rotation (PointXfrm* basis)
{
    if (NDimensions == 4)
    {
        Point v;
        zero_Point (&v);
        v.coords[DriftDim] = 1;
        orthorotate_PointXfrm (basis, basis, &v, DriftDim);
    }
}

