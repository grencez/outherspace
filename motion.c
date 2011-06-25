
#ifndef __OPENCL_VERSION__
#include "motion.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

static void
zero_rotations (ObjectMotion* motion);
static void
move_object (RaySpace* space, ObjectMotion* motions, uint objidx, real dt);
static bool
detect_collision (ObjectMotion* motions,
                  const RaySpace* space,
                  uint objidx,
                  const Point* new_centroid,
                  const PointXfrm* new_orientation,
                  const Point* displacement,
                  const PointXfrm* rotation,
                  real dt);

    void
init_ObjectMotion (ObjectMotion* motion)
{
    uint i;
    motion->mass = 1;
    zero_Point (&motion->veloc);
    motion->thrust = 0;
    UFor( i, N2DimRotations )
        motion->rots[i] = 0;
    motion->collide = true;
        /* Not implemented yet.*/
    motion->gravity = false;
}

    void
rotate_object (ObjectMotion* motion, uint xdim, uint ydim, real angle)
{
    assert (xdim != ydim);
    if (xdim < ydim)
        motion->rots[xdim * (NDimensions - 2) + ydim - 1] += angle;
    else
        motion->rots[ydim * (NDimensions - 2) + xdim - 1] -= angle;
}

    void
move_objects (RaySpace* space, ObjectMotion* motions, real dt)
{
    const uint nincs = 10;
    ObjectRaySpace* objects;
    real inc;
    uint i, nobjects;

    nobjects = space->nobjects;
    objects = space->objects;

    inc = dt / nincs;

    UFor( i, nincs )
    {
        uint j;
        UFor( j, nobjects )
            move_object (space, motions, j, inc);
    }

    UFor( i, nobjects )
    {
        zero_rotations (&motions[i]);
        motions[i].thrust = 0;
    }
}

    void
zero_rotations (ObjectMotion* motion)
{
#ifdef __OPENCL_VERSION__
    uint i;
    UFor( i, N2DimRotations )
    {
        motion->rots[i] = 0;
    }
#else
    memset (motion->rots, 0, N2DimRotations * sizeof (real));
#endif
}

    void
move_object (RaySpace* space, ObjectMotion* motions, uint objidx, real dt)
{
    bool commit_move = true;
    uint i;
    Point veloc;
    Point new_centroid;
    PointXfrm basis;
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
            idx = j * (NDimensions - 2) + i - 1;
            angle = motion->rots[idx] * dt;
            assert (angle < + M_PI / 2);
            assert (angle > - M_PI / 2);
            rotate_PointXfrm (&rotation, j, i, angle);
        }
    }

    trxfrm_PointXfrm (&basis, &rotation, &object->orientation);
    orthonormalize_PointXfrm (&new_orientation, &basis);

    if (motion->thrust != 0)
    {
        const real alpha = 5;  /* Arbitrary coefficient.*/
        const real vthrust = 733;  /* Velocity of thrusters.*/
        real accel, mag;
        Point tmp;

            /* Assume full grip causes orthogonal motion to be lost.*/
        proj_Point (&veloc, &motion->veloc, &new_orientation.pts[DirDimension]);

        mag = magnitude_Point (&veloc);
        accel = alpha * (vthrust - mag);
        if (motion->thrust < 0)  accel = - accel;

        scale_Point (&tmp, &new_orientation.pts[DirDimension], accel * dt);
        summ_Point (&veloc, &veloc, &tmp);
    }
    else
    {
        copy_Point (&veloc, &motion->veloc);
            /* HACK to make racer stop fast!*/
            /* Half the speed every second.*/
        scale_Point (&veloc, &veloc, pow (0.5, dt));
    }

    copy_Point (&motion->veloc, &veloc);
    scale_Point (&veloc, &veloc, dt);
    summ_Point (&new_centroid, &object->centroid, &veloc);

    if (motion->collide)
    {
        bool hit;
        hit = detect_collision (motions,
                                space, objidx,
                                &new_centroid,
                                &new_orientation,
                                &veloc,
                                &rotation,
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
    }
}

    bool
detect_collision (ObjectMotion* motions,
                  const RaySpace* space,
                  uint objidx,
                  const Point* new_centroid,
                  const PointXfrm* new_orientation,
                  const Point* displacement,
                  const PointXfrm* rotation,
                  real dt)
{
    uint i;
    uint hit_idx = Max_uint;
    real hit_mag = Max_real;
    uint hit_objidx = Max_uint;
    real hit_dx = 0;
    Point hit_dir, reflveloc;
    const ObjectRaySpace* object;
    const Scene* scene;
    bool colliding;

    (void) displacement;
    (void) rotation;

    assert (objidx < space->nobjects);
    object = &space->objects[objidx];
    scene = &object->scene;
    zero_Point (&hit_dir);

    UFor( i, scene->nverts )
    {
        Point diff, origin, destin;
        real distance;

        centroid_BoundingBox (&diff, &object->box);
        diff_Point (&diff, &scene->verts[i], &diff);

        trxfrm_Point (&origin, &object->orientation, &diff);
        trxfrm_Point (&destin, new_orientation, &diff);

        summ_Point (&origin, &origin, &object->centroid);
        summ_Point (&destin, &destin, new_centroid);

        diff_Point (&diff, &destin, &origin);
        distance = magnitude_Point (&diff);


        if (0 < distance)
        {
            uint tmp_hit;
            real tmp_mag;
            uint tmp_object;
            bool inside_box;
            Point unit_dir;

            scale_Point (&unit_dir, &diff, 1 / distance);
            inside_box = inside_BoundingBox (&space->main.box, &origin);

            tmp_hit = Max_uint;
            tmp_mag = distance;
            tmp_object = Max_uint;
            cast_nopartition (&tmp_hit, &tmp_mag, &tmp_object,
                              space, &origin, &unit_dir,
                              inside_box, objidx);
            if (tmp_mag < distance &&
                tmp_mag < hit_mag)
            {
                hit_idx = tmp_hit;
                hit_mag = tmp_mag;
                hit_objidx = tmp_object;
                hit_dx = distance;
                copy_Point (&hit_dir, &unit_dir);
            }
        }
    }

    UFor( i, space->nobjects )
    {
        uint j;
        BoundingBox box, tmpbox;
        PointXfrm basis;
        Point cent, centoff;
        const ObjectRaySpace* query_object;

        if (i == objidx)
        {
            query_object = &space->main;
            zero_Point (&centoff);
        }
        else
        {
            query_object = &space->objects[i];
            centroid_BoundingBox (&centoff, &query_object->box);
            diff_Point (&centoff, &centoff, &query_object->centroid);
        }

        xfrm_PointXfrm (&basis, &query_object->orientation,
                        &object->orientation);
        summ_Point (&cent, &centoff, &object->centroid);
        trxfrm_BoundingBox (&box, &basis, &object->box, &cent);


        xfrm_PointXfrm (&basis, &query_object->orientation,
                        new_orientation);
        summ_Point (&cent, &centoff, new_centroid);
        trxfrm_BoundingBox (&tmpbox, &basis, &object->box, &cent);

        merge_BoundingBox (&box, &box, &tmpbox);

        j = inside_BoundingBox_KPTree (&query_object->verttree,
                                       &box, Max_uint);
        while (j != Max_uint)
        {
            FILE* out = stderr;
            output_Point (out, &object->verttree.nodes[j].loc);
            fputc ('\n', out);
            j = inside_BoundingBox_KPTree (&query_object->verttree,
                                           &box, j);
        }
    }

    colliding = false;
    if (hit_objidx <= space->nobjects)
    {
        const Point* rel_normal;
        Point normal;
        real cos_normal;
        real hit_speed;
        const ObjectRaySpace* hit_object;

        hit_speed = hit_dx / dt;
        colliding = true;

        if (hit_objidx < space->nobjects)
            hit_object = &space->objects[hit_objidx];
        else
            hit_object = &space->main;

        rel_normal = &hit_object->simplices[hit_idx].plane.normal;
        if (hit_objidx < space->nobjects)
            xfrm_Point (&normal, &hit_object->orientation, rel_normal);
        else
            copy_Point (&normal, rel_normal);

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


            m1 = motions[objidx].mass;
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
        scale_Point (&motions[objidx].veloc, &reflveloc, 1);
    }

    return colliding;
}

