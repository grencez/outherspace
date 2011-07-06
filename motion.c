
#ifndef __OPENCL_VERSION__
#include "motion.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

static void
zero_rotations (ObjectMotion* motion);
static void
move_object (RaySpace* space, ObjectMotion* motions, uint objidx, real dt);
static void
apply_gravity (ObjectMotion* motion, const ObjectRaySpace* object, real dt);
static void
apply_thrust (Point* veloc,
              PointXfrm* orientation,
              const ObjectMotion* motion,
              real dt);
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
    motion->boost = false;
    UFor( i, N2DimRotations )
        motion->rots[i] = 0;
    motion->collide = true;
    motion->gravity = true;
    motion->stabilize = true;
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
            idx = j * (NDimensions - 2) + i - 1;
            angle = motion->rots[idx] * dt;
            assert (angle < + M_PI / 2);
            assert (angle > - M_PI / 2);
            rotate_PointXfrm (&rotation, j, i, angle);
        }
    }

    trxfrm_PointXfrm (&new_orientation, &rotation, &object->orientation);
    apply_gravity (motion, object, dt);
    apply_thrust (&veloc, &new_orientation, motion, dt);
        /* orthorotate_PointXfrm (&new_orientation, &basis, 0); */

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

    /* Gravity just shoots you at 500 m/s to a middle point.*/
    void
apply_gravity (ObjectMotion* motion, const ObjectRaySpace* object, real dt)
{
    real mag, accel;
    if (!motion->gravity)  return;

    mag = motion->veloc.coords[0];

    accel = 2 * (-500 - mag);

    if (object->centroid.coords[0] < 100)
        accel = - accel;
    motion->veloc.coords[0] += accel * dt;
}

    void
apply_thrust (Point* veloc,
              PointXfrm* orientation,
              const ObjectMotion* motion,
              real dt)
{
    const real alpha = 3;  /* Arbitrary coefficient.*/
    real vthrust = 733;  /* Velocity of thrusters.*/
    real accel, mag, magproj, magorth;
    Point proj, orth, tmp;
    PointXfrm basis;

    if (motion->boost)  vthrust = 2000;

    vthrust *= motion->thrust;

    if (motion->stabilize)  orientation->pts[DirDimension].coords[0] = 0;
    normalize_Point (&orientation->pts[0], &orientation->pts[0]);

    copy_Point (veloc, &motion->veloc);

    proj_Point (&proj, &motion->veloc, &orientation->pts[DirDimension]);
    diff_Point (&orth, &motion->veloc, &proj);

    mag = magnitude_Point (&proj);
    magproj = mag;

    if (0 > dot_Point (&orientation->pts[DirDimension], &proj))
        mag = - mag;

    accel = alpha * (vthrust - mag);

    scale_Point (&tmp, &orientation->pts[DirDimension], accel * dt);
    summ_Point (&proj, &proj, &tmp);

    mag = magnitude_Point (&orth);
    magorth = mag;
#if 1
    if (mag == 0)
    {
        zero_Point (&orth);
    }
    else
    {
        accel = - alpha * mag;

        scale_Point (&tmp, &orth, 1 / mag);
        scale_Point (&tmp, &tmp, accel * dt);
        summ_Point (&orth, &orth, &tmp);
    }
#else
        /* HACK to make racer stop fast!*/
        /* Half the speed every second.*/
    scale_Point (&orth, &orth, pow (0.5, dt));
#endif

    summ_Point (veloc, &proj, &orth);

    if (motion->stabilize)
    {
        zero_Point (&orientation->pts[0]);
        orientation->pts[0].coords[0] = 500+magorth;
        diff_Point (&orientation->pts[0], &orientation->pts[0], &orth);
        orthorotate_PointXfrm (&basis, orientation, 0);
    }
    else
    {
        orthonormalize_PointXfrm (&basis, orientation);
    }
    copy_PointXfrm (orientation, &basis);
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


    assert (objidx < space->nobjects);
    object = &space->objects[objidx];
    scene = &object->scene;
    zero_Point (&hit_dir);

    UFor( i, scene->nverts )
    {
        Point diff, origin, destin;
        real distance;

        trxfrm_Point (&origin, &object->orientation, &scene->verts[i]);
        trxfrm_Point (&destin, new_orientation, &scene->verts[i]);

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
            negate_Point (&centoff, &query_object->centroid);
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
            bool inside_box;
            uint tmp_hit;
            real tmp_mag;
            Point p0, p1, direct;
            Point tmp;
            FILE* out = stderr;
            if (i == objidx)
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
            
            xfrm_Point (&tmp, rotation, &p0);
            if (i == objidx)
                copy_Point (&p0, &tmp);
            else
                summ_Point (&p0, &tmp, &object->centroid);
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

            diff_Point (&direct, &p0, &p1);
            normalize_Point (&direct, &direct);
            inside_box = inside_BoundingBox (&object->box, &p1);
            tmp_hit = Max_uint;
            tmp_mag = Max_real;
            cast_ray (&tmp_hit, &tmp_mag, &p1, &direct,
                      object->nelems, object->tree.elemidcs,
                      object->tree.nodes,
                      object->simplices, object->elems,
                      &object->box, inside_box);

            if (tmp_mag < hit_mag)
            {
                hit_idx = tmp_hit;
                hit_mag = tmp_mag;
                hit_objidx = i;
            }

            j = inside_BoundingBox_KPTree (&query_object->verttree,
                                           &box, j);
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

