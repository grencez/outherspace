
#ifndef __OPENCL_VERSION__
#include "motion.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

static void
zero_rotations (ObjectMotion* motion);
static void
move_object (RaySpace* space, ObjectMotion* motion, uint objidx, real dt);
static bool
detect_collision (const RaySpace* space,
                  uint objidx,
                  const Point* new_centroid,
                  const PointXfrm* new_orientation);

    void
init_ObjectMotion (ObjectMotion* motion)
{
    uint i;
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
    assert (xdim < ydim);
    motion->rots[xdim * (NDimensions - 2) + ydim - 1] += angle;
}

    void
move_objects (RaySpace* space, ObjectMotion* motions, real dt)
{
    const uint nincs = 10;
    RaySpaceObject* objects;
    real inc;
    uint i, nobjects;
    nobjects = space->nobjects;
    objects = space->objects;

    inc = dt / nincs;

    UFor( i, nincs )
    {
        uint j;
        UFor( j, nobjects )
            move_object (space, &motions[j], j, inc);
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
move_object (RaySpace* space, ObjectMotion* motion, uint objidx, real dt)
{
    uint i;
    bool commit_move = true;
    PointXfrm basis, new_orientation;
    Point new_centroid;
    RaySpaceObject* object;
    Point dir, veloc;

    object = &space->objects[objidx];

        /* Rotate object.*/
    dir.coords[DirDimension] = 1;
    UFor( i, NDimensions - 1 )
    {
        uint idx;
        real angle;
        idx = i * (NDimensions - 2) + DirDimension - 1;
        angle = - motion->rots[idx] * dt;
        assert (angle < + M_PI / 2);
        assert (angle > - M_PI / 2);
            /* Find the directional components.*/
        dir.coords[i] = tan (angle);
    }
        /* Rotate relative to the direction.*/
    copy_PointXfrm (&basis, &object->orientation);
    trxfrm_Point (&basis.pts[DirDimension], &object->orientation, &dir);
    orthorotate_PointXfrm (&new_orientation, &basis, DirDimension);

        /* Add in various other rotations.*/
    UFor( i, NDimensions - 1 )
    {
        uint j;
        UFor( j, i )
        {
            uint idx;
            real angle;
            PointXfrm rotation;
            idx = j * (NDimensions - 2) + i - 1;
            angle = motion->rots[idx] * dt;
            assert (angle < + M_PI / 2);
            assert (angle > - M_PI / 2);
            rotation_PointXfrm (&rotation, j, i, angle);

            copy_PointXfrm (&basis, &new_orientation);
            transpose_PointXfrm (&rotation, &rotation);
            xfrm_PointXfrm (&new_orientation, &rotation, &basis);
        }
    }

        /* Assume full grip causes orthogonal motion to be lost.*/
    proj_Point (&veloc, &motion->veloc, &new_orientation.pts[DirDimension]);

    if (motion->thrust != 0)
    {
        const real alpha = 5;  /* Arbitrary coefficient.*/
        const real vthrust = 733;  /* Velocity of thrusters.*/
        real accel, mag;
        Point tmp;

        mag = magnitude_Point (&veloc);
        accel = alpha * (vthrust - mag);
        if (motion->thrust < 0)  accel = - accel;

        scale_Point (&tmp, &new_orientation.pts[DirDimension], accel * dt);
        summ_Point (&veloc, &veloc, &tmp);
    }
    else
    {
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
        hit = detect_collision (space, objidx,
                                &new_centroid,
                                &new_orientation);
        if (hit)
        {
            commit_move = false;
            negate_Point (&motion->veloc, &motion->veloc);
        }
    }

    if (commit_move)
    {
        copy_Point (&object->centroid, &new_centroid);
        copy_PointXfrm (&object->orientation, &new_orientation);
    }
}


    bool
detect_collision (const RaySpace* space,
                  uint objidx,
                  const Point* new_centroid,
                  const PointXfrm* new_orientation)
{
    uint i;
    uint hit_idx;
    real hit_mag;
    const RaySpace* hit_space;
    const RaySpaceObject* object;
    const Scene* scene;

    object = &space->objects[objidx];
    scene = &object->space.scene;

    hit_idx = space->nelems;
    hit_mag = Max_real;
    hit_space = space;

    UFor( i, scene->nverts )
    {
        Point diff, origin, destin;
        real distance;

        centroid_BoundingBox (&diff, &object->space.box);
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
            Point tmp_point;
            const RaySpace* tmp_space;
            bool inside_box;
            Point unit_dir;

            scale_Point (&unit_dir, &diff, 1 / distance);
            inside_box = inside_BoundingBox (&space->box, &origin);

            cast_recurse (&tmp_hit, &tmp_mag, &tmp_space,
                          &tmp_point, &tmp_point,
                          space, &origin, &unit_dir,
                          inside_box, objidx);
            if (tmp_mag < distance &&
                tmp_mag < hit_mag)
            {
                hit_idx = tmp_hit;
                hit_mag = tmp_mag;
                hit_space = tmp_space;
            }
        }

    }

    return hit_idx < hit_space->nelems;
        /* Below here is scratch from the prototype!*/
#if 0
    if (hit_idx < hit_space->nelems)
    {
        Point tdir;
        const Plane* plane;
        plane = &hit_space->simplices[hit_idx].plane;

        proj_Plane (&motion->veloc, &motion->veloc, plane);
            /* diff_Point (&tdir, &motion->veloc, &dir); */
            /* summ_Point (&motion->veloc, &motion->veloc, &tdir); */

        scale_Point (&dir, &motion->veloc,
                     dt - dt * hit_mag / magnitude_Point (&dir));

#if 1
            /*
               scale_Point (&dir, &dir, -1);
               scale_Point (&tdir, &tdir, 2);
               summ_Point (&dir, &dir, &tdir);
               scale_Point (&motion->veloc, &dir, -1);
             */
            /* zero_Point (&dir); */
#else
        zero_Point (&motion->veloc);
#endif
            /* scale_Point (&tdir, &unit_dir, hit_mag*.9); */
    }
#endif
}

