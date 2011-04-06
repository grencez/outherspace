
#ifndef __OPENCL_VERSION__
#include "motion.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

    void
init_ObjectMotion (ObjectMotion* motion)
{
    uint i;
    zero_Point (&motion->veloc);
    motion->accel = 0;
    UFor( i, N2DimRotations )
        motion->rots[i] = 0;
}

    void
rotate_object (ObjectMotion* motion, uint xdim, uint ydim, real angle)
{
    assert (xdim < ydim);
    motion->rots[xdim * (NDimensions - 2) + ydim - 1] += angle;
}

static
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
move_object (RaySpaceObject* object, ObjectMotion* motion,
             real dt, bool preserve)
{
    uint i;
    PointXfrm basis;
    Point dir;

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
    orthorotate_PointXfrm (&object->orientation, &basis, DirDimension);

        /* Add in various other rotations.*/
    UFor( i, NDimensions - 1 )
    {
        uint j;
        UFor( j, i )
        {
            uint idx;
            real angle;
            PointXfrm rotation, tmp;
            idx = j * (NDimensions - 2) + i - 1;
            angle = motion->rots[idx] * dt;
            assert (angle < + M_PI / 2);
            assert (angle > - M_PI / 2);
            rotation_PointXfrm (&rotation, j, i, angle);

            copy_PointXfrm (&tmp, &object->orientation);
            transpose_PointXfrm (&rotation, &rotation);
            xfrm_PointXfrm (&object->orientation, &rotation, &tmp);
        }
    }

    if (!preserve)  zero_rotations (motion);

    scale_Point (&dir,
                 &object->orientation.pts[DirDimension],
                 motion->accel * dt);
    summ_Point (&motion->veloc, &motion->veloc, &dir);
    scale_Point (&dir, &motion->veloc, dt);
    summ_Point (&object->centroid, &object->centroid, &dir);

    if (motion->accel == 0)
    {
        real mag, x;
        x = 5 * dt;
        mag = magnitude_Point (&motion->veloc);
        
        if (mag > x)  scale_Point (&motion->veloc, &motion->veloc,
                                   (mag - x) / mag);
        else          zero_Point (&motion->veloc);
    }
    if (!preserve)  motion->accel = 0;
}

