
#include "gui-indep.h"

    void
init_MotionInput (MotionInput* mot)
{
    uint i;
    mot->vert = 0;
    mot->horz = 0;
    mot->drift = 0;
    UFor( i, NDimensions )  mot->stride[i] = 0;
    UFor( i, 2 )  mot->thrust[i] = 0;
    mot->boost = false;
    mot->inv_vert = true;
    mot->use_roll = false;
    UFor( i, 2 )  mot->firing[i] = false;
    mot->lock_drift = false;
}

    void
init_RaceCraft (RaceCraft* craft)
{
    craft->health = 1;
}

    void
init_Pilot (Pilot* p)
{
    p->craft_idx = 0;
    init_MotionInput (&p->input);
    p->image_start_row = 0;
    p->image_start_col = 0;
    init_RayImage (&p->ray_image);
    p->stride_magnitude = 10;
    p->view_angle = 2 * M_PI / 3;
    p->view_width = 100;
    p->up_offset = 30;
    p->forward_offset = -130;
}

    void
cleanup_Pilot (Pilot* p)
{
    cleanup_RayImage (&p->ray_image);
}

    void
resize_pilot_viewports (uint nrows, uint ncols)
{
    uint i, start_row = 0;
    UFor( i, npilots )
    {
        Pilot* pilot;
        RayImage* ray_image;
        pilot = &pilots[i];
        ray_image = &pilot->ray_image;

        if (!ray_image->pixels) {
            ray_image->pixels = AllocT( byte, 1 );
                /* ray_image->hits = AllocT( uint, 1 ); */
                /* ray_image->mags = AllocT( real, 1 ); */
        }

            /* Horizontal splits.*/
        ray_image->nrows = (nrows - start_row) / (npilots - i);
        ray_image->ncols = ncols;
        resize_RayImage (ray_image);

        pilot->image_start_row = start_row;
        pilot->image_start_col = 0;
        start_row += ray_image->nrows;
    }
}

    /** Global effects!
     * A call to this better be in a mutex!
     * Not every function like this is labeled!
     **/
    void
update_object_motion (ObjectMotion* motion, const MotionInput* input)
{
    real x;

    x = input->vert;
    if (!input->inv_vert) x = - x;

    rotate_object (motion, UpDim, ForwardDim, x * (M_PI / 2));

    x = input->horz * (M_PI / 2);
    if (input->use_roll)
        rotate_object (motion, RightDim, UpDim, x);
    else
        rotate_object (motion, RightDim, ForwardDim, x);

    if (input->stride[ForwardDim] != 0)
    {
        motion->thrust[0] = input->stride[ForwardDim];
        motion->thrust[1] = input->stride[ForwardDim];
    }
    else
    {
        motion->thrust[0] = input->thrust[0];
        motion->thrust[1] = input->thrust[1];
    }
    motion->boost = input->boost;
    motion->lock_drift = input->lock_drift;
}

    void
update_camera_location (Pilot* pilot, const MotionInput* input, real dt)
{
    uint i;
    real x;
    PointXfrm tmp_basis;
    Point* view_origin;
    PointXfrm* view_basis;

    view_origin = &pilot->view_origin;
    view_basis = &pilot->view_basis;

    x = input->vert;
    if (!input->inv_vert) x = - x;

    copy_PointXfrm (&tmp_basis, view_basis);
        /* /x/ comes in as the vertical rotation.*/
    trrotate_PointXfrm (&tmp_basis, UpDim, ForwardDim, x * dt * (M_PI / 2));

    x = input->horz * dt * (M_PI / 2);
    if (input->use_roll)
        trrotate_PointXfrm (&tmp_basis, RightDim, UpDim, x);
    else
        trrotate_PointXfrm (&tmp_basis, RightDim, ForwardDim, x);

    orthonormalize_PointXfrm (view_basis, &tmp_basis);

    UFor( i, NDimensions )
    {
        Point diff;
        scale_Point (&diff, &view_basis->pts[i],
                     pilot->stride_magnitude * dt * input->stride[i]);
        summ_Point (view_origin, view_origin, &diff);
    }

    if (NDimensions == 4 && FollowRacer)
    {
        x = (1 + input->drift) / 2;
        x = clamp_real (x, 0, 1);
        view_origin->coords[DriftDim] = x;
    }
}

    void
update_view_params (Pilot* pilot,
                    const MotionInput* input,
                    const RaySpace* space)
{
    real view_zenith, view_azimuthcc;
    real vert_off, fwd_off;
    real dot;
    Point veloc;
    PointXfrm basis, rotation;
    PointXfrm* view_basis;
    const ObjectRaySpace* object;
    const ObjectMotion* motion;

    if (!FollowRacer)  return;

    object = &space->objects[pilot->craft_idx];
    motion = &racer_motions[pilot->craft_idx];
    view_basis = &pilot->view_basis;

    copy_PointXfrm (&basis, &object->orientation);

    copy_Point (&veloc, &motion->veloc);
    if (NDimensions == 4)
        veloc.coords[DriftDim] = 0;

    dot = dot_Point (&basis.pts[ForwardDim], &veloc);
    if (dot < 0)
    {
        negate_Point (&veloc, &veloc);
        reflect_Point (&veloc, &veloc, &basis.pts[ForwardDim], -dot);
    }

    Op_Point_2010( &basis.pts[ForwardDim]
                   ,+, &basis.pts[ForwardDim]
                   ,   .002*, &veloc );
    orthorotate_PointXfrm (view_basis, &basis, ForwardDim);
    copy_PointXfrm (&basis, view_basis);

    view_zenith    = (M_PI / 2) * (1 + input->view_zenith);
    view_azimuthcc =  M_PI      * (1 + input->view_azimuthcc);

    spherical3_PointXfrm (&rotation, view_zenith, view_azimuthcc - M_PI);

    xfrm_PointXfrm (view_basis, &rotation, &basis);

    vert_off = pilot->up_offset;
    fwd_off = pilot->forward_offset;

    Op_Point_2021010( &pilot->view_origin
                      ,+, &object->centroid
                      ,   +, fwd_off*,  &view_basis->pts[ForwardDim]
                      ,      vert_off*, &view_basis->pts[UpDim] );
}

    void
set_checkpoint_light (PointLightSource* light,
                      ObjectRaySpace* object,
                      const ObjectMotion* mot,
                      const Point* view_origin)
{
    uint i;
    real mag;
    real scale;
    Point direct, centroid, diff;
    Simplex elem;
    PointXfrm tmp, raw;
    const Plane* checkplane;
    const Point* checkpoint;
    
    checkplane = &checkplanes[mot->checkpoint_idx];
    checkpoint = &checkpoints[mot->checkpoint_idx];

    diff_Point (&direct, checkpoint, view_origin);
    mag = magnitude_Point (&direct);
    if (mag != 0)  scale_Point (&direct, &direct, 1/mag);

    scale = mag/2;

    Op_s( real, NColors, light->intensity , 0 );
    light->intensity[1] = 1;
    Op_Point_2010( &light->location
                   ,-, checkpoint
                   ,   scale*, &direct );


    assert (object->scene.nverts == NDimensions);

    identity_PointXfrm (&raw);
    copy_Point (&tmp.pts[0], &checkplane->normal);
    orthonormalize_PointXfrm (&raw, &tmp);

    scale_PointXfrm (&raw, &raw, scale);

    copy_Point (&elem.pts[0], checkpoint);
    copy_Point (&centroid, checkpoint);

    UFor( i, NDimensions-1)
    {
        summ_Point (&elem.pts[i+1], checkpoint, &raw.pts[i+1]);
        summ_Point (&centroid, &centroid, &elem.pts[i+1]);

        if (NDimensions == 4)
            elem.pts[i].coords[DriftDim] =
                view_origin->coords[DriftDim];
    }
    
    scale_Point (&centroid, &centroid, 1 / (real) NDimensions);
    diff_Point (&diff, checkpoint, &centroid);

    UFor( i, NDimensions )
        summ_Point (&elem.pts[i], &elem.pts[i], &diff);

    UFor( i, NDimensions )
        copy_Point (&object->scene.verts[i], &elem.pts[i]);
    update_trivial_ObjectRaySpace (object);
}

    void
relative_laser_origin (Point* origin, uint side,
                       const ObjectRaySpace* object)
{
    centroid_BoundingBox (origin, &object->box);
    origin->coords[ForwardDim] = object->box.max.coords[ForwardDim];
    if (side == 0)
        origin->coords[RightDim] = object->box.min.coords[RightDim];
    else
        origin->coords[RightDim] = object->box.max.coords[RightDim];
}

    void
setup_laser_scenes (RaySpace* space)
{
    uint i;
    fprintf (stderr, "setting up %u lasers\n", nracers*2);
    UFor( i, nracers )
    {
        uint side;
        UFor( side, 2 )
        {
            uint j;
            ObjectRaySpace* object;
            Point origin, destin;
            Material* mat;
            Point* verts;

            laser_objidcs[i][side] = space->nobjects;
            add_1elem_Scene_RaySpace (space);
            object = &space->objects[laser_objidcs[i][side]];

            mat = AllocT( Material, 1 );
            init_Material (mat);
            UFor( j, NColors )
            {
                mat->ambient[j] = 0;
                mat->diffuse[j] = 0;
                mat->specular[j] = 0;
            }
            mat->ambient[0] = .7;
            object->scene.nmatls = 1;
            object->scene.matls = mat;
            object->scene.elems[0].material = 0;

            relative_laser_origin (&origin, side, &space->objects[i]);
            copy_Point (&destin, &origin);
            destin.coords[ForwardDim] += 5000;

            verts = object->scene.verts;
            UFor( j, NDimensions )
            {
                copy_Point (&verts[j], &destin);
                if (j == ForwardDim)
                    verts[j].coords[j] = origin.coords[j];
                else if (j == RightDim && side == 0)
                    verts[j].coords[j] += 10;
                else if (j == RightDim && side == 1)
                    verts[j].coords[j] -= 10;
                else
                    verts[j].coords[j] += 10;
            }

            update_trivial_ObjectRaySpace (object);
            object->visible = false;
        }
    }
}

    void
update_health (const RaySpace* space, real dt)
{
    FILE* out = stderr;
    uint i;

    UFor( i, npilots )
    {
        const Pilot* pilot;
        const ObjectRaySpace* object;
        ObjectRaySpace* laser_objects[2];
        uint j;
        Point direct;

        pilot = &pilots[i];
        object = &space->objects[pilot->craft_idx];
        copy_Point (&direct, &object->orientation.pts[ForwardDim]);

        UFor( j, 2 )
        {
            uint objidx;
            objidx = laser_objidcs[pilot->craft_idx][j];
            laser_objects[j] = &space->objects[objidx];
        }

        UFor( j, 2 )
        {
            bool inside_box;
            uint hit_idx = Max_uint;
            real hit_mag = Max_real;
            uint hit_objidx = Max_uint;
            Point origin;

            laser_objects[j]->visible = pilot->input.firing[j];
            if (!laser_objects[j]->visible)  continue;

            copy_PointXfrm (&laser_objects[j]->orientation,
                            &object->orientation);
            copy_Point (&laser_objects[j]->centroid,
                        &object->centroid);

            relative_laser_origin (&origin, j, object);
            point_from_basis (&origin, &object->orientation,
                              &origin, &object->centroid);


            inside_box = inside_BoundingBox (&space->main.box, &origin);
            cast_nopartition (&hit_idx, &hit_mag, &hit_objidx,
                              space, &origin, &direct,
                              inside_box, pilot->craft_idx);
            if (hit_objidx < nracers)
            {
                RaceCraft* craft;
                craft = &crafts[hit_objidx];
                craft->health -= dt * (1 - pilot->input.thrust[j]);
                fprintf (out, "%u zaps %u, health at %f!!!\n",
                         pilot->craft_idx, hit_objidx, craft->health);

                if (craft->health <= 0)
                {
                    fprintf (out, "%u is DEAAAAAAAAAAAAAD\n", hit_objidx);
                    craft->health = 1;
                }
            }
        }
    }
}

static
    void
set_argb_shifts (byte* shifts, bool argb_order)
{
    if (argb_order)
    { shifts[0] = 24;  shifts[1] = 16;  shifts[2] =  8;  shifts[3] =  0; }
    else
    { shifts[0] =  0;  shifts[1] =  8;  shifts[2] = 16;  shifts[3] = 24; }
}

    void
render_pattern (byte* data, uint nrows, uint ncols, uint stride,
                bool argb_order)
{
    uint row;
    byte shifts[4];

    set_argb_shifts (shifts, argb_order);

    UFor( row, nrows )
    {
        uint col;
        uint32* line;
        line = (uint32*) &(data)[row * stride];
        UFor( col, ncols )
        {
            uint32 v;
            byte r, g, b;
            r = (byte) ((~col |  row) / 2);
            g = (byte) (( col | ~row) / 2);
            b = (byte) ((~col | ~row) / 2);

            v = ((uint32) 0xFF << shifts[0]) |
                ((uint32) r    << shifts[1]) |
                ((uint32) g    << shifts[2]) |
                ((uint32) b    << shifts[3]);

                /* v  = (uint32)(0x7FFFFF*(1+sin(~(x^y) % ~(x|y)))); */
                /* v |= 0xFF000000; */

            line[col] = v;
        }
    }
}

    void
render_RayImage (byte* data, uint nrows, uint ncols, uint stride,
                 const RayImage* ray_image,
                 uint image_start_row,
                 uint image_start_col,
                 bool argb_order)
{
    uint ray_row, start_row, start_col;
    byte shifts[4];

    set_argb_shifts (shifts, argb_order);

    assert (ray_image->pixels);
    if (ray_image->nrows == 0)  return;
    start_row = image_start_row + npixelzoom * ray_image->nrows - 1;
    start_col = image_start_col;

    UFor( ray_row, npixelzoom * ray_image->nrows )
    {
        uint image_row, ray_col;
        const byte* pixline;
        uint32* outline;

        image_row = start_row - ray_row;
        if (image_row >= nrows)  continue;
        pixline = &ray_image->pixels[((ray_row / npixelzoom) *
                                      3 * ray_image->stride)];
        outline = (uint32*) &data[image_row * stride];

        UFor( ray_col, npixelzoom * ray_image->ncols )
        {
            uint image_col;
            image_col = start_col + ray_col;
            if (image_col >= ncols)  break;
            outline[image_col] =
                ((uint32) 0xFF                                << shifts[0]) |
                ((uint32) pixline[3*(image_col/npixelzoom)+0] << shifts[1]) |
                ((uint32) pixline[3*(image_col/npixelzoom)+1] << shifts[2]) |
                ((uint32) pixline[3*(image_col/npixelzoom)+2] << shifts[3]);
        }
    }
}

    void
render_pilot_images (byte* data, uint nrows, uint ncols, uint stride,
                     bool argb_order)
{
    uint pilot_idx;
    UFor( pilot_idx, npilots )
    {
        Pilot* pilot;
        pilot = &pilots[pilot_idx];

        render_RayImage (data, nrows, ncols, stride,
                         &pilot->ray_image,
                         pilot->image_start_row * npixelzoom,
                         pilot->image_start_col * npixelzoom,
                         argb_order);
    }
}

    void
update_pilot_images (RaySpace* space, real frame_t1)
{
    uint pilot_idx;
    FILE* out = stderr;

    if (frame_t0 == 0)
    {
        frame_t0 = frame_t1;
    }
    else
    {
        real dt;
        dt = frame_t1 - frame_t0;
        frame_t0 = frame_t1;

        if (ShowFrameRate)
        {
            framerate_report_dt += dt;
            framerate_report_count += 1;
            if (framerate_report_dt >= 1)
            {
                real fps;
                fps = framerate_report_count / framerate_report_dt;
                fprintf (out, "FPS:%f\n", fps);
                framerate_report_dt = 0;
                framerate_report_count = 0;
            }
        }

        UFor( pilot_idx, npilots )
        {
            uint craft_idx;
            Pilot* pilot;

            pilot = &pilots[pilot_idx];
            craft_idx = pilot->craft_idx;

            if (FollowRacer)
                update_object_motion (&racer_motions[craft_idx],
                                      &pilot->input);
            else
                update_camera_location (pilot, &pilot->input, dt);
        }
        if (FollowRacer)
        {
            uint nobjects = space->nobjects;
                /* TODO: This is essentially a hack to remove
                 * non-racecraft objects from the the ray space
                 * while doing collision tests.
                 */
            space->nobjects = nracers;
            move_objects (space, racer_motions, dt,
                          ncheckplanes, checkplanes);
            update_health (space, dt);
            space->nobjects = nobjects;
        }
    }

    if (FollowRacer && ShowSpeed)
    {
        uint craft_idx;
        craft_idx = pilots[kbd_pilot_idx].craft_idx;
        fprintf (out, "SPEED:%f\n",
                 magnitude_Point (&racer_motions[craft_idx].veloc));
    }

    UFor( pilot_idx, npilots )
    {
        PointXfrm basis;
        Point origin;
        Pilot* pilot;
        RayImage* ray_image;

        pilot = &pilots[pilot_idx];
        ray_image = &pilot->ray_image;


        update_view_params (pilot, &pilot->input, space);
        copy_Point (&origin, &pilot->view_origin);
        copy_PointXfrm (&basis, &pilot->view_basis);

        if (ray_image->perspective)  ray_image->hifov = pilot->view_angle;
        else                         ray_image->hifov = pilot->view_width;

#ifdef DistribCompute
        compute_rays_to_hits (ray_image, space, &origin, &basis);
#else
        if (LightAtCamera)
        {
            assert (space->nlights > 0);
            copy_Point (&space->lights[0].location, &origin);
            Op_s( real, NColors, space->lights[0].intensity , .5 );
        }
        if (ncheckplanes > 0)
        {
            set_checkpoint_light (&space->lights[space->nlights-1],
                                  &space->objects[checkplane_objidx],
                                  &racer_motions[pilot->craft_idx],
                                  &pilot->view_origin);
        }
        update_dynamic_RaySpace (space);
        if (ForceFauxFishEye)
            rays_to_hits_fish (ray_image, space,
                               &origin, &basis, pilot->view_angle);
        else
            cast_RayImage (ray_image, space, &origin, &basis);
#endif
    }
}

    void
init_ui_data (RaySpace* space, const char* inpathname)
{
    uint i;
    if (FollowRacer)
    {
        assert (space->nobjects == 0 && "All objects must be racers.");
        nracers = npilots;
        assert (nracers <= NRacersMax);
        add_racers (space, nracers, inpathname);
        UFor( i, nracers )
            init_ObjectMotion (&racer_motions[i], &space->objects[i]);
    }

    if (ncheckplanes > 0)
    {
        checkplane_objidx = space->nobjects;
        add_1elem_Scene_RaySpace (space);
    }

    setup_laser_scenes (space);
}

    void
cleanup_ui_data ()
{
    uint i;
    UFor( i, NRacersMax )
        cleanup_Pilot (&pilots[i]);
}

