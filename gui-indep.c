
#include "gui-indep.h"

#include "affine.h"
#include "bbox.h"
#include "color.h"
#include "raytrace.h"
#include "simplex.h"
#include "space-junk.h"
#include "testcase.h"
#include "xfrm.h"

#include <assert.h>

static Pilot pilots[NRacersMax];

#ifdef SupportOpenGL
static bool DisplayRayImage = false;
#else
static const bool DisplayRayImage = true;
#endif

    void
init_MotionInput (MotionInput* mot)
{
    uint i;
    mot->vert = 0;
    mot->horz = 0;
    mot->drift = 0;
    UFor( i, NDimensions )  mot->stride[i] = 0;
    UFor( i, 2 )  mot->thrust[i] = 0;
    mot->view_azimuthcc = 0;
    mot->view_zenith = 0;
    mot->light_to_camera = false;
    mot->boost = false;
    mot->inv_vert = true;
    mot->use_roll = false;
    UFor( i, 2 )  mot->firing[i] = false;
    mot->lock_drift = false;
    mot->mouse_orbit = mot->mouse_zoom = mot->mouse_pan = false;
    mot->zoom = 0;
    UFor( i, 2 )
    {
        mot->pan[i] = 0;
        mot->orbit[i] = 0;
    }
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
    zero_Point (&p->view_origin);
    identity_PointXfrm (&p->view_basis);
    p->image_start_row = 0;
    p->image_start_col = 0;
    init_RayImage (&p->ray_image);
    p->stride_magnitude = 10;
    p->view_angle = 2 * atan (1.0 / 3);
    p->view_width = 100;
    p->up_offset = 90;
    p->forward_offset = -380;
    zero_Point (&p->orbit_focus);
    p->mouse_down = false;
}

    void
cleanup_Pilot (Pilot* p)
{
    cleanup_RayImage (&p->ray_image);
}

    void
sync_Pilot (Pilot* gui, Pilot* bkg)
{
    bkg->craft_idx = gui->craft_idx;

    if (FollowRacer ||
        bkg->input.pan[0] != 0 ||
        bkg->input.pan[1] != 0 ||
        bkg->input.zoom != 0 ||
        bkg->input.orbit[0] != 0 ||
        bkg->input.orbit[1] != 0)
    {
        BSfx( gui,=,bkg, ->view_origin );
        BSfx( gui,=,bkg, ->view_basis );
    }
    else
    {
        BSfx( bkg,=,gui, ->view_origin );
        BSfx( bkg,=,gui, ->view_basis );
    }

    BSfx( bkg,=,gui, ->input );

    gui->input.light_to_camera = false;

    { BLoop( i, 2 )
        gui->input.orbit[i] = 0;
        gui->input.pan[i] = 0;
    } BLose()
    gui->input.zoom = 0;

    BSfx( gui,=,bkg, ->image_start_row );
    BSfx( gui,=,bkg, ->image_start_col );

    {
        RayImage* g = &gui->ray_image;
        RayImage* b = &bkg->ray_image;
        BSfx( b,=,g, ->perspective );
        BSfx( b,=,g, ->view_light );
        *g = *b;
    }

    BSfx( bkg,=,gui, ->stride_magnitude );
    BSfx( bkg,=,gui, ->view_angle );
    BSfx( bkg,=,gui, ->view_width );

    BSfx( bkg,=,gui, ->up_offset );
    BSfx( bkg,=,gui, ->forward_offset );

    BSfx( bkg,=,gui, ->orbit_focus );
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

    /** Width (and height) one screen unit represents
     * at a certain distance /d/ away.
     **/
static real
screen_mag(const Pilot* p, real d)
{
    if (p->ray_image.perspective)
        return 2 * d * tan(.5 * p->view_angle);
    else
        return p->ray_image.hifov;
}

    /** When not following the racer,
     * this updates camera location and basis.
     **/
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

    if (input->pan[0] != 0 || input->pan[1] != 0)
    {
        Point p;
        real d, h;
        real diff[2];
        Plane w;

        init_Plane (&w, &view_basis->pts[FwDim], view_origin);
        d = dist_Plane (&w, &pilot->orbit_focus);

        h = screen_mag (pilot, d);

        UFor( i, 2 )  diff[i] = input->pan[i] * (- h);

        Op_Point_21010( &p
                        ,+, diff[0]*, &view_basis->pts[UpDim]
                        ,   diff[1]*, &view_basis->pts[RightDim] );
        summ_Point (view_origin, view_origin, &p);
    }
    if (input->zoom != 0)
    {
        real d, m;
        Plane w;

        init_Plane (&w, &view_basis->pts[FwDim], view_origin);
        d = dist_Plane (&w, &pilot->orbit_focus);
        m = d * (- 4 * input->zoom);
        follow_Point (view_origin, view_origin, &view_basis->pts[FwDim], m);
    }
    if (input->orbit[0] != 0 || input->orbit[1] != 0)
    {
        real rev;
        PointXfrm rot, tilt, B;
        IAMap map;
        PointXfrm xfrm;
        Point p;

        rev = sqrt (+ input->orbit[0] * input->orbit[0]
                    + input->orbit[1] * input->orbit[1]);
        rotn_PointXfrm (&rot, UpDim, FwDim, -rev);
        rotation_PointXfrm (&tilt, UpDim, RightDim,
                            - atan2_real (input->orbit[1], input->orbit[0]));

        identity_IAMap (&map);
        transpose_PointXfrm (&map.xfrm, view_basis);
        map.xlat = *view_origin;

        xfrmtr_PointXfrm (&B, &tilt, &map.xfrm);

        negate_Point (&p, &pilot->orbit_focus);
        xlat_IAMap (&map, &p, &map);

        trxfrm_PointXfrm (&xfrm, &B, &rot);
        xfrm_PointXfrm (&xfrm, &xfrm, &B);
        xfrm_IAMap (&map, &xfrm, &map);

        xlat_IAMap (&map, &pilot->orbit_focus, &map);

        orthonormalize_PointXfrm (&xfrm, &map.xfrm);
        transpose_PointXfrm (view_basis, &xfrm);
        *view_origin = map.xlat;

        /*
        fprintf (stderr, "orbit: %f %f\n",
                 input->orbit[0], input->orbit[1]);
                 */
            /* output_Point (stderr, &pilot->orbit_focus); */
            /* fputc ('\n', stderr); */
    }
    
        /* TODO: This is never true because of /FollowRacer/.*/
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
    Point veloc, fwd;
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

    Op_Point_2010( &fwd
                   ,+, &basis.pts[ForwardDim]
                   ,   .002*, &veloc );
    orthorotate_PointXfrm (&basis, &basis, &fwd, ForwardDim);

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
    PointXfrm raw;
    const Plane* checkplane;
    const Point* checkpoint;
    
    light->on = true;
    checkplane = &track.checkplanes[mot->checkpoint_idx];
    checkpoint = &track.checkpoints[mot->checkpoint_idx];

    diff_Point (&direct, checkpoint, view_origin);
    mag = magnitude_Point (&direct);
    if (mag != 0)  scale_Point (&direct, &direct, 1/mag);

    scale = mag/2;

    zero_Color (&light->intensity);
    light->intensity.coords[1] = 1;
    follow_Point (&light->location, checkpoint, &direct, -scale);

    assert (object->scene.nverts == NDimensions);

    identity_PointXfrm (&raw);
    stable_orthorotate_PointXfrm (&raw, &raw,
                                  &checkplane->normal, 0);
        /*^ /0/ is arbitrary, I believe.^*/

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
    centroid_BBox (origin, &object->box);
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
            zero_Color (&mat->ambient);
            zero_Color (&mat->diffuse);
            zero_Color (&mat->specular);
            mat->ambient.coords[0] = .7;
            object->scene.nmatls = 1;
            object->scene.matls = mat;
            object->scene.elems[0].material = 0;
            object->scene.surfs[0].material = 0;

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


            inside_box = inside_BBox (&space->main.box, &origin);
            cast_nopartition (&hit_idx, &hit_mag, &hit_objidx,
                              space, &origin, &direct,
                              inside_box, May, pilot->craft_idx);
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

static inline
    uint
argb_nbytes (uint32 argb_map)
{
    uint npp = 0;
        /* Count the number of bytes we should use! */
    { BLoop( i, 4 )
        npp += ((0xF & (argb_map >> (4 * i))) < 4) ? 1 : 0;
    } BLose()
    return npp;
}

static inline
    void
set_argb_pixel (byte* dst, uint nbytes, uint32 argb_map,
                byte a, byte r, byte g, byte b)
{
    byte tmp[16];
    tmp[(0xF000 & argb_map) >> 12] = a;
    tmp[(0x0F00 & argb_map) >>  8] = r;
    tmp[(0x00F0 & argb_map) >>  4] = g;
    tmp[(0x000F & argb_map) >>  0] = b;
    { BLoop( i, nbytes )
        dst[i] = tmp[i];
    } BLose()
}

    void
render_pattern (byte* data, uint nrows, uint ncols, uint stride,
                uint32 argb_map)
{
    uint npp = argb_nbytes (argb_map);

    { BLoop( row, nrows )
        byte* line;
        line = &(data)[row * stride];
        { BLoop( col, ncols )
            byte r, g, b;
            r = (byte) ((~col |  row) / 2);
            g = (byte) (( col | ~row) / 2);
            b = (byte) ((~col | ~row) / 2);

            set_argb_pixel (&line[npp * col], npp, argb_map, 0xFF, r, g, b);

                /* v  = (uint32)(0x7FFFFF*(1+sin(~(x^y) % ~(x|y)))); */
                /* v |= 0xFF000000; */
        } BLose()
    } BLose()
}

    void
render_RayImage (byte* data, uint nrows, uint ncols, uint stride,
                 const RayImage* ray_image,
                 uint image_start_row,
                 uint image_start_col,
                 uint32 argb_map)
{
    uint start_row, start_col;
    uint npp = argb_nbytes (argb_map);

    assert (ray_image->pixels);
    if (ray_image->nrows == 0)  return;
    start_row = image_start_row + npixelzoom * ray_image->nrows - 1;
    start_col = image_start_col;

    { BLoop( ray_row, npixelzoom * ray_image->nrows )
        uint image_row;
        const byte* pixline;
        byte* outline;

        image_row = start_row - ray_row;
        if (image_row >= nrows)  continue;
        pixline = &ray_image->pixels[((ray_row / npixelzoom) *
                                      3 * ray_image->stride)];
        outline = &data[image_row * stride];

        { BLoop( ray_col, npixelzoom * ray_image->ncols )
            uint image_col;
            image_col = start_col + ray_col;
            if (image_col >= ncols)  break;
            set_argb_pixel (&outline[npp * image_col], npp, argb_map,
                            0xFF,
                            pixline[3*(image_col/npixelzoom)+0],
                            pixline[3*(image_col/npixelzoom)+1],
                            pixline[3*(image_col/npixelzoom)+2]);
        } BLose()
    } BLose()
}

    void
render_pilot_images (byte* data, uint nrows, uint ncols, uint stride,
                     bool argb_order)
{
    { BLoop( pilot_idx, npilots )
        Pilot* pilot;
        pilot = &pilots[pilot_idx];

        render_RayImage (data, nrows, ncols, stride,
                         &pilot->ray_image,
                         pilot->image_start_row * npixelzoom,
                         pilot->image_start_col * npixelzoom,
                         argb_order);
    } BLose()
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
                          track.ncheckplanes, track.checkplanes);
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
        origin = pilot->view_origin;
        basis = pilot->view_basis;

        if (pilot->input.light_to_camera && space->nlights > 0)
            space->lights[0].location = origin;

        if (ray_image->perspective)  ray_image->hifov = pilot->view_angle;
        else                         ray_image->hifov = pilot->view_width;

#ifdef DistribCompute
        compute_rays_to_hits (ray_image, space, &origin, &basis);
#else
        if (LightAtCamera)
        {
            assert (space->nlights > 0);
            space->lights[0].location = origin;
            set_Color (&space->lights[0].intensity, .5);
        }
        if (FollowRacer && track.ncheckplanes > 0)
        {
            set_checkpoint_light (&space->lights[space->nlights-1],
                                  &space->objects[checkplane_objidx],
                                  &racer_motions[pilot->craft_idx],
                                  &pilot->view_origin);
        }
        if (DisplayRayImage)
        {
            if (space->nobjects > 0)
                update_dynamic_RaySpace (space);
            if (ForceFauxFishEye)
                rays_to_hits_fish (ray_image, space,
                                   &origin, &basis, pilot->view_angle);
            else
                cast_RayImage (ray_image, space, &origin, &basis);
        }
#ifdef SupportOpenGL
        else
        {
            ogl_redraw (space, pilot_idx);
        }
#endif
#endif
    }
}

    void
init_ui_data (RaySpace* space,
              const Pilot* input_pilots,
              const char* inpathname)
{
    uint i;

    UFor( i, NRacersMax )
        pilots[i] = input_pilots[i];

    if (FollowRacer)
    {
        assert (space->nobjects == 0 && "All objects must be racers.");
        nracers = npilots;
        assert (nracers <= NRacersMax);
        add_racers (space, nracers, &track, inpathname);
        UFor( i, nracers )
            init_ObjectMotion (&racer_motions[i], &space->objects[i]);
    }

    if (FollowRacer && track.ncheckplanes > 0)
    {
        Scene* scene;
        Material* matl;
        checkplane_objidx = space->nobjects;
        add_1elem_Scene_RaySpace (space);

        scene = &space->objects[checkplane_objidx].scene;
        scene->nmatls = 1;
        scene->matls = AllocT( Material, scene->nmatls );
        matl = &scene->matls[0];
        scene->elems[0].material = 0;
        scene->surfs[0].material = 0;

        init_Material (matl);
        matl->diffuse.coords[0] = .8824;
        matl->diffuse.coords[1] = .4824;
        matl->diffuse.coords[2] = .0510;
        matl->ambient = matl->diffuse;
        zero_Color (&matl->specular);
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

