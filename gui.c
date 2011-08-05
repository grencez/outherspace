
#include "pnm-image.h"
#include "motion.h"
#include "testcase.h"

#ifdef DistribCompute
#include "compute.h"
#endif

#include <math.h>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <SDL.h>
#include <SDL_mixer.h>

    /* SDL on OS X does some weirdo bootstrapping by redefining /main/.*/
#ifdef main
#undef main
#endif

#define NRacersMax 10

static const bool FollowRacer = true;

static const bool RenderDrawsPattern = false;
static const bool ForceFauxFishEye = false;
static const bool ShowFrameRate = false;
static const bool LightAtCamera = false;
static const bool ShowSpeed = false;

static const bool
#ifdef RunFromMyMac
#undef RunFromMyMac
RunFromMyMac = true;
#else
RunFromMyMac = false;
#endif

static uint racer_idx = 0;
static uint nracers = 0;
static ObjectMotion racer_motions[NRacersMax];  /* Reset every frame.*/

typedef struct motion_input_struct MotionInput;

struct motion_input_struct
{
    real t0;
    real t1;
    real vert;
    real horz;
    real drift;
    tristate stride[NDimensions];
    real view_azimuthcc;
    real view_zenith;
    bool boost;
    bool inv_vert;
    bool use_roll;
};

static real stride_magnitude = 10;
static Point view_origin;
static PointXfrm view_basis;
static uint view_nrows = 400;
static uint view_ncols = 400;
static uint mouse_coords[2];
static uint mouse_diff[2];
static RayImage ray_image;
static real view_angle = 2 * M_PI / 3;
static real view_width = 100;
static bool needs_recast = true;
static bool continue_running = true;
static MotionInput motion_input;
static real framerate_report_dt = 0;
static uint framerate_report_count = 0;


static
    void
init_MotionInput (MotionInput* mot)
{
    uint i;
    mot->t0 = 0;
    mot->t1 = 0;
    mot->vert = 0;
    mot->horz = 0;
    mot->drift = 0;
    UFor( i, NDimensions )  mot->stride[i] = 0;
    mot->boost = false;
    mot->inv_vert = true;
    mot->use_roll = false;
}

static gboolean delete_event (GtkWidget* widget,
                              GdkEvent* event,
                              gpointer data)
{
    (void) widget;
    (void) event;
    (void) data;
    return FALSE;
}

    /* Another callback */
static void destroy_app (GtkWidget* widget,
                         gpointer data)
{
    (void) widget;
    (void) data;
    cleanup_RayImage (&ray_image);
    continue_running = false;
    gtk_main_quit ();
}

static
    gboolean
key_press_fn (GtkWidget* widget, GdkEventKey* event, gpointer data)
{
    uint dim = NDimensions;
    tristate roll = 0;
    tristate stride = 0;
    tristate turn = 0;
    tristate view_angle_change = 0;
    tristate resize = 0;
    tristate view_light_change = 0;
    tristate stride_mag_change = 0;
    tristate switch_racer = 0;
    bool reflect = false;
    bool rotate_dir_dim = false;
    bool change_cast_method = false;
    bool print_view_code = false;
    bool recast = true;
    FILE* out;
    bool shift_mod, ctrl_mod;
    MotionInput* input;
    ObjectMotion* racer_motion;
    racer_motion = &racer_motions[racer_idx];

    input = (MotionInput*) data;
    out = stdout;

    shift_mod = event->state & GDK_SHIFT_MASK;
    ctrl_mod = event->state & GDK_CONTROL_MASK;

        /* /usr/include/gtk-2.0/gdk/gdkkeysyms.h */
        /* printf ("%x\n", event->keyval); */
    switch (event->keyval)
    {
        case GDK_Up:
            if (shift_mod) { dim = UpDim;  stride = 1; }
            else if (FollowRacer) { dim = UpDim;  turn = 1; }
            else if (ctrl_mod) { dim = UpDim;  turn = 1; }
            else { dim = ForwardDim;  stride = 1; }
            break;
        case GDK_Down:
            if (shift_mod) { dim = UpDim;  stride = -1; }
            else if (FollowRacer) { dim = UpDim;  turn = -1; }
            else if (ctrl_mod) { dim = UpDim;  turn = -1; }
            else { dim = ForwardDim;  stride = -1; }
            break;
        case GDK_Right:
            if (shift_mod) { dim = RightDim;  stride = 1; }
            else if (ctrl_mod) { roll = -1; }
            else if (FollowRacer) { dim = RightDim;  turn = 1; }
            else { roll = -1; }
            break;
        case GDK_Left:
            if (shift_mod) { dim = RightDim;  stride = -1; }
            else if (ctrl_mod) { roll = 1; }
            else if (FollowRacer) { dim = RightDim;  turn = -1; }
            else { roll = 1; }
            break;
        case GDK_Tab:
            switch_racer =  1; break;
        case GDK_ISO_Left_Tab:
            switch_racer = -1; break;
        case GDK_Escape:
            gdk_pointer_ungrab (event->time);  break;
        case GDK_a:
            dim = ForwardDim; stride = 1; break;
        case GDK_c:
            racer_motion->collide = !racer_motion->collide;
            break;
        case GDK_D:
            if (NDimensions == 4) { dim = DriftDim; stride = 1; }
            break;
        case GDK_d:
            if (ctrl_mod) { rotate_dir_dim = true; }
            else if (NDimensions == 4) { dim = DriftDim; stride = -1; }
            break;
        case GDK_e:
            input->boost = true;
            break;
        case GDK_g:
            racer_motion->gravity = !racer_motion->gravity;
            break;
        case GDK_L:
            view_light_change = 1;  break;
        case GDK_l:
            view_light_change = -1;  break;
        case GDK_P:
            print_view_code = true;  break;
        case GDK_r:
            reflect = true;  break;
        case GDK_S:
            stride_mag_change = 1; break;
        case GDK_s:
            stride_mag_change = -1; break;
        case GDK_V:
            view_angle_change = 1;  break;
        case GDK_v:
            if (ctrl_mod)  change_cast_method = true;
            else  view_angle_change = -1;
            break;
        case GDK_Z:
            resize = 1;  break;
        case GDK_z:
            resize = -1;  break;
        default:
            break;
    }

    if (view_light_change)
    {
        real diff = stride_magnitude;
        if (view_light_change > 0)
            ray_image.view_light += diff;
        else if (diff <= ray_image.view_light)
            ray_image.view_light -= diff;;

        fprintf (out, "view_light:%f\n", ray_image.view_light);
    }
    else if (stride != 0 && FollowRacer)
    {
        input->stride[dim] = stride;
    }
    else if (stride != 0 && !FollowRacer)
    {
        Point diff;
        scale_Point (&diff, &view_basis.pts[dim], stride_magnitude * stride);
        summ_Point (&view_origin, &view_origin, &diff);

        fputs ("pos:", out);
        output_Point (out, &view_origin);
        fputc ('\n', out);
    }
    else if (stride_mag_change != 0)
    {
        if (stride_mag_change > 0)  stride_magnitude *= 2;
        else                        stride_magnitude /= 2;
        fprintf (out, "stride_magnitude: %f\n", stride_magnitude);
        recast = false;
    }
    else if (roll != 0)
    {
        PointXfrm tmp_basis;
        copy_PointXfrm (&tmp_basis, &view_basis);
        rotate_PointXfrm (&tmp_basis, UpDim, RightDim, roll * M_PI / 8);
        orthonormalize_PointXfrm (&view_basis, &tmp_basis);

        fputs ("basis:", out);
        output_PointXfrm (out, &view_basis);
        fputc ('\n', out);
    }
    else if (turn != 0)
    {
        uint negdim;
        if (turn < 0)  negdim = dim;
        else           negdim = ForwardDim;

        if (FollowRacer)
        {
            if (dim == 1)
                input->horz = - turn;
            else
                input->vert = turn;
        }
        else
        {
            negate_Point (&view_basis.pts[negdim], &view_basis.pts[negdim]);
            swaprows_PointXfrm (&view_basis, dim, ForwardDim);
            needs_recast = true;

            fputs ("basis:", out);
            output_PointXfrm (out, &view_basis);
            fputc ('\n', out);
        }
    }
    else if (view_angle_change != 0)
    {
        if (ray_image.perspective)
        {
            const real epsilon = (real) (.00001);
            const real diff = M_PI / 18;
            if (view_angle_change > 0)  view_angle += diff;
            else                        view_angle -= diff;
            while (view_angle > M_PI - epsilon) view_angle -= diff;
            while (view_angle <        epsilon) view_angle += diff;
            fprintf (out, "view_angle:%fpi\n", view_angle / M_PI);
        }
        else
        {
            const real diff = stride_magnitude;
            if (view_angle_change > 0)  view_width += diff;
            else                        view_width -= diff;
            fprintf (out, "view_width:%f\n", view_width);
        }
    }
    else if (resize != 0)
    {
        uint diff = 100;
        assert (view_nrows == view_ncols);
        if (resize > 0)
        {
            view_nrows += diff;
            view_ncols += diff;
        }
        else if (view_nrows > diff)
        {
            view_nrows -= diff;
            view_ncols -= diff;
        }
        else
        {
            recast = false;
        }
        if (recast)
        {
            ray_image.nrows = view_nrows;
            ray_image.ncols = view_ncols;
            resize_RayImage (&ray_image);
        }
        gtk_window_resize (GTK_WINDOW(widget), view_nrows, view_ncols);
    }
    else if (reflect)
    {
        negate_Point (&view_basis.pts[NDimensions-1],
                      &view_basis.pts[NDimensions-1]);
    }
    else if (rotate_dir_dim)
    {
            /* TODO NO? */
        uint i, n;
        assert (NDimensions >= 3);
        n = NDimensions - 3;
        UFor( i, n )
            swaprows_PointXfrm (&view_basis, 2+i, 3+i);
    }
    else if (FollowRacer && switch_racer != 0)
    {
        if (switch_racer > 0)
            racer_idx = incmod_uint (racer_idx, 1, nracers);
        else
            racer_idx = decmod_uint (racer_idx, 1, nracers);
        printf ("racer_idx:%u\n", racer_idx);
    }
    else if (change_cast_method)
    {
        ray_image.perspective = !ray_image.perspective;
        if (ray_image.perspective)
            fputs ("method:perspective\n", out);
        else
            fputs ("method:parallel\n", out);
    }
    else if (print_view_code)
    {
        uint i;
        fprintf (out, "*view_angle = %g;\n", view_angle);
        UFor( i, NDimensions )
        {
            uint j;
            fprintf (out, "view_origin->coords[%u] = %g;\n",
                     i, view_origin.coords[i]);
            UFor( j, NDimensions )
            {
                fprintf (out, "view_basis->pts[%u].coords[%u] = %g;\n",
                         i, j, view_basis.pts[i].coords[j]);
            }
        }
    }
    else
    {
        recast = false;
    }


    if (recast)  needs_recast = true;

    return TRUE;
}

static
    gboolean
key_release_fn (GtkWidget* _widget, GdkEventKey* event, gpointer data)
{
    MotionInput* input;
    (void) _widget;

    input = (MotionInput*) data;

    switch (event->keyval)
    {
        case GDK_Up:
        case GDK_Down:
            input->vert = 0;
            break;
        case GDK_Right:
        case GDK_Left:
            input->horz = 0;
            break;
        case GDK_a:
            input->stride[ForwardDim] = 0;
            break;
        case GDK_e:
            input->boost = false;
            break;
    }
    return TRUE;
}


static
    void
set_rotate_view (real vert, real horz)
{
    Point dir;
    PointXfrm tmp;
    const uint dir_dim = ForwardDim;
    copy_PointXfrm (&tmp, &view_basis);
    zero_Point (&dir);
    dir.coords[UpDim]    = vert * sin (view_angle / 2);
    dir.coords[RightDim] = horz * sin (view_angle / 2);
    dir.coords[dir_dim] = 1;
    trxfrm_Point (&tmp.pts[dir_dim], &view_basis, &dir);
    orthorotate_PointXfrm (&view_basis, &tmp, dir_dim);
}


static
    void
joystick_button_event (MotionInput* mot,
                       uint devidx, uint btnidx, bool pressed)
{
    tristate value;
#if 0
    FILE* out = stderr;
    fprintf (out, "\rdevidx:%u  btnidx:%u  pressed:%u",
             devidx, btnidx, (uint) pressed);
#else
    (void) devidx;
#endif

    value = pressed ? 1 : 0;

    if (btnidx == 0)  mot->boost = pressed;
    if (btnidx == 2)  mot->stride[ForwardDim] =  value;
    if (btnidx == 3)  mot->stride[ForwardDim] = -value;
}

static
    void
joystick_axis_event (MotionInput* mot, uint devidx, uint axisidx, int value)
{
    const int js_max =  32767;
    const int js_min = -32768;
    const int js_nil = 3000;  /* Dead zone!*/
    real x = 0;
#if 0
    FILE* out = stderr;
    fprintf (out, "\rdevidx:%u  axisidx:%u  value:%d",
             devidx, axisidx, value);
#else
    (void) devidx;
#endif

    if (value < 0)  { if (value < -js_nil)  x = - (real) value / js_min; }
    else            { if (value > +js_nil)  x = + (real) value / js_max; }

    if (axisidx == 0)  mot->horz = -x;
    if (axisidx == 1)  mot->vert = -x;
    if (axisidx == 2)  mot->drift = x;
    if (axisidx == 3)  mot->view_azimuthcc = -x;
    if (axisidx == 4)  mot->view_zenith = -x;
}


    /* Global effects!*/
    /* A call to this better be in a mutex!*/
static
    void
update_object_locations (RaySpace* space, MotionInput* input,
                         uint idx, real dt)
{
    real x;
    ObjectMotion* motion;
    motion = &racer_motions[idx];

    x = input->vert;
    if (!input->inv_vert) x = - x;

    rotate_object (motion, UpDim, ForwardDim, x * (M_PI / 2));

    x = input->horz * (M_PI / 2);
    if (input->use_roll)
        rotate_object (motion, RightDim, UpDim, x);
    else
        rotate_object (motion, RightDim, ForwardDim, x);

    motion->thrust = input->stride[ForwardDim];
    motion->boost = input->boost;

    move_objects (space, racer_motions, dt);
}

static
    void
update_camera_location (const MotionInput* input, real dt)
{
    uint i;
    real x;
    PointXfrm tmp_basis;

    x = input->vert;
    if (!input->inv_vert) x = - x;

    copy_PointXfrm (&tmp_basis, &view_basis);
        /* /x/ comes in as the vertical rotation.*/
    trrotate_PointXfrm (&tmp_basis, UpDim, ForwardDim, x * dt * (M_PI / 2));

    x = input->horz * dt * (M_PI / 2);
    if (input->use_roll)
        trrotate_PointXfrm (&tmp_basis, RightDim, UpDim, x);
    else
        trrotate_PointXfrm (&tmp_basis, RightDim, ForwardDim, x);

    orthonormalize_PointXfrm (&view_basis, &tmp_basis);

    UFor( i, NDimensions )
    {
        Point diff;
        scale_Point (&diff, &view_basis.pts[i],
                     stride_magnitude * dt * input->stride[i]);
        summ_Point (&view_origin, &view_origin, &diff);
    }

    if (NDimensions == 4)
    {
        x = (1 + input->drift) / 2;
        x = clamp_real (x, 0, 1);
        view_origin.coords[DriftDim] = x;
    }
}


    /* Global effects!*/
static
    void
update_view_params (const RaySpace* space, uint idx, const MotionInput* input)
{
    real view_zenith, view_azimuthcc;
    Point veloc;
    PointXfrm basis, rotation;
    const ObjectRaySpace* object;
    const ObjectMotion* motion;

    if (!FollowRacer)  return;

    object = &space->objects[idx];
    motion = &racer_motions[idx];

    copy_PointXfrm (&basis, &object->orientation);

    copy_Point (&veloc, &motion->veloc);
    if (NDimensions == 4)
        veloc.coords[DriftDim] = 0;

    Op_Point_2010( &basis.pts[ForwardDim]
                   ,+, &basis.pts[ForwardDim]
                   ,   .002*, &veloc );
    orthorotate_PointXfrm (&view_basis, &basis, ForwardDim);
    copy_PointXfrm (&basis, &view_basis);

    view_zenith    = (M_PI / 2) * (1 + input->view_zenith);
    view_azimuthcc =  M_PI      * (1 + input->view_azimuthcc);

    spherical3_PointXfrm (&rotation, view_zenith, view_azimuthcc - M_PI);

    xfrm_PointXfrm (&view_basis, &rotation, &basis);

    Op_Point_2021010( &view_origin
                      ,+, &object->centroid
                      ,   +, -130*, &view_basis.pts[ForwardDim]
                      ,       50*,  &view_basis.pts[UpDim] );
}


#if 0
static gboolean move_mouse_fn (GtkWidget* widget,
                               GdkEventButton* event,
                               Gdk)
{
    return TRUE;
}
#endif

static gboolean grab_mouse_fn (GtkWidget* da,
                               GdkEventButton* event,
                               gpointer state)
{
        /* GdkCursor* cursor; */
    int width, height;
    uint row, col;
    FILE* out;
    bool do_rotate = false;
    bool do_raycast = false;
    const RaySpace* space;

    space = (RaySpace*) state;

    out = stdout;

    gdk_drawable_get_size (da->window, &width, &height);

    if (event->x < 0 || view_ncols < event->x ||
        event->y < 0 || view_nrows < event->y)
    {
        puts ("Stay in the box hoser!\n");
        return FALSE;
    }

    row = view_nrows - event->y - 1;
    col = event->x;

    if      (event->button == 1)  do_rotate = true;
    else if (event->button == 3)  do_raycast = true;

    if (do_rotate)
    {
        real vert, horz;
            /* Our X is vertical and Y is horizontal.*/
        vert = (2 * (real) row - view_nrows) / view_nrows;
        horz = (2 * (real) col - view_ncols) / view_ncols;

        fprintf (out, "vert:%f  horz:%f\n", vert, horz);

        set_rotate_view (vert, horz);

        fputs ("basis:", out);
        output_PointXfrm (out, &view_basis);
        fputc ('\n', out);
        needs_recast = true;

#if 0
        cursor = gdk_cursor_new (GDK_GUMBY);
        gdk_pointer_grab (da->window,
                          FALSE,
                          0,
                          da->window,
                          cursor,
                          event->time);
        gdk_cursor_unref (cursor);

#endif

        mouse_coords[0] = event->x_root;
        mouse_coords[1] = event->y_root;
        mouse_diff[0] = 0;
        mouse_diff[1] = 0;
    }
    else if (do_raycast)
    {
        RayCastAPriori priori;
        uint hit_idx = Max_uint;
        real hit_mag = Max_real;
        uint hit_objidx = Max_uint;
        Point origin, dir;

        setup_RayCastAPriori (&priori, &ray_image,
                              &view_origin, &view_basis,
                              &space->main.box);

        ray_from_RayCastAPriori (&origin, &dir,
                                 &priori, row, col, &ray_image);

        cast_nopartition (&hit_idx, &hit_mag, &hit_objidx,
                          space, &origin, &dir,
                          priori.inside_box,
                          Max_uint);
        if (hit_objidx <= space->nobjects)
        {
            const ObjectRaySpace* object;
            Point isect;
            uint nodeidx, parent_nodeidx;
            scale_Point (&isect, &dir, hit_mag);
            summ_Point (&isect, &origin, &isect);

            nodeidx = find_KDTreeNode (&parent_nodeidx, &isect,
                                       space->main.tree.nodes);

            if (hit_objidx < space->nobjects)
                object = &space->objects[hit_objidx];
            else
                object = &space->main;
            output_SceneElement (out, &object->scene, hit_idx);
            fputc ('\n', out);
            fprintf (out, "Elem:%u  Obj:%u  Node:%u  Intersect:",
                     hit_idx, hit_objidx, nodeidx);
            output_Point (out, &isect);
            fputc ('\n', out);
            output_Point (out, &object->simplices[hit_idx].plane.normal);
            fputc ('\n', out);
        }
        else
        {
            fputs ("No hit!\n", out);
        }
    }

    return FALSE;
}

static
    void
render_pattern (byte* data, void* state, int height, int width, int stride)
{
    gint32 x, y;
    (void) state;
    UFor( y, height )
    {
        guint32* line;
        line = (guint32*) &data[stride * y];
        UFor( x, width )
        {
            guint32 v;
            unsigned char r, g, b;
            r = (unsigned char) ((~x |  y) / 2);
            g = (unsigned char) (( x | ~y) / 2);
            b = (unsigned char) ((~x | ~y) / 2);

            v = 0xFF;
            v = (v << 8) | r;
            v = (v << 8) | g;
            v = (v << 8) | b;

                /* v  = (guint32)(0x7FFFFF*(1+sin(~(x^y) % ~(x|y)))); */
                /* v |= 0xFF000000; */

            line[x] = v;
        }
    }
}


static
    void
render_RaySpace (byte* data, RaySpace* space,
                 uint nrows, uint ncols, uint stride)
{
    uint row;

    if (needs_recast)
    {
        FILE* out = stderr;
        PointXfrm basis;
        Point origin;

        motion_input.t1 = (real) SDL_GetTicks () / 1000;
        if (motion_input.t0 == 0)
        {
            motion_input.t0 = motion_input.t1;
        }
        else
        {
            real dt;
            dt = motion_input.t1 - motion_input.t0;
            motion_input.t0 = motion_input.t1;

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

            if (FollowRacer)
                update_object_locations (space, &motion_input, racer_idx, dt);
            else
                update_camera_location (&motion_input, dt);
        }

        if (FollowRacer && ShowSpeed)
            fprintf (out, "SPEED:%f\n",
                     magnitude_Point (&racer_motions[racer_idx].veloc));

        update_view_params (space, racer_idx, &motion_input);
        copy_Point (&origin, &view_origin);
        copy_PointXfrm (&basis, &view_basis);
        needs_recast = false;

        if (ray_image.perspective)  ray_image.hifov = view_angle;
        else                        ray_image.hifov = view_width;

#ifdef DistribCompute
        compute_rays_to_hits (&ray_image, space, &origin, &basis);
#else
        if (LightAtCamera)
        {
            assert (space->nlights > 0);
            copy_Point (&space->lights[0].location, &origin);
            space->lights[0].intensity = .5;
        }
        update_dynamic_RaySpace (space);
        if (ForceFauxFishEye)
            rays_to_hits_fish (&ray_image, space,
                               &origin, &basis, view_angle);
        else
            cast_RayImage (&ray_image, space, &origin, &basis);
#endif
    }

    assert (ray_image.pixels);

    UFor( row, view_nrows )
    {
        uint col;
        const byte* pixline;
        guint32* outline;
        if (view_nrows - row - 1 >= nrows)  continue;

        pixline = &ray_image.pixels[row * 3 * view_ncols];
        outline = (guint32*) &data[stride * (view_nrows - row - 1)];

        UFor( col, view_ncols )
        {
            if (col >= ncols)  break;
            outline[col] =
                ((guint32) 0xFF             << 24) |
                ((guint32) pixline[3*col+0] << 16) |
                ((guint32) pixline[3*col+1] <<  8) |
                ((guint32) pixline[3*col+2] <<  0);
        }
    }
}


static
    gboolean
render_expose (GtkWidget* da,
               GdkEventExpose* event,
               gpointer state)
{
    cairo_surface_t* surface;
    cairo_t* cr;
    byte* data;
    int width, height, stride;

    (void) event;

    gdk_drawable_get_size (da->window, &width, &height);

    surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32, width, height);

    stride = cairo_image_surface_get_stride (surface);
    data = cairo_image_surface_get_data (surface);

    if (RenderDrawsPattern)
        render_pattern (data, (void*) state, height, width, stride);
    else
        render_RaySpace (data, (RaySpace*) state,
                         (uint) height, (uint) width, (uint) stride);

    cairo_surface_mark_dirty (surface);
    cr = gdk_cairo_create (da->window);
    cairo_set_source_surface (cr, surface, 0, 0);
    cairo_paint (cr);

    cairo_surface_destroy (surface);
    cairo_destroy (cr);
    return TRUE;
}

static
    gboolean
update_rendering (gpointer widget)
{
    if (needs_recast)
        gtk_widget_queue_draw ((GtkWidget*)widget);
    return TRUE;
}

static
    void
gui_main (int argc, char* argv[], RaySpace* space)
{
    GtkWidget *window;
    gtk_init (&argc, &argv);

    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_widget_add_events (window, GDK_BUTTON_PRESS_MASK);

    g_signal_connect (window, "delete-event",
                      G_CALLBACK(delete_event), NULL);
    g_signal_connect (window, "destroy",
                      G_CALLBACK(destroy_app), NULL);
    g_signal_connect (window, "expose-event",
                      G_CALLBACK(render_expose), space);
    g_signal_connect (window, "key-press-event",
                      G_CALLBACK(key_press_fn), &motion_input);
    g_signal_connect (window, "key-release-event",
                      G_CALLBACK(key_release_fn), &motion_input);
    g_signal_connect (window, "button-press-event",
                      G_CALLBACK(grab_mouse_fn), space);
    g_timeout_add (1000 / 100, update_rendering, window);


    gtk_window_set_position (GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_window_set_title (GTK_WINDOW(window), "OuTHER SPACE");
    gtk_window_set_default_size (GTK_WINDOW(window), view_ncols, view_nrows); 

    gtk_widget_show_all (window);
    gtk_main ();
}

    Uint32
keepalive_sdl_event_fn (Uint32 interval,  void* param)
{
    SDL_Event event;
    (void) param;
    event.type = SDL_USEREVENT;
    event.user.code = 2;
    event.user.data1 = 0;
    event.user.data2 = 0;
    SDL_PushEvent (&event);
    return interval;
}

    gpointer
sdl_main (gpointer data)
{
    SDL_TimerID timer;
    SDL_Joystick* joystick_handle; 
    SDL_Event event;
    Mix_Chunk* tune = 0;
    int ret;
    int channel = -1;
    FILE* out = stderr;
    const char* pathname;

    pathname = (char*) data;

    SDL_JoystickEventState (SDL_ENABLE);
    joystick_handle = SDL_JoystickOpen (0);
    fprintf (stderr, "Joystick has %d buttons!\n",
             SDL_JoystickNumButtons (joystick_handle));


    ret = Mix_OpenAudio(22050, AUDIO_S16SYS, 2, 1024);
    if (ret != 0)
    {
        fputs ("Could not open audio!\n", out);
    }
    else
    {
        const char fname[] = "music0.ogg";
        char* buf;

        buf = AllocT( char, strlen (pathname) + 1 + strlen (fname) + 1 );
        sprintf (buf, "%s/%s", pathname, fname);
        tune = Mix_LoadWAV (buf);

        if (!tune)
            fprintf (out, "No input music file:%s!\n", buf);
        else
            channel = Mix_PlayChannel (-1, tune, -1);

        free (buf);
    }

        /* Add a timer to assure the event loop has
         * an event to process when we should exit.
         * Also, so updates can occur.
         */
    timer = SDL_AddTimer (40, keepalive_sdl_event_fn, 0);
    assert (timer);

    while (continue_running && SDL_WaitEvent (&event))
    {
        switch (event.type)
        {
            case SDL_JOYAXISMOTION:
                {
                    SDL_JoyAxisEvent* je;
                    je = &event.jaxis;
                    joystick_axis_event (&motion_input,
                                         je->which, je->axis, je->value);
                }
                break;
            case SDL_JOYBUTTONUP:
            case SDL_JOYBUTTONDOWN:
                {
                    SDL_JoyButtonEvent* je;
                    je = &event.jbutton;
                    joystick_button_event (&motion_input,
                                           je->which, je->button,
                                           je->state == SDL_PRESSED);
                }
                break;
            case SDL_USEREVENT:
            default:
                break;

        }
        needs_recast = true;
    }

    SDL_RemoveTimer (timer);

    if (channel >= 0)
        Mix_HaltChannel (channel);
    if (tune)
        Mix_FreeChunk (tune);
    Mix_CloseAudio();

    SDL_JoystickClose (joystick_handle);
    SDL_Quit ();
    return 0;
}


int main (int argc, char* argv[])
{
    bool good = true;
    bool call_gui = true;
    RaySpace space;
    char inpathname[1024];

    sprintf (inpathname, "%s", "input");

    if (RunFromMyMac)
    {
        char pathname[1024];
        uint idx = 0;
        const char* str;
        str = strrchr (argv[0], '/');
        if (str)  idx = index_of (str, argv[0], sizeof(char));
        CopyT( char, pathname, argv[0], 0, idx );
        pathname[idx] = 0;
        sprintf (inpathname, "%s/../Resources/share/outherspace", pathname);
    }


#ifdef DistribCompute
    init_compute (&argc, &argv);
#endif

    good =
#if 0
        setup_testcase_simple (&space, &view_origin,
                               &view_basis, &view_angle,
                               inpathname, "machine0.obj");
#else
#if 0
#elif 0
        setup_testcase_triangles
#elif 1
        setup_testcase_track
#elif 0
        setup_testcase_bouncethru
#elif 0
        setup_testcase_smoothsphere
#elif 0
        setup_testcase_4d_normals
#elif 0
        setup_testcase_4d_surface
#elif 0
        setup_testcase_manual_interp
#elif 1
        setup_testcase_sphere
#endif
        (&space, &view_origin, &view_basis, &view_angle,
         inpathname);
#endif  /* Pre-rolled testcase.*/

    if (!good)
    {
        fputs ("Setup failed!\n", stderr);
        return 1;
    }

    if (FollowRacer)
    {
        uint i;
        assert (space.nobjects <= NRacersMax);
        nracers = space.nobjects;
        UFor( i, space.nobjects )
            init_ObjectMotion (&racer_motions[i], &space.objects[i]);
    }

#ifdef DistribCompute
    call_gui = !rays_to_hits_computeloop (&space);
#endif

    if (call_gui)
    {
        GThread* sdl_thread;
        int ret;
        init_RayImage (&ray_image);
        ray_image.nrows = view_nrows;
        ray_image.ncols = view_ncols;
            /* ray_image.hits = AllocT( uint, 1 ); */
            /* ray_image.mags = AllocT( real, 1 ); */
        ray_image.pixels = AllocT( byte, 1 );
        resize_RayImage (&ray_image);
        ray_image.view_light = 0;
        ray_image.color_distance_on = true;

        g_thread_init (0);

        init_MotionInput (&motion_input);
            /* Note: SDL_INIT_VIDEO is required for some silly reason!*/
        ret = SDL_Init (SDL_INIT_AUDIO |
                        SDL_INIT_EVENTTHREAD |
                        SDL_INIT_JOYSTICK |
                        SDL_INIT_TIMER |
                        SDL_INIT_VIDEO);
        assert (ret == 0);

        sdl_thread = g_thread_create (sdl_main, inpathname, true, 0);

        gui_main (argc, argv, &space);

        g_thread_join (sdl_thread);
#ifdef DistribCompute
        stop_computeloop ();
#endif
    }

#ifdef DistribCompute
    cleanup_compute ();
#endif
    return 0;
}

