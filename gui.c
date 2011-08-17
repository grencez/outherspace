
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
#ifdef SupportImage
#include <SDL_image.h>
#endif
#ifdef SupportSound
#include <SDL_mixer.h>
#endif

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

typedef struct motion_input_struct MotionInput;
typedef struct race_craft_struct RaceCraft;
typedef struct pilot_struct Pilot;


struct motion_input_struct
{
    real vert;
    real horz;
    real drift;
    real stride[NDimensions];
    real thrust[2];
    real view_azimuthcc;
    real view_zenith;
    bool boost;
    bool inv_vert;
    bool use_roll;
    bool firing[2];
};

struct race_craft_struct
{
    real health;
    uint pilot_idx;
};

struct pilot_struct
{
    uint craft_idx;
    MotionInput input;
    Point view_origin;
    PointXfrm view_basis;

    uint image_start_row;
    uint image_start_col;
    RayImage ray_image;

    real stride_magnitude;
    real view_angle;
    real view_width;

        /* Offsets from racer centroid when stationary.*/
    real up_offset;
    real forward_offset;

        /* Not sure if these will be used evar.*/
    uint mouse_coords[2];
    uint mouse_diff[2];
};

    /* Global state changes.*/
static uint ncheckplanes = 0;
static Plane* checkplanes;
static Point* checkpoints;
static uint nracers = 0;
static uint npilots = 1;
static uint kbd_pilot_idx = 0;
static ObjectMotion racer_motions[NRacersMax];  /* Reset every frame.*/
static RaceCraft crafts[NRacersMax];
static Pilot pilots[NRacersMax];
static uint view_nrows = 400;
static uint view_ncols = 400;
static bool needs_recast = true;
static bool continue_running = true;
static real frame_t0 = 0;
static real frame_t1 = 0;
static real framerate_report_dt = 0;
static uint framerate_report_count = 0;
static GCond* pilots_initialized = 0;
    /* Object indices.*/
static uint laser_objidcs[NRacersMax][2];
static uint checkplane_objidx;


static
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
}

static
    void
init_RaceCraft (RaceCraft* craft)
{
    craft->health = 1;
}

static
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

static
    void
cleanup_Pilot (Pilot* p)
{
    cleanup_RayImage (&p->ray_image);
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
    uint i;
    (void) widget;
    (void) data;
    UFor( i, NRacersMax )
        cleanup_Pilot (&pilots[i]);
    continue_running = false;
    gtk_main_quit ();
}

static
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

static
    gboolean
key_press_fn (GtkWidget* widget, GdkEventKey* event, gpointer data)
{
    uint dim = NDimensions;
    tristate roll = 0;
    tristate stride = 0;
    tristate turn = 0;
    tristate camera_offset = 0;
    tristate view_angle_change = 0;
    tristate resize = 0;
    tristate view_light_change = 0;
    tristate stride_mag_change = 0;
    tristate switch_racer = 0;
    tristate switch_kbd_pilot = 0;
    bool reflect = false;
    bool rotate_dir_dim = false;
    bool change_cast_method = false;
    bool print_view_code = false;
    bool recast = true;
    FILE* out = stdout;
    bool shift_mod, ctrl_mod;
    Pilot* pilot;
    MotionInput* input;
    Point* view_origin;
    PointXfrm* view_basis;
    real stride_magnitude;
    RayImage* ray_image;
    uint craft_idx;
    ObjectMotion* racer_motion;

    pilot = &((Pilot*) data)[kbd_pilot_idx];
    input = &pilot->input;
    view_origin = &pilot->view_origin;
    view_basis = &pilot->view_basis;
    stride_magnitude = pilot->stride_magnitude;
    ray_image = &pilot->ray_image;
    craft_idx = pilot->craft_idx;
    racer_motion = &racer_motions[craft_idx];

    shift_mod = event->state & GDK_SHIFT_MASK;
    ctrl_mod = event->state & GDK_CONTROL_MASK;

        /* /usr/include/gtk-2.0/gdk/gdkkeysyms.h */
        /* printf ("%x\n", event->keyval); */
    switch (event->keyval)
    {
        case GDK_Up:
            if (FollowRacer)
            {
                if (shift_mod) { dim = UpDim;  camera_offset = 1; }
                else if (ctrl_mod) { dim = ForwardDim;  camera_offset = 1; }
                else { dim = UpDim;  turn = 1; }
            }
            else if (shift_mod) { dim = UpDim;  stride = 1; }
            else if (ctrl_mod) { dim = UpDim;  turn = 1; }
            else { dim = ForwardDim;  stride = 1; }
            break;
        case GDK_Down:
            if (FollowRacer)
            {
                if (shift_mod) { dim = UpDim;  camera_offset = -1; }
                else if (ctrl_mod) { dim = ForwardDim;  camera_offset = -1; }
                else { dim = UpDim;  turn = -1; }
            }
            else if (shift_mod) { dim = UpDim;  stride = -1; }
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
        case GDK_f:
            switch_kbd_pilot = 1;  break;
        case GDK_F:
            switch_kbd_pilot = -1;  break;
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
            ray_image->view_light += diff;
        else if (diff <= ray_image->view_light)
            ray_image->view_light -= diff;;

        fprintf (out, "view_light:%f\n", ray_image->view_light);
    }
    else if (camera_offset != 0)
    {
        if (dim == UpDim)
            pilot->up_offset += stride_magnitude * (real) camera_offset;
        if (dim == ForwardDim)
            pilot->forward_offset += stride_magnitude * (real) camera_offset;
    }
    else if (stride != 0 && FollowRacer)
    {
        input->stride[dim] = stride;
    }
    else if (stride != 0 && !FollowRacer)
    {
        Point diff;
        scale_Point (&diff, &view_basis->pts[dim], stride_magnitude * stride);
        summ_Point (view_origin, view_origin, &diff);

        fputs ("pos:", out);
        output_Point (out, view_origin);
        fputc ('\n', out);
    }
    else if (stride_mag_change != 0)
    {
        if (stride_mag_change > 0)  stride_magnitude *= 2;
        else                        stride_magnitude /= 2;
        pilot->stride_magnitude = stride_magnitude;
        fprintf (out, "stride_magnitude: %f\n", stride_magnitude);
        recast = false;
    }
    else if (roll != 0)
    {
        PointXfrm tmp_basis;
        copy_PointXfrm (&tmp_basis, view_basis);
        rotate_PointXfrm (&tmp_basis, UpDim, RightDim, roll * M_PI / 8);
        orthonormalize_PointXfrm (view_basis, &tmp_basis);

        fputs ("basis:", out);
        output_PointXfrm (out, view_basis);
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
            negate_Point (&view_basis->pts[negdim], &view_basis->pts[negdim]);
            swaprows_PointXfrm (view_basis, dim, ForwardDim);
            needs_recast = true;

            fputs ("basis:", out);
            output_PointXfrm (out, view_basis);
            fputc ('\n', out);
        }
    }
    else if (view_angle_change != 0)
    {
        if (ray_image->perspective)
        {
            const real epsilon = (real) (.00001);
            const real diff = M_PI / 18;
            real view_angle;
            view_angle = pilot->view_angle;
            if (view_angle_change > 0)  view_angle += diff;
            else                        view_angle -= diff;
            while (view_angle > M_PI - epsilon) view_angle -= diff;
            while (view_angle <        epsilon) view_angle += diff;
            pilot->view_angle = view_angle;
            fprintf (out, "view_angle:%fpi\n", view_angle / M_PI);
        }
        else
        {
            const real diff = stride_magnitude;
            real view_width;
            view_width = pilot->view_width;
            if (view_angle_change > 0)  view_width += diff;
            else                        view_width -= diff;
            pilot->view_width = view_width;
            fprintf (out, "view_width:%f\n", view_width);
        }
    }
    else if (resize != 0)
    {
        uint diff = 100;
            /* assert (view_nrows == view_ncols); */
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
            resize_pilot_viewports (view_nrows, view_ncols);
        }
        gtk_window_resize (GTK_WINDOW(widget), view_ncols, view_nrows);
    }
    else if (reflect)
    {
        negate_Point (&view_basis->pts[NDimensions-1],
                      &view_basis->pts[NDimensions-1]);
    }
    else if (rotate_dir_dim)
    {
            /* TODO NO? */
        uint i, n;
        assert (NDimensions >= 3);
        n = NDimensions - 3;
        UFor( i, n )
            swaprows_PointXfrm (view_basis, 2+i, 3+i);
    }
    else if (FollowRacer && switch_racer != 0)
    {
        if (switch_racer > 0)
            craft_idx = incmod_uint (craft_idx, 1, nracers);
        else
            craft_idx = decmod_uint (craft_idx, 1, nracers);
        pilot->craft_idx = craft_idx;
        printf ("craft_idx:%u\n", craft_idx);
    }
    else if (switch_kbd_pilot != 0)
    {
        if (switch_kbd_pilot > 0)
            kbd_pilot_idx = incmod_uint (kbd_pilot_idx, 1, npilots);
        else
            kbd_pilot_idx = decmod_uint (kbd_pilot_idx, 1, npilots);
    }
    else if (change_cast_method)
    {
        ray_image->perspective = !ray_image->perspective;
        if (ray_image->perspective)
            fputs ("method:perspective\n", out);
        else
            fputs ("method:parallel\n", out);
    }
    else if (print_view_code)
    {
        uint i;
        fprintf (out, "*view_angle = %g;\n", pilot->view_angle);
        UFor( i, NDimensions )
        {
            uint j;
            fprintf (out, "view_origin->coords[%u] = %g;\n",
                     i, view_origin->coords[i]);
            UFor( j, NDimensions )
            {
                fprintf (out, "view_basis->pts[%u].coords[%u] = %g;\n",
                         i, j, view_basis->pts[i].coords[j]);
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

    input = &((Pilot*) data)[kbd_pilot_idx].input;

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
joystick_button_event (MotionInput* mot, uint btnidx, bool pressed)
{
    tristate value;
#if 0
    FILE* out = stderr;
    fprintf (out, "\rdevidx:%u  btnidx:%u  pressed:%u",
             devidx, btnidx, (uint) pressed);
#endif

    value = pressed ? 1 : 0;

    if (btnidx == 0)  mot->boost = pressed;
    if (btnidx == 2)  mot->stride[ForwardDim] =  value;
    if (btnidx == 3)  mot->stride[ForwardDim] = -value;
    if (btnidx == 4)  mot->firing[0] = pressed;
    if (btnidx == 5)  mot->firing[1] = pressed;
}

static
    void
joystick_axis_event (MotionInput* mot, uint axisidx, int value)
{
    const int js_max =  32767;
    const int js_min = -32768;
    const int js_nil = 3000;  /* Dead zone!*/
    real x = 0;
#if 0
    FILE* out = stderr;
    fprintf (out, "\raxisidx:%u  value:%d", axisidx, value);
#endif

    if (value < 0)  { if (value < -js_nil)  x = - (real) value / js_min; }
    else            { if (value > +js_nil)  x = + (real) value / js_max; }

    if (axisidx == 0)  mot->horz = -x;
    if (axisidx == 1)  mot->vert = -x;
    if (axisidx == 2)  mot->thrust[0] = clamp_real ((x+1)/2, 0, 1);
    if (axisidx == 3)  mot->view_azimuthcc = -x;
    if (axisidx == 4)  mot->view_zenith = -x;
    if (axisidx == 5)  mot->thrust[1] = clamp_real ((x+1)/2, 0, 1);
}


    /* Global effects!*/
    /* A call to this better be in a mutex!*/
static
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
}

static
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

    if (NDimensions == 4)
    {
        x = (1 + input->drift) / 2;
        x = clamp_real (x, 0, 1);
        view_origin->coords[DriftDim] = x;
    }
}


static
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
    RayCastAPriori priori;
    Point origin, dir;

    const RaySpace* space;
    Pilot* pilot;

    space = (RaySpace*) state;
    pilot = &pilots[kbd_pilot_idx];

    out = stdout;

    gdk_drawable_get_size (da->window, &width, &height);

    row = event->y;
    col = event->x;
    if (event->y < pilot->image_start_row ||
        event->x < pilot->image_start_col)
    {
        fputs ("Stay in the box hoser!\n", out);
        return FALSE;
    }

    row -= pilot->image_start_row;
    col -= pilot->image_start_col;
    if (pilot->ray_image.nrows <= row ||
        pilot->ray_image.ncols <= col)
    {
        fputs ("Stay in the box hoser!\n", out);
        return FALSE;
    }

        /* Row numbering becomes bottom-to-top.*/
    row = pilot->ray_image.nrows - row - 1;


    printf ("row:%u col:%u\n", row, col);

    if      (event->button == 1)  do_rotate = true;
    else if (event->button == 3)  do_raycast = true;

    if (do_rotate || do_raycast)
    {
        setup_RayCastAPriori (&priori,
                              &pilot->ray_image,
                              &pilot->view_origin,
                              &pilot->view_basis,
                              &space->main.box);

        ray_from_RayCastAPriori (&origin, &dir,
                                 &priori, row, col,
                                 &pilot->ray_image);
    }

    if (do_rotate)
    {
        PointXfrm tmp_basis;
        copy_Point (&pilot->view_origin, &origin);

        copy_PointXfrm (&tmp_basis, &pilot->view_basis);
        copy_Point (&tmp_basis.pts[ForwardDim], &dir);
        orthorotate_PointXfrm (&pilot->view_basis, &tmp_basis, ForwardDim);

        fputs ("basis:", out);
        output_PointXfrm (out, &pilot->view_basis);
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

        pilot->mouse_coords[0] = event->x_root;
        pilot->mouse_coords[1] = event->y_root;
        pilot->mouse_diff[0] = 0;
        pilot->mouse_diff[1] = 0;
    }
    else if (do_raycast)
    {
        uint hit_idx = Max_uint;
        real hit_mag = Max_real;
        uint hit_objidx = Max_uint;

        cast_nopartition (&hit_idx, &hit_mag, &hit_objidx,
                          space, &origin, &dir,
                          priori.inside_box,
                          Max_uint);
        if (hit_objidx <= space->nobjects)
        {
            const ObjectRaySpace* object;
            Point isect;
            uint nodeidx, parent_nodeidx;
            Op_Point_2010( &isect
                           ,+, &origin
                           ,   hit_mag*, &dir );

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

static
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

static
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

static
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
render_RaySpace (byte* data, RaySpace* space,
                 uint nrows, uint ncols, uint stride)
{
    uint pilot_idx;
    if (needs_recast)
    {
        FILE* out = stderr;

        needs_recast = false;

        frame_t1 = (real) SDL_GetTicks () / 1000;
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

    UFor( pilot_idx, npilots )
    {
        uint ray_row, start_row, start_col;
        Pilot* pilot;
        RayImage* ray_image;

        pilot = &pilots[pilot_idx];
        ray_image = &pilot->ray_image;

        assert (ray_image->pixels);
        if (ray_image->nrows == 0)  continue;
        start_row = pilot->image_start_row + ray_image->nrows - 1;
        start_col = pilot->image_start_col;

        UFor( ray_row, ray_image->nrows )
        {
            uint image_row, ray_col;
            const byte* pixline;
            guint32* outline;

            image_row = start_row - ray_row;
            if (image_row >= nrows)  continue;

            pixline = &ray_image->pixels[ray_row * 3 * view_ncols];
            outline = (guint32*) &data[image_row * stride];

            UFor( ray_col, ray_image->ncols )
            {
                uint image_col;
                image_col = start_col + ray_col;
                if (image_col >= ncols)  break;
                outline[image_col] =
                    ((guint32) 0xFF                   << 24) |
                    ((guint32) pixline[3*image_col+0] << 16) |
                    ((guint32) pixline[3*image_col+1] <<  8) |
                    ((guint32) pixline[3*image_col+2] <<  0);
            }
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
                      G_CALLBACK(key_press_fn), pilots);
    g_signal_connect (window, "key-release-event",
                      G_CALLBACK(key_release_fn), pilots);
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
    SDL_Joystick* joysticks[NRacersMax]; 
    SDL_Event event;
    uint i, njoysticks;
#ifdef SupportSound
    Mix_Chunk* tune = 0;
    int ret;
    int channel = -1;
    FILE* out = stderr;
#endif
    const char* pathname;

    pathname = (char*) data;

    SDL_JoystickEventState (SDL_ENABLE);
    njoysticks = SDL_NumJoysticks ();

    UFor( i, njoysticks )
    {
        joysticks[i] = SDL_JoystickOpen (i);
        fprintf (stderr, "Joystick %u has %d buttons!\n",
                 i, SDL_JoystickNumButtons (joysticks[i]));
    }

    if (njoysticks > 0)  npilots = njoysticks;
    resize_pilot_viewports (view_nrows, view_ncols);
    g_cond_signal (pilots_initialized);

#ifdef SupportSound
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
#endif

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
                    MotionInput* input;
                    SDL_JoyAxisEvent* je;
                    je = &event.jaxis;
                    assert ((uint)je->which < npilots);
                    input = &pilots[je->which].input;

                    joystick_axis_event (input, je->axis, je->value);
                }
                break;
            case SDL_JOYBUTTONUP:
            case SDL_JOYBUTTONDOWN:
                {
                    MotionInput* input;
                    SDL_JoyButtonEvent* je;
                    je = &event.jbutton;
                    assert ((uint)je->which < njoysticks);
                    input = &pilots[je->which].input;

                    joystick_button_event (input, je->button,
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

#ifdef SupportSound
    if (channel >= 0)
        Mix_HaltChannel (channel);
    if (tune)
        Mix_FreeChunk (tune);
    Mix_CloseAudio();
#endif

    UFor( i, njoysticks )
        SDL_JoystickClose (joysticks[i]);
    SDL_Quit ();
    return 0;
}


int main (int argc, char* argv[])
{
    bool good = true;
    bool call_gui = true;
    uint i;
    int ret;
    RaySpace space;
    char inpathname[1024];
    Point view_origin;
    PointXfrm view_basis;
    real view_angle;

    sprintf (inpathname, "%s", "data");

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

#ifdef SupportImage
        ret = IMG_Init (IMG_INIT_PNG);
        assert (ret == IMG_INIT_PNG);
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
#elif 1
        true;
        setup_checkplanes_4d_surface (&ncheckplanes, &checkplanes,
                                      &checkpoints);
        good =
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

#ifdef DistribCompute
    call_gui = !rays_to_hits_computeloop (&space);
#endif

    if (call_gui)
    {
        GThread* sdl_thread;
        GMutex* initializing_lock;

        UFor( i, NRacersMax )
        {
            Pilot* pilot;
            RayImage* ray_image;
            pilot = &pilots[i];
            init_Pilot (pilot);
            init_RaceCraft (&crafts[i]);

            pilot->craft_idx = i;

            copy_Point (&pilot->view_origin, &view_origin); 
            copy_PointXfrm (&pilot->view_basis, &view_basis); 
            pilot->view_angle = view_angle;

            ray_image = &pilot->ray_image;
            ray_image->view_light = 0;
            ray_image->color_distance_on = true;

        }

        g_thread_init (0);
        pilots_initialized = g_cond_new ();

            /* Note: SDL_INIT_VIDEO is required for some silly reason!*/
        ret = SDL_Init (
#ifdef SupportSound
                        SDL_INIT_AUDIO |
#endif
                        SDL_INIT_EVENTTHREAD |
                        SDL_INIT_JOYSTICK |
                        SDL_INIT_TIMER |
                        SDL_INIT_VIDEO);
        assert (ret == 0);

        sdl_thread = g_thread_create (sdl_main, inpathname, true, 0);

        initializing_lock = g_mutex_new ();
        g_mutex_lock (initializing_lock);
        g_cond_wait (pilots_initialized, initializing_lock);
        g_mutex_unlock (initializing_lock);
        g_mutex_free (initializing_lock);
        g_cond_free (pilots_initialized);


        if (FollowRacer)
        {
            assert (space.nobjects == 0 && "All objects must be racers.");
            nracers = npilots;
            assert (nracers <= NRacersMax);
            add_racers (&space, nracers, inpathname);
            UFor( i, nracers )
                init_ObjectMotion (&racer_motions[i], &space.objects[i]);
        }

        if (ncheckplanes > 0)
        {
            checkplane_objidx = space.nobjects;
            add_1elem_Scene_RaySpace (&space);
        }

        setup_laser_scenes (&space);

        gui_main (argc, argv, &space);

        g_thread_join (sdl_thread);
#ifdef DistribCompute
        stop_computeloop ();
#endif
    }

#ifdef SupportImage
    IMG_Quit ();
#endif

    if (ncheckplanes > 0)
    {
        free (checkplanes);
        free (checkpoints);
    }

#ifdef DistribCompute
    cleanup_compute ();
#endif
    return 0;
}

