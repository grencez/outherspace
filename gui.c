
#include "main.h"
#include "motion.h"

#ifdef DistribCompute
#include "compute.h"
#endif

#include <math.h>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <SDL.h>

#define NRacers 10

static const bool RenderDrawsPattern = false;
static const bool ForceFauxFishEye = false;

static ObjectMotion racer_motions[NRacers];

static real prev_time;
static Point view_origin;
static PointXfrm view_basis;
static uint view_nrows = 400;
static uint view_ncols = 400;
static uint mouse_coords[2];
static uint mouse_diff[2];
static RayImage ray_image;
static real view_angle = 2 * M_PI / 3;
static real view_width = 100;
static bool view_perspective = true;
static bool needs_recast = true;
static SDL_Joystick* joystick_handle; 

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
    gtk_main_quit ();
}

static
    gboolean
key_press_fn (GtkWidget* widget, GdkEventKey* event, gpointer _data)
{
    uint dim = NDimensions;
    tristate roll = 0;
    tristate stride = 0;
    tristate turn = 0;
    tristate view_angle_change = 0;
    tristate resize = 0;
    tristate view_light_change = 0;
    bool reflect = false;
    bool rotate_dir_dim = false;
    bool change_cast_method = false;
    bool print_view_code = false;
    bool recast = true;
    const real scale = 5;
    FILE* out;
    bool shift_mod, ctrl_mod;

    (void) _data;
    out = stdout;

    shift_mod = event->state & GDK_SHIFT_MASK;
    ctrl_mod = event->state & GDK_CONTROL_MASK;

    switch (event->keyval)
    {
        case GDK_Up:
            if (shift_mod) { dim = 0;  stride = 1; }
            else if (ctrl_mod) { dim = 0;  turn = 1; }
            else { dim = DirDimension;  stride = 1; }
            break;
        case GDK_Down:
            if (shift_mod) { dim = 0;  stride = -1; }
            else if (ctrl_mod) { dim = 0;  turn = -1; }
            else { dim = DirDimension;  stride = -1; }
            break;
        case GDK_Right:
            if (shift_mod) { dim = 1;  stride = 1; }
            else if (ctrl_mod) { dim = 1;  turn = 1; }
            else { roll = -1; }
            break;
        case GDK_Left:
            if (shift_mod) { dim = 1;  stride = -1; }
            else if (ctrl_mod) { dim = 1;  turn = -1; }
            else { roll = 1; }
            break;
        case GDK_Escape:
            gdk_pointer_ungrab (event->time);  break;
        case GDK_d:
            rotate_dir_dim = true;  break;
        case GDK_L:
            view_light_change = 1;  break;
        case GDK_l:
            view_light_change = -1;  break;
        case GDK_P:
            print_view_code = true;  break;
        case GDK_r:
            reflect = true;  break;
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
        real diff = 10;
        if (view_light_change > 0)
            ray_image.view_light += diff;
        else if (diff <= ray_image.view_light)
            ray_image.view_light -= diff;;

        fprintf (out, "view_light:%f\n", ray_image.view_light);
    }

    if (stride != 0)
    {
        Point diff;
        scale_Point (&diff, &view_basis.pts[dim], scale * stride);
        summ_Point (&view_origin, &view_origin, &diff);

        fputs ("pos:", out);
        output_Point (out, &view_origin);
        fputc ('\n', out);
    }
    else if (roll != 0)
    {
        PointXfrm rotation, tmp;
        rotation_PointXfrm (&rotation, 0, 1, roll * M_PI / 8);
        xfrm_PointXfrm (&tmp, &rotation, &view_basis);
        orthonormalize_PointXfrm (&view_basis, &tmp);

        fputs ("basis:", out);
        output_PointXfrm (out, &view_basis);
        fputc ('\n', out);
    }
    else if (turn != 0)
    {
        uint negdim;
        if (turn < 0)  negdim = dim;
        else           negdim = DirDimension;

        negate_Point (&view_basis.pts[negdim], &view_basis.pts[negdim]);
        swaprows_PointXfrm (&view_basis, dim, DirDimension);
        needs_recast = true;

        fputs ("basis:", out);
        output_PointXfrm (out, &view_basis);
        fputc ('\n', out);
    }
    else if (view_angle_change != 0)
    {
        if (view_perspective)
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
            const real diff = 50;
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
    else if (change_cast_method)
    {
        view_perspective = !view_perspective;
        if (view_perspective)
            fputs ("method:perspective\n", out);
        else
            fputs ("method:parallel\n", out);
    }
    else if (print_view_code)
    {
        uint i;
        fprintf (out, "view_angle = (real) %15g;\n", view_angle);
        UFor( i, NDimensions )
        {
            uint j;
            fprintf (out, "view_origin.coords[%u] = (real) %15g;\n",
                     i, view_origin.coords[i]);
            UFor( j, NDimensions )
            {
                fprintf (out, "view_basis.pts[%u].coords[%u] = (real) %15g;\n",
                         i, j, view_basis.pts[i].coords[j]);
            }
        }
    }
    else
    {
        recast = false;
    }


    if (recast)  needs_recast = true;
    gtk_widget_queue_draw (widget);

    return TRUE;
}


static
    void
set_rotate_view (real vert, real horz)
{
    Point dir;
    PointXfrm tmp;
    const uint dir_dim = DirDimension;
    copy_PointXfrm (&tmp, &view_basis);
    zero_Point (&dir);
    dir.coords[0] = vert * sin (view_angle / 2);
    dir.coords[1] = horz * sin (view_angle / 2);
    dir.coords[dir_dim] = 1;
    trxfrm_Point (&tmp.pts[dir_dim], &view_basis, &dir);
    orthorotate_PointXfrm (&view_basis, &tmp, dir_dim);
}


static gboolean poll_joystick (gpointer widget)
{
    const bool flight_style = true;
    int x, y;
    tristate stride = 0;
    real vert, horz, roll;

    SDL_JoystickUpdate ();

    if (0 != SDL_JoystickGetButton (joystick_handle, 2))  stride =  1;
    if (0 != SDL_JoystickGetButton (joystick_handle, 3))  stride = -1;

    x = SDL_JoystickGetAxis (joystick_handle, 1);
    y = SDL_JoystickGetAxis (joystick_handle, 0);

    if (x > 0 && x <  3000)  x = 0;
    if (x < 0 && x > -3000)  x = 0;
    if (y > 0 && y <  3000)  y = 0;
    if (y < 0 && y > -3000)  y = 0;

    vert = (real) x / 327670;
    horz = (real) y / 327670;
    if (flight_style)
    {
        roll = horz;
        horz = 0;
    }
    else
    {
        vert = - vert;
    }

#if 1
    set_rotate_view (vert, horz);

    if (flight_style)
    {
        PointXfrm rotation, tmp;
        rotation_PointXfrm (&rotation, 0, 1, - roll * M_PI / 5);
        xfrm_PointXfrm (&tmp, &rotation, &view_basis);
        orthonormalize_PointXfrm (&view_basis, &tmp);
    }

    if (stride != 0)
    {
        Point diff;
        scale_Point (&diff, &view_basis.pts[DirDimension], stride * 5);
        summ_Point (&view_origin, &view_origin, &diff);
    }
#else
    rotate_object (&racer_motions[0], 0, 2,
                   - vert * M_PI / 3);
    rotate_object (&racer_motions[0], 1, 2,
                   - horz * M_PI / 3);
    if (flight_style)
    {
        rotate_object (&racer_motions[0], 0, 1, roll * M_PI / 3);
    }

    racer_motions[0].accel = 5 * stride;
#endif

        /* fprintf (stderr, "x:%d  y:%d\n", x, y); */
    if (false)
    {
        int nbuttons;
        int i;
        nbuttons = SDL_JoystickNumButtons (joystick_handle);
        for (i = 0; i < nbuttons; ++i)
        {
            uint state;
            state = SDL_JoystickGetButton (joystick_handle, i);
            fprintf (stderr, "button %d has state %u\n", i, state);
        }
    }

    gtk_widget_queue_draw ((GtkWidget*) widget);
    needs_recast = true;

    return TRUE;
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
                               GdkWindowEdge edge)
{
        /* GdkCursor* cursor; */
    int width, height;
    real vert, horz;
    FILE* out;

    (void) edge;

    out = stdout;

    gdk_drawable_get_size (da->window, &width, &height);

    if (event->x < 0 || view_ncols < event->x ||
        event->y < 0 || view_nrows < event->y)
    {
        puts ("Stay in the box hoser!\n");
        return FALSE;
    }

        /* Our X is vertical and Y is horizontal.*/
    vert = (view_nrows - 2 * event->y) / view_nrows;
    horz = (2 * event->x - view_ncols) / view_ncols;

        /* fprintf (out, "vert:%f  horz:%f\n", vert, horz); */

    set_rotate_view (vert, horz);

    fputs ("basis:", out);
    output_PointXfrm (out, &view_basis);
    fputc ('\n', out);
    needs_recast = true;
    gtk_widget_queue_draw (da);

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
render_RaySpace (byte* data, const RaySpace* space,
                 uint nrows, uint ncols, uint stride)
{
    uint row;

    if (needs_recast)
    {
        uint i;
        real time, dt;

        time = monotime ();
        dt = time - prev_time;

        fprintf (stderr, "FPS:%f\n", 1 / dt);

#if 0
        UFor( i, space->nobjects )
        {
            uint j;
            ObjectMotion* motion;
            motion = &racer_motions[i];

                /* space->objects[i].centroid.coords[0] = 50 * sin (time + i); */
                /* space->objects[i].centroid.coords[1] = 50 * cos (time + i); */


                /* rotate_object (motion, 0, 1, .5); */
            UFor( j, 5 )
            {
                    /* rotate_object (motion, 0, 2, .5); */
                    /* rotate_object (motion, 0, 1, .7); */
                    /* rotate_object (motion, 0, 1, .7); */
                    /* motion->accel = 10; */
                    /* motion->veloc.coords[0] = 10 * sin (time + i); */
                    /* motion->veloc.coords[1] = 10 * cos (time + i); */
                move_object (&space->objects[i], motion, dt, true);
            }
            zero_rotations (motion);
        }

        {
            Point tmp;
            PointXfrm basis;
            const ObjectMotion* motion;
            const RaySpaceObject* object;
            motion = &racer_motions[0];
            object = &space->objects[0];
            scale_Point (&tmp, &object->orientation.pts[DirDimension],
                         1 + magnitude_Point (&motion->veloc));
            summ_Point (&tmp, &tmp, &motion->veloc);
                /* summ_Point (&tmp, &motion->veloc, &object->orientation.pts[DirDimension]); */

            normalize_Point (&tmp, &tmp);
            copy_PointXfrm (&basis, &object->orientation);
            copy_Point (&basis.pts[DirDimension], &tmp);
            orthorotate_PointXfrm (&view_basis, &basis, DirDimension);

            scale_Point (&view_origin, &tmp, -130);
            scale_Point (&tmp, &object->orientation.pts[0], 70);
            summ_Point (&view_origin, &view_origin, &tmp);
            summ_Point (&view_origin, &view_origin, &object->centroid);
        }
#endif

#ifdef DistribCompute
        compute_rays_to_hits (&ray_image,
                              space, &view_origin, &view_basis, view_angle);
#else
        if (ForceFauxFishEye)
            rays_to_hits_fish (&ray_image, space,
                               &view_origin, &view_basis, view_angle);
        else if (view_perspective)
            rays_to_hits (&ray_image, space,
                          &view_origin, &view_basis, view_angle);
        else
            rays_to_hits_parallel (&ray_image, space,
                                   &view_origin, &view_basis, view_width);
#endif
        prev_time = time;
    }
    needs_recast = false;

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

#if 0
    UFor( row, view_nrows )
    {
        guint32 color_diff;
        color_diff = (guint32) 0xFFFFFF / (guint32) space->nelems;
        uint* hitline;
        guint32* outline;
        if (row >= nrows)  break;

        hitline = &ray_hits[view_ncols * row];
        outline = (guint32*) &data[stride * (view_nrows - row - 1)];

        UFor( col, view_ncols )
        {
            guint32 i, x, y;
            if (col >= ncols)  break;

            x = color_diff * (space->nelems - hitline[col]);
            y = 0xFF000000;

            UFor( i, 3 )
            {
                guint32 j;
                UFor( j, 8 )
                {
                    if (0 != (x & (1 << (i + 3*j))))
                        y |= (1 << (8*i + j));
                }
            }
            outline[col] = y;
        }
    }
#endif
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
    void
gui_main (int argc, char* argv[], RaySpace* space)
{
        /* GtkWidget is the storage type for widgets */
        /* GtkWidget *frame; */
    GtkWidget *window;
        /* GtkWidget *vbox; */
        /* GtkWidget *da; */

    SDL_InitSubSystem (SDL_INIT_JOYSTICK);
    joystick_handle = SDL_JoystickOpen (0);
    SDL_JoystickEventState (SDL_QUERY);
    fprintf (stderr, "Joystick has %d buttons!\n",
             SDL_JoystickNumButtons (joystick_handle));

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
                      G_CALLBACK(key_press_fn), NULL);
    g_signal_connect (window, "button-press-event",
                      G_CALLBACK(grab_mouse_fn), NULL);
    g_timeout_add (1000 / 60, &poll_joystick, window);


#if 0
    vbox = gtk_vbox_new (FALSE, 8);
        /* gtk_container_set_border_width (GTK_CONTAINER (vbox), 8); */
    gtk_container_add (GTK_CONTAINER (window), vbox);

    frame = gtk_frame_new (NULL);
        /* gtk_frame_set_shadow_type (GTK_FRAME (frame), GTK_SHADOW_IN); */
    gtk_box_pack_start (GTK_BOX (vbox), frame, TRUE, TRUE, 0);

    da = gtk_drawing_area_new ();
    gtk_widget_set_size_request (da, 100, 100);

    gtk_container_add (GTK_CONTAINER (frame), da);

    /*
    {
        int width = 50;
        int height = 50;
        cairo_format_t fmt = CAIRO_FORMAT_ARGB32;
        cairo_surface_t* surface;
        int stride;
        unsigned char* data;
        stride = cairo_format_stride_for_width (fmt, width);
        data = (unsigned char*) malloc (stride * height);

        surface = cairo_image_surface_create_for_data (data, fmt,
                                                       width, height,
                                                       stride);
    }
    */

    /*
    {
        GdkGeometry hints;
        hints.max_width = 500;
        hints.max_height = 500;

        gtk_window_set_geometry_hints (vbox->window, vbox, &hints, GDK_HINT_MAX_SIZE);
    }
    */

    g_signal_connect (da, "expose-event",
                      G_CALLBACK (render_expose), NULL);
#endif

        /* gtk_widget_set_app_paintable (window, TRUE); */
    gtk_window_set_position (GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_window_set_title (GTK_WINDOW(window), "lines");
    gtk_window_set_default_size (GTK_WINDOW(window), view_ncols, view_nrows); 

    gtk_widget_show_all (window);
    gtk_main ();
}


int main (int argc, char* argv[])
{
    const bool use_random_scene = true;
    bool call_gui = true;
    RaySpace space;

#ifdef DistribCompute
    init_compute ();
#endif

    init_RaySpace (&space);
    zero_Point (&view_origin);

    if (use_random_scene)
    {
        PointXfrm tmp_basis;
        random_RaySpace (&space, 20);

        view_origin.coords[0] = 50;
        view_origin.coords[1] = 50;
        view_origin.coords[DirDimension] = -70;

        identity_PointXfrm (&tmp_basis);
            /* Tilt backwards a bit.*/
            /* tmp_basis.pts[0].coords[2] = -0.5; */
            /* tmp_basis.pts[0].coords[3] = -0.3; */
            /* tmp_basis.pts[0].coords[4] = -0.1; */
        orthorotate_PointXfrm (&view_basis, &tmp_basis, 1);
    }
    else
    {
        uint i;
        bool good;
        good = readin_wavefront (&space, "track_1.obj");
        if (!good)  return 1;

        view_origin.coords[0] = 0;
        view_origin.coords[1] = 0;
        view_origin.coords[DirDimension] = -10;
        identity_PointXfrm (&view_basis);

        space.nobjects = NRacers;
        space.objects = AllocT( RaySpaceObject, space.nobjects );
        UFor( i, space.nobjects )
        {
            bool good;
            init_RaySpace (&space.objects[i].space);
            good = readin_wavefront (&space.objects[i].space, "machine_1.obj");
            if (!good)  return 1;
            zero_Point (&space.objects[i].centroid);
            space.objects[i].centroid.coords[0] = 75 * i + 300;
            identity_PointXfrm (&space.objects[i].orientation);
            init_ObjectMotion (&racer_motions[i]);
        }
    }

    partition_RaySpace (&space);

#ifdef DistribCompute
    call_gui = !rays_to_hits_computeloop (&space);
#endif

    if (call_gui)
    {
        init_RayImage (&ray_image);
        ray_image.nrows = view_nrows;
        ray_image.ncols = view_ncols;
            /* ray_image.hits = AllocT( uint, 1 ); */
            /* ray_image.mags = AllocT( real, 1 ); */
        ray_image.pixels = AllocT( byte, 1 );
        resize_RayImage (&ray_image);
        ray_image.view_light = 0;
        ray_image.color_distance_on = true;

        prev_time = monotime ();
        gui_main (argc, argv, &space);
#ifdef DistribCompute
        stop_computeloop ();
#endif
    }

#ifdef DistribCompute
    cleanup_compute ();
#endif
    return 0;
}


