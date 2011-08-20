
#ifdef DistribCompute
#include "compute.h"
#endif

    /* Note: important stuff in this file!*/
#include "gui-indep.c"

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <SDL.h>
#include <SDL_thread.h>
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


static const bool
#ifdef RunFromMyMac
#undef RunFromMyMac
RunFromMyMac = true;
#else
RunFromMyMac = false;
#endif


static SDL_cond* pilots_initialized = 0;

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
        render_pattern (data, (uint) height, (uint)  width, (uint) stride);
    else
        render_RaySpace ((RaySpace*) state,
                         data, (uint) height, (uint) width, (uint) stride,
                         (real) SDL_GetTicks () / 1000);

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

    void
redraw_loop (RaySpace* space)
{
    SDL_Surface* screen;
    screen = SDL_SetVideoMode (view_ncols, view_nrows, 32, SDL_HWSURFACE);

    while (true)
    {
        if (SDL_MUSTLOCK (screen) && SDL_LockSurface (screen) < 0)  break;


    if (RenderDrawsPattern)
        render_pattern (screen->pixels, screen->h, screen->w, screen->pitch);
    else
        render_RaySpace (space, (byte*)screen->pixels,
                         screen->h, screen->w, screen->pitch,
                         (real) SDL_GetTicks () / 1000);

        if (SDL_MUSTLOCK (screen))  SDL_UnlockSurface (screen);
        SDL_Flip (screen);
    }
}

    uint32
keepalive_sdl_event_fn (uint32 interval, void* param)
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

    int
sdl_main (void* data)
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
    SDL_CondSignal (pilots_initialized);

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
        SDL_Thread* sdl_thread;
        SDL_mutex* initializing_lock;

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

        pilots_initialized = SDL_CreateCond ();

            /* Note: SDL_INIT_VIDEO is required for some silly reason!*/
        ret = SDL_Init (SDL_INIT_VIDEO |
#ifdef SupportSound
                        SDL_INIT_AUDIO |
#endif
                        SDL_INIT_EVENTTHREAD |
                        SDL_INIT_JOYSTICK |
                        SDL_INIT_TIMER);
                        
        assert (ret == 0);

        sdl_thread = SDL_CreateThread (sdl_main, inpathname);

        initializing_lock = SDL_CreateMutex ();
        SDL_mutexP (initializing_lock);
        SDL_CondWait (pilots_initialized, initializing_lock);
        SDL_mutexV (initializing_lock);
        SDL_DestroyMutex (initializing_lock);
        SDL_DestroyCond (pilots_initialized);


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

        SDL_WaitThread (sdl_thread, 0);
        SDL_Quit ();
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

