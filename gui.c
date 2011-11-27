
    /* Note: important stuff in this file!*/
#include "gui-indep.c"

#include <SDL.h>
#ifdef SupportImage
#include <SDL_image.h>
#endif
#ifdef SupportSound
#include <SDL_mixer.h>
#endif

#ifdef SupportHaptic
#include <SDL_haptic.h>
#endif

#ifdef SupportOpenGL
#include "gui-opengl.c"
#endif

    /* SDL on OS X does some weirdo bootstrapping by redefining /main/.*/
#ifdef main
#undef main
#endif

#ifdef RunFromMyMac
#undef RunFromMyMac
static const bool RunFromMyMac = true;
static const bool SeparateRenderThread = false;
extern int wrapped_main_fn (int argc, char* argv[]);
#else
static const bool RunFromMyMac = false;
# ifdef SupportOpenGL
static const bool SeparateRenderThread = false;
# else
static const bool SeparateRenderThread = true;
# endif
#define wrapped_main_fn main
#endif

static bool needs_recast = true;
static uint resize_nrows = 0;
static uint resize_ncols = 0;

struct redraw_loop_param_struct
{
    RaySpace* space;
    SDL_sem* sig;
    bool continue_running;
    bool resize;
};
typedef struct redraw_loop_param_struct RedrawLoopParam;


static
    void
key_press_fn (Pilot* pilot, const SDL_keysym* event)
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
    tristate nperpixel_change = 0;  /* Magnify.*/
    bool reflect = false;
    bool rotate_dir_dim = false;
    bool change_cast_method = false;
    bool print_view_code = false;
    bool print_plane_line = false;
    bool quit_app = false;
    bool recast = true;
    FILE* out = stdout;
    bool shift_mod, ctrl_mod;
    MotionInput* input;
    Point* view_origin;
    PointXfrm* view_basis;
    real stride_magnitude;
    RayImage* ray_image;
    uint craft_idx;
    ObjectMotion* racer_motion;

    input = &pilot->input;
    view_origin = &pilot->view_origin;
    view_basis = &pilot->view_basis;
    stride_magnitude = pilot->stride_magnitude;
    ray_image = &pilot->ray_image;
    craft_idx = pilot->craft_idx;
    racer_motion = &racer_motions[craft_idx];

    shift_mod = 0 != (event->mod & KMOD_SHIFT);
    ctrl_mod = 0 != (event->mod & KMOD_CTRL);

    switch (event->sym)
    {
        case SDLK_UP:
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
        case SDLK_DOWN:
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
        case SDLK_RIGHT:
            if (shift_mod) { dim = RightDim;  stride = 1; }
            else if (ctrl_mod) { roll = -1; }
            else if (FollowRacer) { dim = RightDim;  turn = 1; }
            else { roll = -1; }
            break;
        case SDLK_LEFT:
            if (shift_mod) { dim = RightDim;  stride = -1; }
            else if (ctrl_mod) { roll = 1; }
            else if (FollowRacer) { dim = RightDim;  turn = -1; }
            else { roll = 1; }
            break;
        case SDLK_TAB:
            if (shift_mod)  switch_racer = -1;
            else            switch_racer =  1;
            break;
        case SDLK_a:
            dim = ForwardDim; stride = 1; break;
        case SDLK_c:
            racer_motion->collide = !racer_motion->collide;
            break;
        case SDLK_d:
            if (shift_mod)
            {
                if (NDimensions == 4) { dim = DriftDim; stride = 1; }
            }
            else
            {
                if (ctrl_mod) { rotate_dir_dim = true; }
                else if (NDimensions == 4) { dim = DriftDim; stride = -1; }
            }
            break;
        case SDLK_e:
            input->boost = true;
            break;
        case SDLK_f:
            if (shift_mod)  switch_kbd_pilot = -1;
            else            switch_kbd_pilot =  1;
            break;
        case SDLK_g:
            racer_motion->gravity = !racer_motion->gravity;
            break;
        case SDLK_l:
            if (shift_mod)  view_light_change =  1;
            else            view_light_change = -1;
            break;
        case SDLK_m:
            if (shift_mod)  nperpixel_change =  1;
            else            nperpixel_change = -1;
        case SDLK_p:
            if (shift_mod)  print_view_code = true;
            else            print_plane_line = true;
            break;
        case SDLK_q:
            if (ctrl_mod)  quit_app = true;
            break;
        case SDLK_r:
            reflect = true;  break;
        case SDLK_s:
            if (shift_mod)  stride_mag_change =  1;
            else            stride_mag_change = -1;
            break;
        case SDLK_v:
            if (ctrl_mod)  change_cast_method = true;
            else if (shift_mod)  view_angle_change = 1;
            else  view_angle_change = -1;
            break;
        case SDLK_z:
            if (shift_mod)  resize =  1;
            else            resize = -1;
            break;
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
        if (resize_nrows == 0)
        {
            resize_nrows = view_nrows;
            resize_ncols = view_ncols;
        }
        if (resize > 0)
        {
            resize_nrows += diff;
            resize_ncols += diff;
        }
        else if (resize_nrows > diff)
        {
            resize_nrows -= diff;
            resize_ncols -= diff;
        }
        else
        {
            recast = false;
        }
    }
    else if (nperpixel_change != 0)
    {
        if (nperpixel_change > 0)  npixelzoom += 1;
        else if (npixelzoom > 1)   npixelzoom -= 1;
        else                       recast = false;

        if (recast && resize_nrows == 0)
        {
            resize_nrows = view_nrows;
            resize_ncols = view_ncols;
        }
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
    else if (print_plane_line)
    {
        fprintf (out, "%f %f %f  %f %f %f\n",
                 view_origin->coords[0],
                 view_origin->coords[1],
                 view_origin->coords[2],
                 view_basis->pts[ForwardDim].coords[0],
                 view_basis->pts[ForwardDim].coords[1],
                 view_basis->pts[ForwardDim].coords[2]);
    }
    else if (quit_app)
    {
        SDL_Event ev;
        ev.type = SDL_QUIT;
        SDL_PushEvent (&ev);
        recast = false;
    }
    else
    {
        recast = false;
    }


    if (recast)  needs_recast = true;
}

static
    void
key_release_fn (MotionInput* input, const SDL_keysym* event)
{
    switch (event->sym)
    {
        case SDLK_UP:
        case SDLK_DOWN:
            input->vert = 0;
            break;
        case SDLK_RIGHT:
        case SDLK_LEFT:
            input->horz = 0;
            break;
        case SDLK_a:
            input->stride[ForwardDim] = 0;
            break;
        case SDLK_e:
            input->boost = false;
            break;
        default:
            break;
    }
}


static
    void
joystick_button_event (MotionInput* mot, uint btnidx, bool pressed)
{
    tristate value;
#if 0
    FILE* out = stderr;
    fprintf (out, "\rbtnidx:%u  pressed:%u",
             btnidx, (uint) pressed);
#endif

    value = pressed ? 1 : 0;

    if (btnidx == 0)  mot->boost = pressed;
    if (btnidx == 1)  mot->lock_drift = pressed;
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


static
    void
grab_mouse_fn (const SDL_MouseButtonEvent* event, const RaySpace* space)
{
    uint row, col;
    FILE* out = stdout;
    bool do_rotate = false;
    bool do_raycast = false;
    RayCastAPriori priori;
    Point origin, dir;
    Pilot* pilot;

    pilot = &pilots[kbd_pilot_idx];

    row = event->y / npixelzoom;
    col = event->x / npixelzoom;
    if (event->y < (int)pilot->image_start_row ||
        event->x < (int)pilot->image_start_col)
    {
        fputs ("Stay in the box hoser!\n", out);
    }

    row -= pilot->image_start_row;
    col -= pilot->image_start_col;
    if (pilot->ray_image.nrows <= row ||
        pilot->ray_image.ncols <= col)
    {
        fputs ("Stay in the box hoser!\n", out);
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
        copy_Point (&pilot->view_origin, &origin);

        orthorotate_PointXfrm (&pilot->view_basis, &pilot->view_basis,
                               &dir, ForwardDim);

        fputs ("basis:", out);
        output_PointXfrm (out, &pilot->view_basis);
        fputc ('\n', out);
        needs_recast = true;

        pilot->mouse_coords[0] = event->x;
        pilot->mouse_coords[1] = event->y;
        pilot->mouse_diff[0] = 0;
        pilot->mouse_diff[1] = 0;
    }
    else if (do_raycast)
    {
        uint hit_idx = Max_uint;
        real hit_mag = Max_real;
        uint hit_objidx = Max_uint;

            /* Use /cast_partitioned/ here and
             * set /SeparateRenderThread/ to false
             * to cast rays matching exactly what you see.
             */
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
}

static
    void
sdl_redraw (SDL_Surface* screen)
{
    const bool argb_order = !RunFromMyMac;
    if (SDL_MUSTLOCK (screen) && SDL_LockSurface (screen) < 0)  return;

    if (RenderDrawsPattern)
        render_pattern ((byte*)screen->pixels,
                        screen->h, screen->w, screen->pitch, argb_order);
    else
        render_pilot_images ((byte*)screen->pixels,
                             screen->h, screen->w, screen->pitch, argb_order);

    if (SDL_MUSTLOCK (screen))  SDL_UnlockSurface (screen);
    SDL_Flip (screen);
}

static
    int
render_loop_fn (void* data)
{
    SDL_Event draw_event;
    RedrawLoopParam* param;
    RaySpace* space;

    param = (RedrawLoopParam*) data;
    space = param->space;

    draw_event.type = SDL_USEREVENT;
    draw_event.user.code = 2;
    draw_event.user.data1 = 0;
    draw_event.user.data2 = 0;

    while (param->continue_running)
    {
        param->resize = (resize_nrows > 0);
        if (param->resize)
        {
            view_nrows = resize_nrows;
            view_ncols = resize_ncols;
            resize_nrows = resize_ncols = 0;
            resize_pilot_viewports (view_nrows, view_ncols);
        }
        if (!RenderDrawsPattern && needs_recast)
        {
            needs_recast = false;
            update_pilot_images (space, (real) SDL_GetTicks () / 1000);
#ifdef SupportOpenGL
            SDL_GL_SwapBuffers ();
            glFlush ();
                /* glFinish (); */
            glClear (GL_COLOR_BUFFER_BIT);
#endif
        }

        SDL_PushEvent (&draw_event);
        if (SeparateRenderThread)
            SDL_SemWait (param->sig);
        else
            break;
    }

    return 0;
}

static
    void
sdl_main (RaySpace* space, const char* pathname)
{
    RedrawLoopParam redraw_loop_param;
    SDL_Event event;
        /* SDL_TimerID timer; */
    SDL_Thread* render_thread = 0;
    SDL_Surface* icon = 0;
    SDL_Surface* screen = 0;
    SDL_Joystick* joysticks[NRacersMax]; 
#ifdef SupportHaptic
    SDL_Haptic* haptics[NRacersMax];
    SDL_HapticEffect haptic_effect;
    int haptic_id;
#endif

    uint i, njoysticks;
    int ret;
#ifdef SupportSound
    Mix_Chunk* tune = 0;
    int channel = -1;
    FILE* out = stderr;
#endif
    RedrawLoopParam* param;

    param = &redraw_loop_param;

    {
        char* iconpath;
        iconpath = cat_filepath (pathname, "icon.bmp");
        icon = SDL_LoadBMP (iconpath);
        free (iconpath);
    }

        /* Note: SDL_INIT_VIDEO is required for some silly reason!
         * (This note written when GTK was used for display.)
         */
    ret = SDL_Init (SDL_INIT_VIDEO |
#ifdef SupportSound
                    SDL_INIT_AUDIO |
#endif
                            /* SDL_INIT_EVENTTHREAD DOES NOT WORK!*/
#ifdef SupportHaptic
                    SDL_INIT_HAPTIC |
#endif
                    SDL_INIT_JOYSTICK |
                    SDL_INIT_TIMER);
    assert (ret == 0);
        /* Get a fast key repeat.*/
    SDL_EnableKeyRepeat(50, 50);

    SDL_JoystickEventState (SDL_ENABLE);
    njoysticks = SDL_NumJoysticks ();

    UFor( i, njoysticks )
    {
        joysticks[i] = SDL_JoystickOpen (i);
        fprintf (stderr, "Joystick %u has %d buttons!\n",
                 i, SDL_JoystickNumButtons (joysticks[i]));

#ifdef SupportHaptic
        haptics[i] = (SDL_JoystickIsHaptic (joysticks[i]))
            ? SDL_HapticOpenFromJoystick (joysticks[i])
            : 0;
#endif
    }

    if (njoysticks > 0)  npilots = njoysticks;
    init_ui_data (space, pathname);
        /* Assure pilot views will be initialized (trigger a resize).*/
    resize_nrows = npilots * view_nrows;
    resize_ncols = view_ncols;

        /* Allow the render thread to begin.*/
    param->space = space;
    param->continue_running = true;
    if (SeparateRenderThread)
    {
        param->sig = SDL_CreateSemaphore (0);
        render_thread = SDL_CreateThread (render_loop_fn, param);
    }
    else
    {
        render_loop_fn (param);
    }

#ifdef SupportHaptic
    memset (&haptic_effect, 0, sizeof (SDL_HapticEffect));
        /* supported = SDL_HapticQuery (haptics[0]); */
        /* fprintf (stderr, "supported:%x\n", supported); */
        /* if (supported & SDL_HAPTIC_SQUARE) */
    haptic_effect.type = SDL_HAPTIC_SQUARE;
        /* haptic_effect.periodic.period = 500; */
    haptic_effect.periodic.period = 800;
    haptic_effect.periodic.magnitude = 0x1000;
    haptic_effect.periodic.length = SDL_HAPTIC_INFINITY;
        /* haptic_effect.periodic.attack_length = 1000; */
        /* haptic_effect.periodic.fade_length = 1000; */

    haptic_id = SDL_HapticNewEffect (haptics[0], &haptic_effect);
    if (haptic_id < 0)
        fprintf (stderr, "Unable to upload haptic effect: %s.\n", SDL_GetError());
    else SDL_HapticRunEffect (haptics[0], haptic_id, 1);
#endif


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

        buf = cat_filepath (pathname, fname);
        tune = Mix_LoadWAV (buf);

        if (!tune)
            fprintf (out, "No input music file:%s!\n", buf);
        else
            channel = Mix_PlayChannel (-1, tune, -1);

        free (buf);
    }
#endif

    SDL_WM_SetCaption ("OuTHER SPACE", 0);
    if (icon)  SDL_WM_SetIcon (icon, 0);

        /* Add a timer to assure the event loop has
         * an event to process when we should exit.
         * Also, so updates can occur.
         */
        /* timer = SDL_AddTimer (10, keepalive_sdl_event_fn, 0); */
        /* assert (timer); */

    while (param->continue_running && SDL_WaitEvent (&event))
    {
            /* Loop here when single threaded so all events can be processed
             * before committing to a render.
             */
        do  switch (event.type)
        {
            case SDL_KEYDOWN:
                key_press_fn (&pilots[kbd_pilot_idx],
                              &event.key.keysym);
                break;
            case SDL_KEYUP:
                key_release_fn (&pilots[kbd_pilot_idx].input,
                                &event.key.keysym);
                break;
            case SDL_MOUSEBUTTONDOWN:
                grab_mouse_fn (&event.button, space);
                break;
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
            case SDL_VIDEORESIZE:
                resize_nrows = event.resize.h / npixelzoom;
                resize_ncols = event.resize.w / npixelzoom;
                break;
            case SDL_USEREVENT:
                if (param->resize) {
                    screen = SDL_SetVideoMode (npixelzoom * view_ncols,
                                               npixelzoom * view_nrows,
                                               32,
#ifdef SupportOpenGL
                                               SDL_OPENGL|
#else
                                               SDL_DOUBLEBUF|
#endif
                                               SDL_HWSURFACE|
                                               SDL_RESIZABLE);
#ifdef SupportOpenGL
                    ogl_setup (space);
#endif
                }


                UFor( i, njoysticks )
                {
                    const ObjectMotion* motion;
                    real mag;
                    motion = &racer_motions[pilots[i].craft_idx];
                    mag = magnitude_Point (&motion->veloc);

#ifdef SupportHaptic
                        /* haptic_effect.periodic.period = (uint) (2000 / (1+mag)); */
                    haptic_effect.periodic.magnitude = (uint) (0x8000 * (mag / 4000));
                        /* printf ("period: %u\n", haptic_effect.periodic.period); */
                    SDL_HapticUpdateEffect (haptics[i], haptic_id, &haptic_effect);
                        /* SDL_HapticRunEffect (haptics[i], haptic_id, 1); */
#endif
                }

#ifdef SupportOpenGL
                    /* When OpenGL is being used,
                     * render_loop_fn() actually does the drawing also.
                     */
                (void) sdl_redraw;
#else
                sdl_redraw (screen);
#endif

                needs_recast = true;
                    /* When there is a separate render thread,
                     * it is timely to signal a render at this point.
                     */
                if (SeparateRenderThread)  SDL_SemPost (param->sig);
                break;
            case SDL_QUIT:
                param->continue_running = false;
                if (SeparateRenderThread)  SDL_SemPost (param->sig);
                break;
            default:
                break;

        } while (!SeparateRenderThread && SDL_PollEvent (&event));

        if (!SeparateRenderThread && needs_recast)
            render_loop_fn (param);
    }

        /* SDL_RemoveTimer (timer); */

#ifdef SupportSound
    if (channel >= 0)
        Mix_HaltChannel (channel);
    if (tune)
        Mix_FreeChunk (tune);
    Mix_CloseAudio();
#endif

    UFor( i, njoysticks )
    {
#ifdef SupportHaptic
        if (haptics[i])
        {
            SDL_HapticStopEffect (haptics[i], haptic_id);
            SDL_HapticClose (haptics[i]);
        }
#endif
        SDL_JoystickClose (joysticks[i]);
    }

    if (SeparateRenderThread)
    {
        SDL_WaitThread (render_thread, 0);
        SDL_DestroySemaphore (param->sig);
    }

    SDL_Quit ();
    if (icon)  SDL_FreeSurface (icon);
}

int wrapped_main_fn (int argc, char* argv[])
{
    FILE* out = stderr;
    bool good = true;
    bool call_gui = true;
    uint i;
#ifdef SupportImage
    int ret;
#endif
    RaySpace ray_space;
    char inpathname[1024];
    Point view_origin;
    PointXfrm view_basis;
    real view_angle;
    RaySpace* space;

    if (argc > 1)
        fputs ("No options for this program, ignoring.\n", out);

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

    space = &ray_space;

#ifdef DistribCompute
    init_compute (&argc, &argv);
#endif

#ifdef SupportImage
    ret = IMG_Init (IMG_INIT_PNG);
    assert (ret == IMG_INIT_PNG);
#endif

    init_Track (&track);

    good =
#if 0
        setup_testcase_simple (space, &view_origin,
                               &view_basis, &view_angle,
                               inpathname, "machine0.obj");
#elif 1
    readin_Track (&track, space, inpathname, "curve-track.txt");
    identity_PointXfrm (&view_basis);
    zero_Point (&view_origin);
    view_angle = 2 * M_PI / 3;
#else
#if 0
#elif 0
        setup_testcase_triangles
#elif 1
        setup_testcase_track
#elif 0
        setup_testcase_4d_normals
#elif 0
        setup_testcase_manual_interp
#endif
        (space, &view_origin, &view_basis, &view_angle,
         inpathname);
#endif  /* Pre-rolled testcase.*/

    if (!good)
    {
        fputs ("Setup failed!\n", out);
        return 1;
    }

#ifdef DistribCompute
    call_gui = !rays_to_hits_computeloop (space);
#endif

    if (call_gui)
    {
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

        sdl_main (space, inpathname);

        cleanup_ui_data ();
#ifdef DistribCompute
        stop_computeloop ();
#endif
    }

#ifdef SupportImage
    IMG_Quit ();
#endif

#ifdef DistribCompute
    cleanup_compute ();
#endif

    cleanup_RaySpace (space);
    cleanup_Track (&track);

    return 0;
}

