
#include "main.h"

#include <math.h>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

static Point view_origin;
static PointXfrm view_basis;
static uint view_nrows = 400;
static uint view_ncols = 400;
static uint mouse_coords[2];
static uint mouse_diff[2];

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
    gtk_main_quit ();
}

static
    gboolean
key_press_fn (GtkWidget* widget, GdkEventKey* event, gpointer _data)
{
    Point diff;
    tristate step = 0, roll = 0;
    tristate x_stride = 0, y_stride = 0;
    const real scale = 5;
    (void) _data;

    if (event->state & GDK_SHIFT_MASK)
    {
        if (event->keyval == GDK_Up)    y_stride =  1;
        if (event->keyval == GDK_Down)  y_stride = -1;
        if (event->keyval == GDK_Right)  x_stride =  1;
        if (event->keyval == GDK_Left)   x_stride = -1;
    }
    else
    {
        if (event->keyval == GDK_Up)    step =  1;
        if (event->keyval == GDK_Down)  step = -1;
        if (event->keyval == GDK_Right)  roll =  1;
        if (event->keyval == GDK_Left)   roll = -1;
        if (event->keyval == GDK_Escape)  gdk_pointer_ungrab (event->time);
    }

    if (step != 0)
    {
        scale_Point (&diff, &view_basis.pts[NDimensions-1], scale * step);
        summ_Point (&view_origin, &view_origin, &diff);
    }
    if (roll != 0)
    {
        PointXfrm rotation, tmp;
        rotation_PointXfrm (&rotation, 0, 1, roll * M_PI / 8);
        xfrm_PointXfrm (&tmp, &rotation, &view_basis);
        orthonormalize_PointXfrm (&view_basis, &tmp);
    }
    if (y_stride != 0)
    {
        scale_Point (&diff, &view_basis.pts[1], scale * y_stride);
        summ_Point (&view_origin, &view_origin, &diff);
    }
    if (x_stride != 0)
    {
#if 1
        scale_Point (&diff, &view_basis.pts[0], scale * x_stride);
        summ_Point (&view_origin, &view_origin, &diff);
#else
        PointXfrm rotation, tmp;
        rotation_PointXfrm (&rotation, 0, 2, x_stride * M_PI / 8);
        xfrm_PointXfrm (&tmp, &rotation, &view_basis);
        orthonormalize_PointXfrm (&view_basis, &tmp);
#endif

    }

    if (step != 0 || roll != 0 || x_stride != 0 || y_stride != 0)
    {
        FILE* out;
        out = stdout;


        fputs ("pos:", out);
        output_Point (out, &view_origin);
        fputs (" basis:", out);
        output_PointXfrm (out, &view_basis);
        fputc ('\n', out);

        gtk_widget_queue_draw (widget);
    }

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
    real x, y;
    PointXfrm tmp;
    Point dir;
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

    x = (2 * event->x - view_ncols) / view_ncols;
    y = (view_ncols - 2 * event->y) / view_nrows;

    fprintf (out, "x:%f  y:%f\n", x, y);

    copy_PointXfrm (&tmp, &view_basis);
    zero_Point (&dir);
    dir.coords[0] = x * cos (M_PI / 3);
    dir.coords[1] = y * cos (M_PI / 3);
    dir.coords[2] = 1;
    trxfrm_Point (&tmp.pts[2], &view_basis, &dir);
    orthorotate_PointXfrm (&view_basis, &tmp, 2);

    fputs (" basis:", out);
    output_PointXfrm (out, &view_basis);
    fputc ('\n', out);
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

#if 0
#elif 0
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

#elif 1

static
    void
render_RaySpace (byte* data, const RaySpace* space,
                 uint nrows, uint ncols, uint stride)
{
    uint* hits;
    guint32 color_diff;
    uint row, col;

    hits = AllocT( uint, view_nrows * view_ncols );
    fprintf (stderr, "nrows:%u  ncols:%u\n", view_nrows, view_ncols);
#if 0
#elif 0
    rays_to_hits_perspective (hits, view_nrows, view_ncols,
                              space, view_origin.coords[2]);
#elif 1
    rays_to_hits (hits, view_nrows, view_ncols,
                  space, &view_origin, &view_basis);
#endif

    color_diff = (guint32) 0xFFFFFF / (guint32) space->nelems;

    UFor( row, view_nrows )
    {
        uint* hitline;
        guint32* outline;
        if (row >= nrows)  break;

        hitline = &hits[view_ncols * row];
        outline = (guint32*) &data[stride * row];

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

    free (hits);
}
#endif

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

#if 0
#elif 0
    render_pattern (data, (void*) state, height, width, stride);
#elif 1
    render_RaySpace (data, (RaySpace*) state,
                     (uint) height, (uint) width, (uint) stride);
#endif

    cairo_surface_mark_dirty (surface);
    cr = gdk_cairo_create (da->window);
    cairo_set_source_surface (cr, surface, 0, 0);
    cairo_paint (cr);

    cairo_surface_destroy (surface);
    cairo_destroy (cr);
    return TRUE;
}


int main (int argc, char* argv[])
{
    RaySpace space;

        /* GtkWidget is the storage type for widgets */
        /* GtkWidget *frame; */
    GtkWidget *window;
        /* GtkWidget *vbox; */
        /* GtkWidget *da; */

    zero_Point (&view_origin);

#if 0
    random_RaySpace (&space, 20);
    
    view_origin.coords[0] = 50;
    view_origin.coords[1] = 50;
    view_origin.coords[2] = -70;
    identity_PointXfrm (&view_basis);
#else
    {
        bool good = readin_wavefront (&space, "mba2.obj");
        if (!good)  return 1;
    }

    view_origin.coords[0] = 0;
    view_origin.coords[1] = 10;
    view_origin.coords[2] = -250;

    {
        PointXfrm tmp_basis;
        identity_PointXfrm (&tmp_basis);
        tmp_basis.pts[1].coords[2] = -0.5;  /* Tilt backwards a bit.*/
        orthorotate_PointXfrm (&view_basis, &tmp_basis, 1);
    }

#endif

    build_KDTree (&space.tree, space.nelems, space.elems, &space.scene.box);
        /* output_KDTree (stderr, &space.tree, space.nelems, space.elems); */

    gtk_init (&argc, &argv);

    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_widget_add_events (window, GDK_BUTTON_PRESS_MASK);

    g_signal_connect (window, "delete-event",
                      G_CALLBACK(delete_event), NULL);
    g_signal_connect (window, "destroy",
                      G_CALLBACK(destroy_app), NULL);
    g_signal_connect (window, "expose-event",
                      G_CALLBACK(render_expose), &space);
    g_signal_connect (window, "key-press-event",
                      G_CALLBACK(key_press_fn), NULL);
    g_signal_connect (window, "button-press-event",
                      G_CALLBACK(grab_mouse_fn), NULL);


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

    return 0;
}


