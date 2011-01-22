
#include "main.h"

#include <math.h>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

static real zposition = -1;

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
    tristate step = 0;
    (void) _data;

    if (event->keyval == GDK_i)  step =  1;
    if (event->keyval == GDK_o)  step = -1;

    if (step != 0)
    {
        zposition += step * 10;
        printf ("z:%f\n", zposition);
        gtk_widget_queue_draw(widget);
    }

    return TRUE;
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
    hits = AllocT( uint, nrows * ncols );
    rays_to_hits_perspective (hits, nrows, ncols,
                              space->nelems, space->selems,
                              &space->tree, zposition);

    color_diff = (guint32) 0xFFFFFF / (guint32) space->nelems;

    UFor( row, nrows )
    {
        uint* hitline;
        guint32* outline;

        hitline = &hits[ncols * row];
        outline = (guint32*) &data[stride * row];

        UFor( col, ncols )
        {
            outline[col] = (0xFF000000 |
                            (color_diff * (space->nelems - hitline[col])));
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
    GtkWidget *frame;
    GtkWidget *window;
    GtkWidget *vbox;
    GtkWidget *da;

    random_RaySpace (&space, 20);

    gtk_init (&argc, &argv);

    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

    g_signal_connect (window, "delete-event",
                      G_CALLBACK (delete_event), NULL);
    g_signal_connect (window, "destroy",
                      G_CALLBACK (destroy_app), NULL);
    g_signal_connect (window, "expose-event",
                      G_CALLBACK (render_expose), &space);
    g_signal_connect (window, "key-press-event",
                      G_CALLBACK (key_press_fn), NULL);


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
    gtk_window_set_default_size (GTK_WINDOW(window), 400, 300); 

    gtk_widget_show_all (window);
    gtk_main ();

    return 0;
}


