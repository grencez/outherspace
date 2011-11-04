
#include <GL/gl.h>
#include <GL/glu.h>

static
    void
ogl_setup ()
{
    const SDL_VideoInfo* info;
    int bpp = 0;
    GLfloat light_ambient[] = {0.2, 0.2, 0.2, 1.0};
        /* GLfloat light_ambient[] = {0, 0, 0, 1}; */
    GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
        /* GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0}; */
    GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat light_position[] = {0.0, 1.0, 0.0, 0.0};

        /* TODO: This video info does nothing, and is misplaced to boot!*/
    info = SDL_GetVideoInfo ();
    bpp = info->vfmt->BitsPerPixel;
        /* printf ("bpp:%d\n", bpp); */


    SDL_GL_SetAttribute (SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute (SDL_GL_RED_SIZE, 5);
    SDL_GL_SetAttribute (SDL_GL_GREEN_SIZE, 5);
    SDL_GL_SetAttribute (SDL_GL_BLUE_SIZE, 5);
    SDL_GL_SetAttribute (SDL_GL_DEPTH_SIZE, 16);

    glEnable (GL_LIGHT0);
    glDepthFunc (GL_LESS);
    glEnable (GL_LIGHTING);

    glEnable (GL_DEPTH_TEST);
    glEnable (GL_COLOR_MATERIAL);

    glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv (GL_LIGHT0, GL_POSITION, light_position);

    glDisable (GL_BLEND);
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

        /* Our shading model--Gouraud (smooth). */
    glShadeModel (GL_SMOOTH);

        /* Culling. */
        /* glCullFace (GL_BACK); */
        /* glFrontFace (GL_CCW); */
        /* glEnable (GL_CULL_FACE); */

}

static
    void
ogl_redraw (const RaySpace* space)
{
    uint i;
    const Pilot* pilot;
    const Scene* scene;

        /* static GLfloat red[] = { 1, 0, 0, 1 }; */

    pilot = &pilots[0];
    scene = &space->main.scene;

    glClearColor (0, 0, 0, 0);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();

    {
        uint height, width;

        height = npixelzoom * pilot->ray_image.nrows;
        width = npixelzoom * pilot->ray_image.ncols;

        glViewport (pilot->image_start_col,
                    pilot->image_start_row,
                    width, height);

        gluPerspective (180 / M_PI * pilot->view_angle,
                        width / (real) height,
                        1.0, 1024.0);
    }

        /* Clear the color and depth buffers. */
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    {
        Point p;
        diff_Point (&p, &space->main.centroid, &pilot->view_origin);
            /* proj_Point (&p, &pilot->view_basis.pts[ForwardDim], &p); */
            /* summ_Point (&p, &p, &pilot->view_origin); */
        summ_Point (&p, &pilot->view_origin, &pilot->view_basis.pts[ForwardDim]);

        gluLookAt (pilot->view_origin.coords[RightDim],
                   pilot->view_origin.coords[UpDim],
                   pilot->view_origin.coords[ForwardDim],
                   p.coords[RightDim],
                   p.coords[UpDim],
                   p.coords[ForwardDim],
                   pilot->view_basis.pts[UpDim].coords[RightDim],
                   pilot->view_basis.pts[UpDim].coords[UpDim],
                   pilot->view_basis.pts[UpDim].coords[ForwardDim]);
    }

        /* We don't want to modify the projection matrix. */
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();

#if 0
    glPushAttrib (GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, red);
#endif
    glColorMaterial (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);


        /* Send our triangle data to the pipeline. */
    glBegin (GL_TRIANGLES);
    UFor( i, scene->nelems )
    {
        uint j;
        const SceneElement* elem;

        elem = &scene->elems[i];
        if (elem->material < Max_uint)
        {
            const Material* matl = 0;
            matl = &scene->matls[elem->material];
            glColor3f(matl->ambient[0],
                      matl->ambient[1],
                      matl->ambient[2]);
        }
        else
        {
            glColor3f(1.0, 1.0, 1.0);
        }

        UFor( j, 3 )
        {
            const Point* p;
            GLfloat v[3];

            if (elem->vnmls[j] < Max_uint)
                p = &scene->vnmls[elem->vnmls[j]];
            else
                p = &space->main.simplices[i].plane.normal;

            v[0] = p->coords[RightDim];
            v[1] = p->coords[UpDim];
            v[2] = p->coords[ForwardDim];

            glNormal3fv (v);

            p = &scene->verts[elem->verts[j]];
            v[0] = p->coords[RightDim];
            v[1] = p->coords[UpDim];
            v[2] = p->coords[ForwardDim];

                /* if (j == 0)  glColor3f (1, 0, 0); */
                /* if (j == 1)  glColor3f (0, 1, 0); */
                /* if (j == 2)  glColor3f (0, 0, 1); */

            glVertex3fv (v);
        }
    }
    glEnd ();

#if 0
    glPopAttrib ();
#endif

    glFlush ();
    glFinish ();
    SDL_GL_SwapBuffers ();
}

