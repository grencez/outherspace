
#include <GL/gl.h>
#include <GL/glu.h>

static
    void
setup_opengl (int width, int height)
{
    const Pilot* pilot;
    float ratio = (float) width / (float) height;

    pilot = &pilots[0];

        /* Our shading model--Gouraud (smooth). */
        /* glShadeModel( GL_SMOOTH ); */

        /* Culling. */
        /* glCullFace( GL_BACK ); */
        /* glFrontFace( GL_CCW ); */
        /* glEnable( GL_CULL_FACE ); */

        /* Set the clear color. */
    glClearColor (0, 0, 0, 0);

        /* Setup our viewport. */
    glViewport (0, 0, width, height);

        /*
         * Change to the projection matrix and set
         * our viewing volume.
         */
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
        /*
         * EXERCISE:
         * Replace this with a call to glFrustum.
         */
    gluPerspective (180 / M_PI * pilot->view_angle,
                    ratio, 1.0, 1024.0);
}

static
    void
ogl_redraw (const RaySpace* space)
{
    uint i;
    const Pilot* pilot;
    const Scene* scene;

    static GLfloat red[] = { 1, 0, 0, 1 };

    pilot = &pilots[0];
    scene = &space->main.scene;

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

    glColorMaterial (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

        /* We don't want to modify the projection matrix. */
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity( );

    glPushAttrib (GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glMaterialfv (GL_FRONT, GL_DIFFUSE, red);
        /* glColor3f(1.0, 0.0, 0.0); */


        /* Send our triangle data to the pipeline. */
    glBegin (GL_TRIANGLES);
    UFor( i, scene->nelems )
    {
        uint j;
        const SceneElement* elem;
        elem = &scene->elems[i];
        UFor( j, 3 )
        {
            const Point* p;
            GLfloat v[3];

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

    glPopAttrib ();


    glEnd ();
    glFlush ();
    glFinish ();
    SDL_GL_SwapBuffers ();
}

