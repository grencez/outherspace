
#include <GL/gl.h>
#include <GL/glu.h>

static void
ogl_redraw_ObjectRaySpace (const ObjectRaySpace* object);

    /** Convert a Point to a 3-vec.**/
static
    void
ogl_vec_Point (GLdouble* dst, const Point* src)
{
    dst[0] = src->coords[RightDim];
    dst[1] = src->coords[UpDim];
    dst[2] = - src->coords[ForwardDim];
}

    /** Convert a Point to a 3-vec.**/
static
    void
ogl_fvec_Point (GLfloat* dst, const Point* src)
{
    dst[0] = src->coords[RightDim];
    dst[1] = src->coords[UpDim];
    dst[2] = - src->coords[ForwardDim];
}

    /** Convert to an OpenGL-style matrix.**/
static
    void
ogl_matrix (GLdouble* dst, const Point* origin, const PointXfrm* basis)
{
    uint i;
    ogl_vec_Point (&dst[0], &basis->pts[RightDim]);
    ogl_vec_Point (&dst[4], &basis->pts[UpDim]);
    ogl_vec_Point (&dst[8], &basis->pts[ForwardDim]);
    UFor( i, 3 )  dst[8+i] = - dst[8+i];
    ogl_vec_Point (&dst[12], origin);

    dst[3] = dst[7] = dst[11] = 0;
    dst[15] = 1;
}

static
    void
ogl_setup (const RaySpace* space)
{
    const SDL_VideoInfo* info;
    int bpp = 0;
    GLfloat light_ambient[] = {0.2, 0.2, 0.2, 1.0};
        /* GLfloat light_ambient[] = {0, 0, 0, 1}; */
    GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
        /* GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0}; */
    GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat light_position[] = {0.0, 1.0, 0.0, 0.0};
    const GLenum light_idcs[8] =
    {   GL_LIGHT0, GL_LIGHT1, GL_LIGHT2, GL_LIGHT3,
        GL_LIGHT4, GL_LIGHT5, GL_LIGHT6, GL_LIGHT7
    };

        /* TODO: This video info does nothing, and is misplaced to boot!*/
    info = SDL_GetVideoInfo ();
    bpp = info->vfmt->BitsPerPixel;
        /* printf ("bpp:%d\n", bpp); */


    SDL_GL_SetAttribute (SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute (SDL_GL_RED_SIZE, 5);
    SDL_GL_SetAttribute (SDL_GL_GREEN_SIZE, 5);
    SDL_GL_SetAttribute (SDL_GL_BLUE_SIZE, 5);
    SDL_GL_SetAttribute (SDL_GL_DEPTH_SIZE, 16);

    assert (space->nlights < 8);
    if (space->nlights > 0)
    {
        uint i;
            /* for (i = 0; i < 8; ++i) */
        for (i = 0; i < 1; ++i)
        {
            if (i >= space->nlights)
            {
                glDisable (light_idcs[i]);
                continue;
            }

            ogl_fvec_Point (light_position, &space->lights[i].location);
            glLightfv (light_idcs[i], GL_AMBIENT, light_ambient);
            glLightfv (light_idcs[i], GL_DIFFUSE, light_diffuse);
            glLightfv (light_idcs[i], GL_SPECULAR, light_specular);
            glLightfv (light_idcs[i], GL_POSITION, light_position);
            glEnable (light_idcs[i]);
        }
        glEnable (GL_LIGHTING);
    }
    else
    {
        glDisable (GL_LIGHTING);
    }

    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
        /* glEnable (GL_COLOR_MATERIAL); */

#if 0
        /* This works.*/
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
#endif
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

        /* Our shading model--Gouraud (smooth). */
    glShadeModel (GL_SMOOTH);

        /* Culling. */
        /* glCullFace (GL_BACK); */
        /* glFrontFace (GL_CCW); */
        /* glEnable (GL_CULL_FACE); */

}

    /** Render using OpenGL.**/
static
    void
ogl_redraw (const RaySpace* space, uint pilot_idx)
{
    uint i;
    uint height, width;
    const Pilot* pilot;

    pilot = &pilots[pilot_idx];

    glClearColor (0, 0, 0, 0);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();

    height = npixelzoom * pilot->ray_image.nrows;
    width = npixelzoom * pilot->ray_image.ncols;

    glViewport (npixelzoom * pilot->image_start_col,
                npixelzoom * pilot->image_start_row,
                width, height);

        /* Clear the depth buffer. */
    glClear (GL_DEPTH_BUFFER_BIT);

    if (pilot->ray_image.perspective)
    {
        Point p;

        gluPerspective (180 / M_PI * pilot->ray_image.hifov,
                        width / (real) height,
                        1.0, 3 * magnitude_Point (&space->main.box.max));

            /* diff_Point (&p, &object->centroid, &pilot->view_origin); */
            /* proj_Point (&p, &pilot->view_basis.pts[ForwardDim], &p); */
            /* summ_Point (&p, &p, &pilot->view_origin); */
        summ_Point (&p, &pilot->view_origin, &pilot->view_basis.pts[ForwardDim]);

        gluLookAt (pilot->view_origin.coords[RightDim],
                   pilot->view_origin.coords[UpDim],
                   - pilot->view_origin.coords[ForwardDim],
                   p.coords[RightDim],
                   p.coords[UpDim],
                   - p.coords[ForwardDim],
                   pilot->view_basis.pts[UpDim].coords[RightDim],
                   pilot->view_basis.pts[UpDim].coords[UpDim],
                   - pilot->view_basis.pts[UpDim].coords[ForwardDim]);
    }
    else
    {
        uint max_n;
        real horz, vert;
        GLdouble matrix[16];
        if (width >= height)  max_n = width;
        else                  max_n = height;
        horz = .5 * pilot->ray_image.hifov * (width / (real) max_n);
        vert = .5 * pilot->ray_image.hifov * (height / (real) max_n);

        glOrtho (-horz, horz, -vert, vert,
                 1, 3 * magnitude_Point (&space->main.box.max));

        ogl_matrix (matrix, &pilot->view_origin, &pilot->view_basis);
        glMultMatrixd (matrix);
    }

        /* We don't want to modify the projection matrix. */
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();

    ogl_redraw_ObjectRaySpace (&space->main);
    UFor( i, space->nobjects )
        ogl_redraw_ObjectRaySpace (&space->objects[i]);
}

    void
ogl_redraw_ObjectRaySpace (const ObjectRaySpace* object)
{
    Material default_material;
    GLdouble matrix[16];
    const Scene* scene;
    uint i;

    if (!object->visible)  return;

    scene = &object->scene;

    glPushMatrix ();

    ogl_matrix (matrix, &object->centroid, &object->orientation);

    glMultMatrixd (matrix);

        /* Send our triangle data to the pipeline. */
    glBegin (GL_TRIANGLES);
    UFor( i, scene->nelems )
    {
        const SceneElement* elem;
        uint j;

        elem = &scene->elems[i];
        if (i == 0 || elem->material != scene->elems[i-1].material)
        {
            const Material* matl;
            GLfloat color[4];
            if (elem->material < scene->nmatls)
                matl = &scene->matls[elem->material];
            else
                matl = &default_material;

            color[3] = matl->opacity;
            UFor( j, 3 )  color[j] = matl->ambient[j];
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);

            UFor( j, 3 )  color[j] = matl->diffuse[j];
            glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);

            UFor( j, 3 )  color[j] = matl->specular[j];
            glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, color);

            glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS,
                         matl->optical_density);
        }

        UFor( j, 3 )
        {
            const Point* p;
            GLdouble v[3];

            if (elem->vnmls[j] < scene->nvnmls)
                p = &scene->vnmls[elem->vnmls[j]];
            else
                p = &object->simplices[i].plane.normal;

            ogl_vec_Point (v, p);

            glNormal3dv (v);

            p = &scene->verts[elem->verts[j]];
            ogl_vec_Point (v, p);

                /* if (j == 0)  glColor3f (1, 0, 0); */
                /* if (j == 1)  glColor3f (0, 1, 0); */
                /* if (j == 2)  glColor3f (0, 0, 1); */

            glVertex3dv (v);
        }
    }
    glEnd ();

    glPopMatrix ();
}

