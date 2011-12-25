
#ifdef _WIN32
# include <windows.h>
#endif

#define GL_GLEXT_PROTOTYPES
#include <SDL_opengl.h>

#ifdef main
# undef main
#endif

#if NDimensions != 4
    /* Force Match4dGeom to imply NDimensions == 4
     * since it is assumed in this file.
     */
# ifdef Match4dGeom
#  undef Match4dGeom
# endif
#endif

static void
ogl_redraw_ObjectRaySpace (const ObjectRaySpace* object,
                           const Point* view_origin,
                           const PointXfrm* view_basis);
#ifdef Match4dGeom
static uint
view_element (GLdouble* ret_verts,
              GLdouble* ret_vnmls,
              const SceneElement* elem,
              const Scene* scene,
              const AffineMap* map,
              const Point* normal);
#endif

GLuint vert_shader;
GLuint frag_shader;
GLuint shader_program;

    /** Convert a Point to a 3-vec.**/
static
    void
ogl_vec_Point (GLdouble* dst, const Point* src)
{
    dst[0] = src->coords[0];
    dst[1] = src->coords[1];
    dst[2] = src->coords[2];
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
    PointXfrm m;
    Point p;
    uint perms[NDimensions];
    uint i;

    UFor( i, NDimensions )  perms[i] = 2 * i;
    perms[0] = 2 * RightDim;
    perms[1] = 2 * UpDim;
    perms[2] = 2 * ForwardDim + 1;
    permutation_PointXfrm (&m, perms);

    xfrm_Point (&p, &m, origin);
    xfrm_PointXfrm (&m, basis, &m);

    UFor( i, 3 )
    {
        uint j;
        UFor( j, 3 )
            dst[4 * i + j] = m.pts[i].coords[j];
        dst[4 * i + 3] = 0;
        dst[4 * 3 + i] = p.coords[i];
    }
    dst[15] = 1;
}

static
    void
init_ogl_ui_data ()
{
#include "phong.glsl.h"
    GLint status = GL_NO_ERROR; 
    FILE* err = stderr;

    (void) nfiles;

        /* Create program objects.*/
    vert_shader = glCreateShader (GL_VERTEX_SHADER);
    frag_shader = glCreateShader (GL_FRAGMENT_SHADER);
    shader_program = glCreateProgram ();
    glAttachShader (shader_program, vert_shader);
    glAttachShader (shader_program, frag_shader);

        /* Fill program contents.*/
    glShaderSource (vert_shader, 1,
                    (const GLchar**) &files_bytes[0],
                    (const GLint*) &files_nbytes[0]);
    glShaderSource (frag_shader, 1,
                    (const GLchar**) &files_bytes[1],
                    (const GLint*) &files_nbytes[1]);

        /* Compile/link shader program.*/
    glCompileShader (vert_shader);
    glCompileShader (frag_shader);
    glLinkProgram (shader_program);

        /* Check for errors.*/
    glGetShaderiv (vert_shader, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE)
    {
        fputs ("Failed to compile vertex shader!\n", err);
        exit (1);
    }

    glGetShaderiv (frag_shader, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE)
    {
        fputs ("Failed to compile fragment shader!\n", err);
        exit (1);
    }

    glGetShaderiv (shader_program, GL_LINK_STATUS, &status);
    if (status != GL_TRUE)
    {
        fputs ("Failed to link shader program!\n", err);
        exit (1);
    }

    glUseProgram (shader_program);
}

static
    void
ogl_setup (const RaySpace* space)
{
    const SDL_VideoInfo* info;
    int bpp = 0;
    const GLfloat light_ambient[] = {0.2, 0.2, 0.2, 1.0};
        /* GLfloat light_ambient[] = {0, 0, 0, 1}; */
    const GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
        /* GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0}; */
    const GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
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
            uint j;
            GLfloat v[4];
            const PointLightSource* light = 0;

            if (i < space->nlights)  light = &space->lights[i];
            if (!light || !light->on)
            {
                glDisable (light_idcs[i]);
                continue;
            }

            v[3] = 1;
                /* Position.*/
            ogl_fvec_Point (v, &light->location);
            glLightfv (light_idcs[i], GL_POSITION, v);
                /* Ambient.*/
            UFor( j, 3 )  v[j] = light_ambient[j] * light->intensity[j];
            glLightfv (light_idcs[i], GL_AMBIENT, v);
                /* Diffuse.*/
            UFor( j, 3 )  v[j] = light_diffuse[j] * light->intensity[j];
            glLightfv (light_idcs[i], GL_DIFFUSE, v);
                /* Specular.*/
            UFor( j, 3 )  v[j] = light_specular[j] * light->intensity[j];
            if (light->diffuse)  UFor( j, 3 )  v[j] = 0;
            glLightfv (light_idcs[i], GL_SPECULAR, v);

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
        real near_mag = 0.1;
        real far_mag = 0;
        Ray ray;
        const BoundingBox* box;
        box = &space->main.box;

        copy_Point (&ray.origin, &pilot->view_origin);
        copy_Point (&ray.direct, &pilot->view_basis.pts[ForwardDim]);

        UFor( i, NDimensions )
        {
            real m;
            m = (ray.direct.coords[i] > 0)
                ? box->max.coords[i] - ray.origin.coords[i]
                : ray.origin.coords[i] - box->min.coords[i];

            if (m > 0)  far_mag += m;
        }

        gluPerspective (180 / M_PI * pilot->ray_image.hifov,
                        width / (real) height,
                        near_mag, far_mag);

            /* diff_Point (&p, &object->centroid, &pilot->view_origin); */
            /* proj_Point (&p, &pilot->view_basis.pts[ForwardDim], &p); */
            /* summ_Point (&p, &p, &pilot->view_origin); */
        summ_Point (&p, &ray.origin, &ray.direct);

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

#ifndef Match4dGeom
        /* Use this as a flag to say this is the main object (track).*/
    ogl_redraw_ObjectRaySpace (&space->main, &pilot->view_origin, 0);
#else
    ogl_redraw_ObjectRaySpace (&space->main,
                               &pilot->view_origin, &pilot->view_basis);
#endif
    UFor( i, space->nobjects )
        ogl_redraw_ObjectRaySpace (&space->objects[i],
                                   &pilot->view_origin, &pilot->view_basis);
}

    void
ogl_redraw_ObjectRaySpace (const ObjectRaySpace* object,
                           const Point* view_origin,
                           const PointXfrm* view_basis)
{
    Material default_material;
    GLdouble matrix[16];
    const Scene* scene;
    uint i;
    bool first_elem = true;
    uint material_idx = Max_uint;
#if NDimensions == 4
# ifdef Match4dGeom
    AffineMap affine_map;
    AffineMap* map;
    (void) view_basis;
# else
    Scene interp4d_scene;
# endif
#else
    (void) view_origin;
    (void) view_basis;
#endif

    if (!object->visible)  return;
    scene = &object->scene;

    init_Material (&default_material);

#if NDimensions == 4
# ifdef Match4dGeom
    map = &affine_map;
    identity_AffineMap (map);
    map->xlat.coords[DriftDim] = (+ object->centroid.coords[DriftDim]
                                  - view_origin->coords[DriftDim]);
# else

    if (!view_basis)
    {
        real alpha;
        alpha = ((view_origin->coords[DriftDim] - track.morph_dcoords[0]) /
                 (track.morph_dcoords[1] - track.morph_dcoords[0]));
        alpha = clamp_real (alpha, 0, 1);
        interpolate1_Scene (&interp4d_scene, alpha,
                            &track.morph_scenes[0], &track.morph_scenes[1]);
        scene = &interp4d_scene;
    }
# endif
#endif

    UFor( i, scene->ntxtrs )
    {
        const Texture* txtr;
        txtr = &scene->txtrs[i];
        glBindTexture (GL_TEXTURE_2D, i);
        glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexImage2D (GL_TEXTURE_2D, 0, 3, txtr->ncols, txtr->nrows, 0,
                      GL_RGB, GL_UNSIGNED_BYTE, txtr->pixels);
    }


    glPushMatrix ();

    ogl_matrix (matrix, &object->centroid, &object->orientation);

    glMultMatrixd (matrix);

        /* Send our triangle data to the pipeline. */
    UFor( i, scene->nelems )
    {
        const SceneElement* elem;
        uint nelems = 1;
        GLdouble verts[6*3];
        GLdouble vnmls[6*3];
        uint j;

        elem = &scene->elems[i];

#ifdef Match4dGeom
        nelems = view_element (verts, vnmls, elem, scene, map,
                               &object->simplices[i].plane.normal);
        if (nelems == 0)  continue;
#else
# if NDimensions == 4
        if (view_basis)
        {
            real d[4];
            UFor( j, 4 )
                d[j] = scene->verts[elem->verts[j]].coords[DriftDim];
            if (d[0] < d[1] || d[1] != d[2] || d[2] != d[3])  continue;
        }
# endif
        UFor( j, 3 )
        {
            Point dflt_normal;
            const Point* p;
            uint idx;

            idx = j;
# if NDimensions == 4
            if (view_basis)  idx += 1;
# endif

            p = &scene->verts[elem->verts[idx]];
            ogl_vec_Point (&verts[j*3], p);

            if (elem->vnmls[idx] < scene->nvnmls)
            {
                p = &scene->vnmls[elem->vnmls[idx]];
            }
            else
            {
                normalize_Point (&dflt_normal,
                                 &object->simplices[i].plane.normal);
                p = &dflt_normal;
            }
            ogl_vec_Point (&vnmls[j*3], p);
        }
#endif

        if (first_elem || elem->material != material_idx)
        {
            const Material* matl;
            GLfloat color[4];
            GLint loc;

            if (!first_elem)  glEnd ();
            else              first_elem = false;

            material_idx = elem->material;
            if (material_idx < scene->nmatls)
                matl = &scene->matls[material_idx];
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

            loc = glGetUniformLocation (shader_program, "HaveDiffuseTex");
            if (matl->diffuse_texture < Max_uint)
            {
                glUniform1i (loc, 1);
                glEnable (GL_TEXTURE_2D);
                glActiveTexture (GL_TEXTURE0);
                glBindTexture (GL_TEXTURE_2D, matl->diffuse_texture);
                loc = glGetUniformLocation (shader_program, "DiffuseTex");
                glUniform1i (loc, 0);
            }
            else
            {
                glUniform1i (loc, 0);
                glDisable (GL_TEXTURE_2D);
            }

            glBegin (GL_TRIANGLES);
        }

        UFor( j, 3 * nelems )
        {
                /* if (j % 3 == 0)  glColor3f (1, 0, 0); */
                /* if (j % 3 == 1)  glColor3f (0, 1, 0); */
                /* if (j % 3 == 2)  glColor3f (0, 0, 1); */
            glNormal3dv (&vnmls[j*3]);

            if (elem->txpts[j%3] < Max_uint)
            {
                const BaryPoint* p;
                p = &scene->txpts[elem->txpts[j%3]];
                glTexCoord2f (p->coords[0], p->coords[1]);
            }

            glVertex3dv (&verts[j*3]);
        }
    }
    if (scene->nelems > 0)  glEnd ();

    glPopMatrix ();

    if (scene->ntxtrs > 0)
    {
        GLuint* ids;
        ids = AllocT( GLuint, scene->ntxtrs );
        UFor( i, scene->ntxtrs )  ids[i] = i;
        glDeleteTextures (scene->ntxtrs, ids);
        free (ids);
    }

#if NDimensions == 4
# ifndef Match4dGeom
    if (!view_basis)  cleanup_Scene (&interp4d_scene);
# endif
#endif
}

#ifdef Match4dGeom
static void
view_element_helpfn (GLdouble* ret_verts,
                     GLdouble* ret_vnmls,
                     const real* alphas,
                     uint (* inds)[2],
                     const Point* verts,
                     const Point* vnmls)
{
    uint i;
    UFor( i, 3 )
    {
        real alpha;
        Point v;

        alpha = alphas[i];
        zero_Point (&v);

        Op_21010( real, 3, v.coords
                  ,+, (    alpha)*, verts[inds[i][0]].coords
                  ,   (1 - alpha)*, verts[inds[i][1]].coords );

        ogl_vec_Point (&ret_verts[i*3], &v);

        Op_21010( real, 3, v.coords
                  ,+, (    alpha)*, vnmls[inds[i][0]].coords
                  ,   (1 - alpha)*, vnmls[inds[i][1]].coords );

        normalize_Point (&v, &v);
        ogl_vec_Point (&ret_vnmls[i*3], &v);
    }
}

    uint
view_element (GLdouble* ret_verts,
              GLdouble* ret_vnmls,
              const SceneElement* elem,
              const Scene* scene,
              const AffineMap* map,
              const Point* normal)
{
    uint i, k = 0;
    uint nabove = 0;
    Point verts[NDimensions];
    Point vnmls[NDimensions];
    real d[NDimensions];
    bool positives[4];
    real alphas[4];
    uint inds[4][2];

    UFor( i, NDimensions )
    {
        copy_Point (&verts[i], &scene->verts[elem->verts[i]]);
        map_Point (&verts[i], map, &verts[i]);
        d[i] = verts[i].coords[DriftDim];
        positives[i] = (d[i] > 0);
        if (positives[i])  ++ nabove;
    }
    if (nabove == 0 || nabove == NDimensions)  return 0;

    UFor( i, NDimensions-1 )
    {
        uint j;
        for (j = i+1; j < NDimensions; ++j)
        {
            if (positives[i] != positives[j])
            {
                alphas[k] = fabs (d[j] / (d[j] - d[i]));
                inds[k][0] = i;
                inds[k][1] = j;
                ++ k;
            }
        }
    }

    assert (k >= 3);
    assert (k <= 4);
    UFor( i, NDimensions )
    {
        uint j;
        j = elem->vnmls[i];

        if (j < scene->nvnmls)
            copy_Point (&vnmls[i], &scene->vnmls[j]);
        else
            copy_Point (&vnmls[i], normal);

        mapovec_Point (&vnmls[i], map, &vnmls[i]);
    }

    view_element_helpfn (ret_verts, ret_vnmls, alphas, inds, verts, vnmls);
    if (k == 3)  return 1;

    inds[0][0] = inds[3][0];
    inds[0][1] = inds[3][1];
    view_element_helpfn (&ret_verts[3*3], &ret_vnmls[3*3],
                         alphas, inds, verts, vnmls);
    return 2;
}
#endif

