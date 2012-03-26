
#ifndef _WIN32
# define GL_GLEXT_PROTOTYPES
#endif
#include <SDL_opengl.h>

#ifndef GL_GLEXT_PROTOTYPES
    /* #define glActiveTexture glActiveTextureFunc */
static PFNGLUSEPROGRAMPROC glUseProgram = 0;
static PFNGLLINKPROGRAMPROC glLinkProgram = 0;
static PFNGLCOMPILESHADERPROC glCompileShader = 0;
static PFNGLSHADERSOURCEPROC glShaderSource = 0;
static PFNGLATTACHSHADERPROC glAttachShader = 0;
static PFNGLCREATEPROGRAMPROC glCreateProgram = 0;
static PFNGLCREATESHADERPROC glCreateShader = 0;
static PFNGLACTIVETEXTUREPROC glActiveTexture = 0;
static PFNGLUNIFORM1IPROC glUniform1i = 0;
static PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation = 0;
static PFNGLGETSHADERIVPROC glGetShaderiv = 0;

static PFNGLDELETEPROGRAMPROC glDeleteProgram = 0;
static PFNGLDELETESHADERPROC glDeleteShader = 0;
static PFNGLGENBUFFERSPROC glGenBuffers = 0;
static PFNGLDELETEBUFFERSPROC glDeleteBuffers = 0;
static PFNGLBINDBUFFERPROC glBindBuffer = 0;
static PFNGLBUFFERDATAPROC glBufferData = 0;
#endif

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

typedef struct SceneGL SceneGL;

struct SceneGL
{
    GLuint vidcs_buffer;
    GLuint verts_buffer;
    GLuint vnmls_buffer;
    GLuint txpts_buffer;
    GLuint texture_offset;
};

#ifdef SupportOpenCL
#include "gui-opencl.c"
#endif

SceneGL* scenegls = 0;
GLuint vert_shader;
GLuint frag_shader;
GLuint shader_program;
#if NDimensions == 4
static GLuint hivert_attrib_loc;
static GLuint hivnml_attrib_loc;
#endif
static GLuint ambient_texflag_loc;
static GLuint diffuse_texflag_loc;
static GLuint specular_texflag_loc;
static GLuint normal_texflag_loc;


static void
ogl_redraw_ObjectRaySpace (const RaySpace* space,
                           uint objidx,
                           const Point* view_origin,
                           const PointXfrm* view_basis);
#ifdef Match4dGeom
static uint
view_element (Point* ret_verts,
              Point* ret_vnmls,
              const SceneElement* elem,
              const Scene* scene,
              const AffineMap* map,
              const Point* normal);
#endif

    /** Set the current OpenGL transformation matrix.
     * This implicitly takes coordinate system differences into account!
     * Assume identity values for NULL /origin/ or /basis/.
     **/
static
    void
map0_ogl_matrix (const Point* origin, const PointXfrm* basis)
{
    GLdouble matrix[16];
    PointXfrm m;
    Point p;
    uint perms[NDimensions];
    uint i;

    UFor( i, NDimensions )  perms[i] = 2 * i;
    perms[0] = 2 * RightDim;
    perms[1] = 2 * UpDim;
    perms[2] = 2 * ForwardDim + 1;
    permutation_PointXfrm (&m, perms);

    if (origin)  xfrm_Point (&p, &m, origin);
    else         zero_Point (&p);
    if (basis)  xfrm_PointXfrm (&m, basis, &m);

    UFor( i, 3 )
    {
        uint j;
        UFor( j, 3 )
            matrix[4 * i + j] = m.pts[i].coords[j];
        matrix[4 * i + 3] = 0;
        matrix[4 * 3 + i] = p.coords[i];
    }
    matrix[15] = 1;
    glMultMatrixd (matrix);
}

static
    void
init_ogl_ui_data ()
{
    GLint status = GL_NO_ERROR; 
    FILE* err = stderr;
    GLint loc;
#ifdef EmbedFiles
#include EmbedInclude(phong.glsl)
    (void) nfiles;
#else  /* ^^^ defined(EmbedFiles) */
    static const char* const files[] =
    {   "phong.vert",
        "phong.frag",
        "4d.vert"
    };
    static const uint nfiles = ArraySz( files );
    uint files_nbytes[ArraySz( files )];
    byte* files_bytes[ArraySz( files )];
    bool good = true;

    good = readin_files (nfiles, files_nbytes, files_bytes, 0, files);

    if (!good) { fputs ("Bad read!\n", err); exit (1); }
#endif  /* !defined(EmbedFiles) */

#ifndef GL_GLEXT_PROTOTYPES
    glUseProgram = (PFNGLUSEPROGRAMPROC) SDL_GL_GetProcAddress ("glUseProgram");
    glLinkProgram = (PFNGLLINKPROGRAMPROC) SDL_GL_GetProcAddress ("glLinkProgram");
    glCompileShader = (PFNGLCOMPILESHADERPROC) SDL_GL_GetProcAddress ("glCompileShader");
    glShaderSource = (PFNGLSHADERSOURCEPROC) SDL_GL_GetProcAddress ("glShaderSource");
    glAttachShader = (PFNGLATTACHSHADERPROC) SDL_GL_GetProcAddress ("glAttachShader");
    glCreateProgram = (PFNGLCREATEPROGRAMPROC) SDL_GL_GetProcAddress ("glCreateProgram");
    glCreateShader = (PFNGLCREATESHADERPROC) SDL_GL_GetProcAddress ("glCreateShader");
    glActiveTexture = (PFNGLACTIVETEXTUREPROC) SDL_GL_GetProcAddress ("glActiveTexture");
    glUniform1i = (PFNGLUNIFORM1IPROC) SDL_GL_GetProcAddress ("glUniform1i");
    glGetUniformLocation = (PFNGLGETUNIFORMLOCATIONPROC) SDL_GL_GetProcAddress ("glGetUniformLocation");
    glGetShaderiv = (PFNGLGETSHADERIVPROC) SDL_GL_GetProcAddress ("glGetShaderiv");

    glDeleteProgram = (PFNGLDELETEPROGRAMPROC) SDL_GL_GetProcAddress ("glDeleteProgram");
    glDeleteShader = (PFNGLDELETESHADERPROC) SDL_GL_GetProcAddress ("glDeleteShader");
    glGenBuffers = (PFNGLGENBUFFERSPROC) SDL_GL_GetProcAddress ("glGenBuffers");
    glDeleteBuffers = (PFNGLDELETEBUFFERSPROC) SDL_GL_GetProcAddress ("glDeleteBuffers");
    glBindBuffer = (PFNGLBINDBUFFERPROC) SDL_GL_GetProcAddress ("glBindBuffer");
    glBufferData = (PFNGLBUFFERDATAPROC) SDL_GL_GetProcAddress ("glBufferData");

    if (!glUseProgram) { fputs ("glUseProgram\n", err); exit(1); }
    if (!glLinkProgram) { fputs ("glLinkProgram\n", err); exit(1); }
    if (!glCompileShader) { fputs ("glCompileShader\n", err); exit(1); }
    if (!glShaderSource) { fputs ("glShaderSource\n", err); exit(1); }
    if (!glAttachShader) { fputs ("glAttachShader\n", err); exit(1); }
    if (!glCreateProgram) { fputs ("glCreateProgram\n", err); exit(1); }
    if (!glCreateShader) { fputs ("glCreateShader\n", err); exit(1); }
    if (!glActiveTexture) { fputs ("glActiveTexture\n", err); exit(1); }
    if (!glUniform1i) { fputs ("glUniform1i\n", err); exit(1); }
    if (!glGetUniformLocation) { fputs ("glGetUniformLocation\n", err); exit(1); }
    if (!glGetShaderiv) { fputs ("glGetShaderiv\n", err); exit(1); }

    if (!glDeleteProgram) { fputs ("glDeleteProgram\n", err); exit(1); }
    if (!glDeleteShader) { fputs ("glDeleteShader\n", err); exit(1); }
    if (!glGenBuffers) { fputs ("glGenBuffers\n", err); exit(1); }
    if (!glDeleteBuffers) { fputs ("glDeleteBuffers\n", err); exit(1); }
    if (!glBindBuffer) { fputs ("glBindBuffer\n", err); exit(1); }
    if (!glBufferData) { fputs ("glBufferData\n", err); exit(1); }
    fputs ("Loaded all required function pointers!\n", err);
#endif

        /* Create program objects.*/
    vert_shader = glCreateShader (GL_VERTEX_SHADER);
    frag_shader = glCreateShader (GL_FRAGMENT_SHADER);
    shader_program = glCreateProgram ();
    glAttachShader (shader_program, vert_shader);
    glAttachShader (shader_program, frag_shader);

        /* Fill program contents.*/
#if NDimensions == 4 && !defined(Match4dGeom)
    glShaderSource (vert_shader, 1,
                    (const GLchar**) &files_bytes[2],
                    (const GLint*) &files_nbytes[2]);
#else
    glShaderSource (vert_shader, 1,
                    (const GLchar**) &files_bytes[0],
                    (const GLint*) &files_nbytes[0]);
#endif
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
#if NDimensions == 4
    hivert_attrib_loc = glGetAttribLocation (shader_program, "hivert");
    hivnml_attrib_loc = glGetAttribLocation (shader_program, "hivnml");
#endif

        /* These are hard-coded texture numbers!*/
    loc = glGetUniformLocation (shader_program, "AmbientTex");
    glUniform1i (loc, 0);
    loc = glGetUniformLocation (shader_program, "DiffuseTex");
    glUniform1i (loc, 1);
    loc = glGetUniformLocation (shader_program, "SpecularTex");
    glUniform1i (loc, 2);
    loc = glGetUniformLocation (shader_program, "NormalTex");
    glUniform1i (loc, 3);
    ambient_texflag_loc = glGetUniformLocation (shader_program, "HaveAmbientTex");
    diffuse_texflag_loc = glGetUniformLocation (shader_program, "HaveDiffuseTex");
    specular_texflag_loc = glGetUniformLocation (shader_program, "HaveSpecularTex");
    normal_texflag_loc = glGetUniformLocation (shader_program, "HaveNormalTex");

#ifdef SupportOpenCL
    init_opencl_data ();
#endif

#ifndef EmbedFiles
    { BLoop( i, nfiles )
        free (files_bytes[i]);
    } BLose()
#endif
}

static void
cleanup_ogl_ui_data (const RaySpace* space)
{
#if NDimensions == 4 && !defined(Match4dGeom)
    const uint nscenes = space->nobjects + track.nmorphs;
#else
    const uint nscenes = space->nobjects+1;
#endif
    uint scenei;

#ifdef SupportOpenCL
    cleanup_opencl_data ();
#endif

    glUseProgram (0);
    glDeleteProgram (shader_program);
    glDeleteShader (vert_shader);
    glDeleteShader (frag_shader);

    UFor( scenei, nscenes )
    {
        GLuint vbos[4];
        SceneGL* scenegl;
        const Scene* scene;

        scenegl = &scenegls[scenei];
        if (scenei < space->nobjects)
            scene = &space->objects[scenei].scene;
        else
        {
#if NDimensions == 4 && !defined(Match4dGeom)
                scene = &track.morph_scenes[scenei - space->nobjects];
#else
                scene = &space->main.scene;
#endif
        }

        if (scene->ntxtrs > 0)
        {
            GLuint* ids;
            uint i;
            ids = AllocT( GLuint, scene->ntxtrs );
            UFor( i, scene->ntxtrs )
                ids[i] = i + scenegl->texture_offset;
            glDeleteTextures (scene->ntxtrs, ids);
            free (ids);
        }

        vbos[0] = scenegl->vidcs_buffer;
        vbos[1] = scenegl->verts_buffer;
        vbos[2] = scenegl->vnmls_buffer;
        vbos[3] = scenegl->txpts_buffer;
        glDeleteBuffers (ArraySz( vbos ), vbos);
    }
}

static
    void
ogl_setup (const RaySpace* space)
{
    const SDL_VideoInfo* info;
    int bpp = 0;
    uint i;
    uint ntextures = 0;

    if (!scenegls)
    {
#if NDimensions == 4 && !defined(Match4dGeom)
        const uint nscenes = space->nobjects + track.nmorphs;
#else
        const uint nscenes = space->nobjects+1;
#endif
        scenegls = AllocT( SceneGL, nscenes );
        glActiveTexture (GL_TEXTURE0);
        UFor( i, nscenes )
        {
            GLuint vbos[4];
            SceneGL* scenegl;
            const Scene* scene;
            uint texi;

            scenegl = &scenegls[i];
            if (i < space->nobjects)
                scene = &space->objects[i].scene;
            else
            {
#if NDimensions == 4 && !defined(Match4dGeom)
                scene = &track.morph_scenes[i - space->nobjects];
#else
                scene = &space->main.scene;
#endif
            }

            glGenBuffers (ArraySz( vbos ), vbos);
            scenegl->vidcs_buffer = vbos[0];
            scenegl->verts_buffer = vbos[1];
            scenegl->vnmls_buffer = vbos[2];
            scenegl->txpts_buffer = vbos[3];

#ifdef Match4dGeom
            glBindBuffer (GL_ARRAY_BUFFER, scenegl->verts_buffer);
            glBufferData (GL_ARRAY_BUFFER, scene->nelems * 3*6 * sizeof (real),
                          0, GL_DYNAMIC_DRAW);
            glBindBuffer (GL_ARRAY_BUFFER, scenegl->vnmls_buffer);
            glBufferData (GL_ARRAY_BUFFER, scene->nelems * 3*6 * sizeof (real),
                          0, GL_DYNAMIC_DRAW);
#else  /* ^^^ defined(Match4dGeom) */
            glBindBuffer (GL_ARRAY_BUFFER, scenegl->verts_buffer);
            glBufferData (GL_ARRAY_BUFFER, scene->nverts * sizeof (Point),
                          scene->verts, GL_STATIC_DRAW);
            glBindBuffer (GL_ARRAY_BUFFER, scenegl->vnmls_buffer);
            glBufferData (GL_ARRAY_BUFFER, scene->nvnmls * sizeof (Point),
                          scene->vnmls, GL_STATIC_DRAW);
            glBindBuffer (GL_ARRAY_BUFFER, scenegl->txpts_buffer);
            glBufferData (GL_ARRAY_BUFFER, scene->ntxpts * sizeof (BaryPoint),
                          scene->txpts, GL_STATIC_DRAW);

            if (scene->vidcs)
            {
                glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, scenegl->vidcs_buffer);
                glBufferData (GL_ELEMENT_ARRAY_BUFFER,
                              3 * scene->nelems * sizeof (uint),
                              scene->vidcs, GL_STATIC_DRAW);
            }
#endif  /* !defined(Match4dGeom) */

            scenegl->texture_offset = ntextures;
            UFor( texi, scene->ntxtrs )
            {
                const Texture* txtr;
                txtr = &scene->txtrs[texi];
                glBindTexture (GL_TEXTURE_2D, scenegl->texture_offset + texi);
                glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                glTexImage2D (GL_TEXTURE_2D, 0, txtr->pixelsz,
                              txtr->ncols, txtr->nrows, 0,
                              (txtr->alpha ? GL_RGBA : GL_RGB),
                              GL_UNSIGNED_BYTE, txtr->pixels);
            }
            ntextures += scene->ntxtrs;
        }
    }
    glBindBuffer (GL_ARRAY_BUFFER, 0);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);

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
    if (space->nlights > 0)  glEnable (GL_LIGHTING);
    else                     glDisable (GL_LIGHTING);

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
    real near_mag = 0, far_mag = 0;
    Ray ray;

    pilot = &pilots[pilot_idx];

    glClearColor (0, 0, 0, 0);

    height = npixelzoom * pilot->ray_image.nrows;
    width = npixelzoom * pilot->ray_image.ncols;

    glViewport (npixelzoom * pilot->image_start_col,
                npixelzoom * pilot->image_start_row,
                width, height);

        /* Clear the depth buffer. */
    glClear (GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    ray.origin = pilot->view_origin;
    ray.direct = pilot->view_basis.pts[FoDim];

    UFor( i, 3 )
    {
        const BoundingBox* box = &space->main.box;
        real m =
            (ray.direct.coords[i] > 0)
            ? box->max.coords[i] - ray.origin.coords[i]
            : ray.origin.coords[i] - box->min.coords[i];

        if (m > 0)  far_mag += m;
    }
    near_mag = far_mag / 1000;


    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    if (pilot->ray_image.perspective)
    {

        gluPerspective (180 / M_PI * pilot->ray_image.hifov,
                        width / (real) height,
                        near_mag, far_mag);

    }
    else
    {
        uint max_n;
        real horz, vert;
        if (width >= height)  max_n = width;
        else                  max_n = height;
        horz = .5 * pilot->ray_image.hifov * (width / (real) max_n);
        vert = .5 * pilot->ray_image.hifov * (height / (real) max_n);

        glOrtho (-horz, horz, -vert, vert,
                 near_mag, far_mag);
    }

        /* We don't want to modify the projection matrix. */
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();

    if (true)
    {
        Point p;
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

    if (space->nlights > 0)
    {
        const GLfloat light_ambient[] = {0.2, 0.2, 0.2, 1.0};
            /* GLfloat light_ambient[] = {0, 0, 0, 1}; */
        const GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
            /* GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0}; */
        const GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
        const GLenum light_idcs[8] =
        {   GL_LIGHT0, GL_LIGHT1, GL_LIGHT2, GL_LIGHT3,
            GL_LIGHT4, GL_LIGHT5, GL_LIGHT6, GL_LIGHT7
        };
        glPushMatrix();
        map0_ogl_matrix (0, 0);

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

            UFor( j, 3 )  v[j] = light->location.coords[j];
            v[3] = 1;

                /* Position.*/
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
        glPopMatrix();
    }

    glEnable (GL_TEXTURE_2D);

    ogl_redraw_ObjectRaySpace (space, space->nobjects,
                               &pilot->view_origin, &pilot->view_basis);
    UFor( i, space->nobjects )
        ogl_redraw_ObjectRaySpace (space, i,
                                   &pilot->view_origin, &pilot->view_basis);

    glDisable (GL_TEXTURE_2D);
}

static void
ogl_set_ObjectSurface (const ObjectSurface* surf,
                       const Scene* scene, const SceneGL* scenegl)
{
    Material default_material;
    const Material* matl;
    GLfloat color[4];
    bool flag;
    uint j;

    if (surf->material < scene->nmatls)
    {
        matl = &scene->matls[surf->material];
    }
    else
    {
        init_Material (&default_material);
        matl = &default_material;
    }

    color[3] = matl->opacity;
    UFor( j, 3 )  color[j] = matl->ambient[j];
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);

    UFor( j, 3 )  color[j] = matl->diffuse[j];
    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);

    UFor( j, 3 )  color[j] = matl->specular[j];
    glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, color);

    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS,
                 matl->optical_density);

        /* These are hard-coded texture numbers set in init_ogl_ui_data()!*/
    flag = (matl->ambient_texture < Max_uint);
    glUniform1i (ambient_texflag_loc, flag ? 1 : 0);
    if (flag)
    {
        glActiveTexture (GL_TEXTURE0);
        glBindTexture (GL_TEXTURE_2D,
                       ((matl->ambient_texture < Max_uint)
                        ? scenegl->texture_offset + matl->ambient_texture
                        : 0));
    }

    flag = (matl->diffuse_texture < Max_uint);
    glUniform1i (diffuse_texflag_loc, flag ? 1 : 0);
    if (flag)
    {
        glActiveTexture (GL_TEXTURE1);
        glBindTexture (GL_TEXTURE_2D,
                       ((matl->diffuse_texture < Max_uint)
                        ? scenegl->texture_offset + matl->diffuse_texture
                        : 0));
    }

    flag = (matl->specular_texture < Max_uint);
    glUniform1i (specular_texflag_loc, flag ? 1 : 0);
    if (flag)
    {
        glActiveTexture (GL_TEXTURE2);
        glBindTexture (GL_TEXTURE_2D,
                       ((matl->specular_texture < Max_uint)
                        ? scenegl->texture_offset + matl->specular_texture
                        : 0));
    }

    flag = (matl->bump_texture < Max_uint);
    glUniform1i (normal_texflag_loc, flag ? 1 : 0);
    if (flag)
    {
        glActiveTexture (GL_TEXTURE3);
        glBindTexture (GL_TEXTURE_2D,
                       scenegl->texture_offset + matl->bump_texture);
    }
}

static void
ogl_immediate_redraw_ObjectRaySpace (const RaySpace* space,
                                     uint objidx,
                                     const Point* view_origin,
                                     const PointXfrm* view_basis)
{
    const Scene* scene;
    const SceneGL* scenegl;
    const ObjectRaySpace* object;
    uint i;
    bool first_elem = true;
    uint material_idx = Max_uint;
#if NDimensions == 4
# ifdef Match4dGeom
    AffineMap affine_map;
    AffineMap* map;
    (void) view_basis;  /* TODO: Use this.*/
# else
    Scene interp4d_scene;
    (void) view_basis;
# endif
#else  /* ^^^ NDimensions == 4 */
    (void) view_origin;
    (void) view_basis;
#endif  /* NDimensions != 4 */

    if (objidx < space->nobjects)
        object = &space->objects[objidx];
    else
        object = &space->main;
    scene = &object->scene;
    scenegl = &scenegls[objidx];

#if NDimensions == 4
# ifdef Match4dGeom
    map = &affine_map;
    identity_AffineMap (map);
    map->xlat.coords[DriftDim] = (+ object->centroid.coords[DriftDim]
                                  - view_origin->coords[DriftDim]);
# else

    if (objidx >= space->nobjects)
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
#endif  /* NDimensions == 4 */

    glPushMatrix ();

    map0_ogl_matrix (&object->centroid, &object->orientation);

        /* Send our triangle data to the pipeline. */
    UFor( i, scene->nelems )
    {
        const SceneElement* elem;
        uint nelems = 1;
        Point verts[6];
        Point vnmls[6];
        uint j;

        elem = &scene->elems[i];

#ifdef Match4dGeom
        nelems = view_element (verts, vnmls, elem, scene, map,
                               &object->simplices[i].plane.normal);
        if (nelems == 0)  continue;
#else  /* ^^^ defined(Match4dGeom) */

#if NDimensions == 4
        if (objidx < space->nobjects)
        {
            real d[4];
            UFor( j, 4 )
                d[j] = scene->verts[elem->verts[j]].coords[DriftDim];
            if (d[0] < d[1] || d[1] != d[2] || d[2] != d[3])  continue;
        }
#endif

        UFor( j, 3 )
        {
            uint idx;

            idx = j;
#if NDimensions == 4
            if (objidx < space->nobjects)  idx += 1;
#endif

            verts[j] = scene->verts[elem->verts[idx]];

            if (elem->vnmls[idx] < scene->nvnmls)
                vnmls[j] = scene->vnmls[elem->vnmls[idx]];
            else
                normalize_Point (&vnmls[j],
                                 &object->simplices[i].plane.normal);
        }
#endif  /* !defined(Match4dGeom) */

        if (first_elem || elem->material != material_idx)
        {
            if (!first_elem)  glEnd ();
            else              first_elem = false;

            material_idx = elem->material;
            ogl_set_ObjectSurface (&scene->surfs[elem->surface],
                                   scene, scenegl);

            glBegin (GL_TRIANGLES);
        }

        UFor( j, 3 * nelems )
        {
                /* if (j % 3 == 0)  glColor3f (1, 0, 0); */
                /* if (j % 3 == 1)  glColor3f (0, 1, 0); */
                /* if (j % 3 == 2)  glColor3f (0, 0, 1); */

            glNormal3f (vnmls[j].coords[0],
                        vnmls[j].coords[1],
                        vnmls[j].coords[2]);

            if (elem->txpts[j%3] < Max_uint)
            {
                const BaryPoint* p;
                p = &scene->txpts[elem->txpts[j%3]];
                glTexCoord2f (p->coords[0], p->coords[1]);
            }
            glVertex3f (verts[j].coords[0],
                        verts[j].coords[1],
                        verts[j].coords[2]);


        }
    }
    if (scene->nelems > 0)  glEnd ();

    glPopMatrix ();

#if NDimensions == 4
# ifndef Match4dGeom
    if (objidx >= space->nobjects)  cleanup_Scene (&interp4d_scene);
# endif
#endif
}

    void
ogl_redraw_ObjectRaySpace (const RaySpace* space,
                           uint objidx,
                           const Point* view_origin,
                           const PointXfrm* view_basis)
{
    const Scene* scene;
    const SceneGL* scenegl;
    const ObjectRaySpace* object;
    uint surfi;
#if NDimensions == 4
#ifdef Match4dGeom
    uint elem_offset;
#else
    real alpha;
    GLint alpha_loc;
#endif
#endif

    if (objidx < space->nobjects)
        object = &space->objects[objidx];
    else
        object = &space->main;
    if (!object->visible)  return;

#ifdef Match4dGeom
#ifdef SupportOpenCL
    if (object->nelems == 1)
#else
    if (true)
#endif
#else  /* ^^^ defined(Match4dGeom) */
    if (NDimensions == 4 && objidx < space->nobjects)
#endif  /* !defined(Match4dGeom) */
    {
        ogl_immediate_redraw_ObjectRaySpace (space, objidx,
                                             view_origin, view_basis);
        return;
    }

    glEnableClientState (GL_VERTEX_ARRAY);
    scene = &object->scene;
#if NDimensions == 4
#ifdef Match4dGeom
    glEnableClientState (GL_NORMAL_ARRAY);
#else
    scene = &track.morph_scenes[0];
    glEnableVertexAttribArray (hivert_attrib_loc);
    alpha = ((view_origin->coords[DriftDim] - track.morph_dcoords[0]) /
             (track.morph_dcoords[1] - track.morph_dcoords[0]));
    alpha = clamp_real (alpha, 0, 1);
    alpha_loc = glGetUniformLocation (shader_program, "alpha");
    glUniform1f (alpha_loc, alpha);
#endif
#endif  /* NDimensions == 4 */
    scenegl = &scenegls[objidx];
    glPushMatrix ();

    map0_ogl_matrix (&object->centroid, &object->orientation);
#ifdef Match4dGeom
#ifdef SupportOpenCL
    {
        AffineMap map;

        identity_AffineMap (&map);
        map.xlat.coords[DriftDim] = (+ object->centroid.coords[DriftDim]
                                     - view_origin->coords[DriftDim]);

        view_4d_vertices (scene, scenegl, &map.xfrm, &map.xlat);
    }
#endif
#else
        /* Single-element objects are the only ones that change.*/
    if (scene->nelems == 1)
    {
        glBindBuffer (GL_ARRAY_BUFFER, scenegl->verts_buffer);
        glBufferData (GL_ARRAY_BUFFER, scene->nverts * sizeof (Point),
                      scene->verts, GL_DYNAMIC_DRAW);
    }
#ifdef SupportOpenCL
    perturb_vertices (scene, scenegl);
#endif
#endif

    UFor( surfi, scene->nsurfs )
    {
        const ObjectSurface* surf;
        surf = &scene->surfs[surfi];
        ogl_set_ObjectSurface (surf, scene, scenegl);
#ifdef Match4dGeom
        elem_offset = surf->vidcs_offset / 4;
        glBindBuffer (GL_ARRAY_BUFFER, scenegl->verts_buffer);
        glVertexPointer (3, GL_REAL, 0, elem_offset * 6 * 3 + (real*) 0);
        glBindBuffer (GL_ARRAY_BUFFER, scenegl->vnmls_buffer);
        glNormalPointer (GL_REAL, 0, elem_offset * 6 * 3 + (real*) 0);
        glDrawArrays (GL_TRIANGLES, 0, 6 * surf->nelems);

#else  /* ^^^ defined(Match4dGeom) */
        glBindBuffer (GL_ARRAY_BUFFER, scenegl->verts_buffer);
        glVertexPointer (3, GL_REAL, sizeof (Point),
                         surf->verts_offset + (Point*) 0);
#if NDimensions == 4
        glBindBuffer (GL_ARRAY_BUFFER, scenegls[objidx+1].verts_buffer);
        glVertexAttribPointer (hivert_attrib_loc,
                               3, GL_REAL, GL_FALSE, sizeof (Point),
                               surf->verts_offset + (Point*) 0);
#endif
        if (surf->vnmls_offset < Max_uint)
        {
            glBindBuffer (GL_ARRAY_BUFFER, scenegl->vnmls_buffer);
            glNormalPointer (GL_REAL, sizeof (Point),
                             surf->vnmls_offset + (Point*) 0);
            glEnableClientState (GL_NORMAL_ARRAY);
#if NDimensions == 4
            glBindBuffer (GL_ARRAY_BUFFER, scenegls[objidx+1].vnmls_buffer);
            glVertexAttribPointer (hivnml_attrib_loc,
                                   3, GL_REAL, GL_FALSE, sizeof (Point),
                                   surf->vnmls_offset + (Point*) 0);
            glEnableVertexAttribArray (hivnml_attrib_loc);
#endif
        }
        if (surf->txpts_offset < Max_uint)
        {
            glBindBuffer (GL_ARRAY_BUFFER, scenegl->txpts_buffer);
            glTexCoordPointer (2, GL_REAL, sizeof (BaryPoint),
                               surf->txpts_offset + (BaryPoint*) 0);
            glEnableClientState (GL_TEXTURE_COORD_ARRAY);
        }

        if (surf->vidcs_offset < Max_uint)
        {
            glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, scenegl->vidcs_buffer);
            glDrawElements (GL_TRIANGLES,
                            3 * surf->nelems,
                            GL_UNSIGNED_INT,
                            surf->vidcs_offset + (uint*) 0);
        }
        else
        {
            glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);
            glDrawArrays (GL_TRIANGLES, 0, 3 * surf->nelems);
        }

        if (surf->vnmls_offset < Max_uint)
        {
            glDisableClientState (GL_NORMAL_ARRAY);
#if NDimensions == 4
            glDisableVertexAttribArray (hivnml_attrib_loc);
#endif
        }
        if (surf->txpts_offset < Max_uint)
            glDisableClientState (GL_TEXTURE_COORD_ARRAY);
#endif  /* !defined(Match4dGeom) */
    }

    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer (GL_ARRAY_BUFFER, 0);
    glDisableClientState (GL_VERTEX_ARRAY);
#ifdef Match4dGeom
    glDisableClientState (GL_NORMAL_ARRAY);
#endif
#if NDimensions == 4 && !defined(Match4dGeom)
    glUniform1f (alpha_loc, 0);
    glDisableVertexAttribArray (hivert_attrib_loc);
#endif

    glPopMatrix ();
}

#ifdef Match4dGeom
    uint
view_element (Point* ret_verts,
              Point* ret_vnmls,
              const SceneElement* elem,
              const Scene* scene,
              const AffineMap* map,
              const Point* normal)
{
    uint i, k = 0;
    uint nabove = 0;
    uint inds[4][2];
    Simplex tet;

    UFor( i, NDimensions )
    {
        tet.pts[i] = scene->verts[elem->verts[i]];
        map_Point (&tet.pts[i], map, &tet.pts[i]);
        if (tet.pts[i].coords[DriftDim] > 0)  ++ nabove;
    }
    if (nabove == 0 || nabove == NDimensions)  return 0;

        /* Find all edges which cross the view plane (3 or 4).*/
    UFor( i, NDimensions-1 )
    {
        const real x = tet.pts[i].coords[DriftDim];
        uint j;
        for (j = i+1; j < NDimensions; ++j)
        {
            const real y = tet.pts[j].coords[DriftDim];
            if ((y <= 0) == (0 < x))
            {
                inds[k][0] = i;
                inds[k][1] = j;
                ++ k;
            }
        }
    }

    assert (k >= 3);
    assert (k <= 4);
    UFor( i, k )
    {
        Point a, b;
        real alpha;
        uint j;

        a = tet.pts[inds[i][0]];
        b = tet.pts[inds[i][1]];
        alpha = fabs (a.coords[3] / (a.coords[3] - b.coords[3]));

            /* Mix.*/
        UFor( j, 3 )
            a.coords[j] += (b.coords[j] - a.coords[j]) * alpha;

        if (scene->nvnmls > 0)
        {
            Point c;
            b = scene->vnmls[elem->vnmls[inds[i][0]]];
            c = scene->vnmls[elem->vnmls[inds[i][1]]];

            UFor( j, NDimensions )
                b.coords[j] += (c.coords[j] - b.coords[j]) * alpha;
        }
        else
            b = *normal;

        mapo_Point (&b, map, &b);
        normalize_Point (&b, &b);

        ret_verts[i] = a;
        ret_vnmls[i] = b;

        if (i < NDimensions -1 && k > 3)
        {
            ret_verts[i+3] = a;
            ret_vnmls[i+3] = b;
        }
    }
    return (k == 3) ? 1 : 2;
}
#endif  /* defined(Match4dGeom) */

