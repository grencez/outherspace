
    /* File included from gui-opengl.c.*/
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif
#include <CL/cl_gl.h>
#include <GL/glx.h>

typedef struct SysOpenCL SysOpenCL;

struct SysOpenCL
{
    cl_program program;
    cl_kernel kernel;

    uint ndevices;
    cl_device_id* devices;
    cl_context context;
    cl_command_queue* comqs;
};

SysOpenCL opencl_state;


static
    void
compute_devices (uint* ret_ndevices,
                 cl_device_id** ret_devices,
                 cl_context* ret_context,
                 cl_command_queue** ret_queues)
{
    FILE* out = stdout;
    cl_int err;
    uint i;
    cl_uint nplatforms = 0, ndevices = 0;
    cl_uint device_offset;

    cl_platform_id* platforms;
    cl_device_id* devices;
    cl_context context;
    cl_command_queue* comqs;
    cl_context_properties context_props[5];

    i = 0;
    context_props[i++] = CL_GL_CONTEXT_KHR;
    context_props[i++] = (intptr_t) glXGetCurrentContext ();
    context_props[i++] = CL_GLX_DISPLAY_KHR;
    context_props[i++] = (intptr_t) glXGetCurrentDisplay ();
    context_props[i++] = 0;

    err = clGetPlatformIDs (0, 0, &nplatforms);
    AssertStatus( err, "query platform count" );

    platforms = AllocT( cl_platform_id, nplatforms );

    err = clGetPlatformIDs (nplatforms, platforms, 0);
    AssertStatus( err, "fetch platforms" );

    UFor( i, nplatforms )
    {
        cl_uint n;
        err = clGetDeviceIDs (platforms[i], CL_DEVICE_TYPE_ALL,
                                 0, 0, &n);
        AssertStatus( err, "query device count" );
        ndevices += n;
    }

    devices = AllocT( cl_device_id, ndevices );

    device_offset = 0;
    UFor( i, nplatforms )
    {
        cl_uint n;
        err = clGetDeviceIDs (platforms[i], CL_DEVICE_TYPE_ALL,
                              ndevices - device_offset,
                              &devices[device_offset],
                              &n);
        AssertStatus( err, "fetch devices" );
        device_offset += n;
    }

    if (platforms)  free (platforms);

        /* Create a compute context.*/
    context = clCreateContext (context_props, ndevices, devices, NULL, NULL, &err);
    AssertStatus( err, "create context" );

    comqs = AllocT( cl_command_queue, ndevices );
    UFor( i, ndevices )
    {
        char buf[BUFSIZ];
        err = clGetDeviceInfo (devices[i], CL_DRIVER_VERSION,
                               BUFSIZ-1, buf, 0);
        buf[BUFSIZ-1] = 0;
        fputs ("OpenCL Version: ", out);
        fputs (buf, out);
        fputc ('\n', out);
        AssertStatus( err, "platform" );
            /* Create a command queue.*/
        comqs[i] = clCreateCommandQueue (context, devices[i], 0, &err);
        AssertStatus( err, "create command queue" );
    }

    *ret_ndevices = ndevices;
    *ret_devices = devices;
    *ret_context = context;
    *ret_queues = comqs;
}


static
    void
load_program (cl_program* ret_program, cl_context context,
              uint ndevices, const cl_device_id* devices,
              uint nfiles,
              const uint* files_nbytes,
              const byte* const* files_bytes)
{
    uint i;
    size_t* sizes;
    cl_program program;
    cl_int err;
    const bool show_warnings = false;

    sizes = AllocT( size_t, nfiles );
    UFor( i, nfiles )
        sizes[i] = files_nbytes[i];

    program = clCreateProgramWithSource (context, nfiles,
                                         (const char**) files_bytes,
                                         sizes, &err);
    AssertStatus( err, "create program" );

    err = clBuildProgram (program, ndevices, devices, "", 0, 0);
    if (err != CL_SUCCESS || show_warnings)
    {
        cl_build_status build_stat;
        UFor( i, ndevices )
        {
            cl_int be;
            be = clGetProgramBuildInfo (program, devices[i],
                                        CL_PROGRAM_BUILD_STATUS,
                                        sizeof (cl_build_status),
                                        &build_stat,
                                        0);
            AssertStatus( be, "build info" );
            if (build_stat != CL_BUILD_SUCCESS || show_warnings) {
                char* log;
                size_t size;
                FILE* out;
                out = stderr;
                be = clGetProgramBuildInfo (program, devices[i],
                                            CL_PROGRAM_BUILD_LOG,
                                            0, 0, &size);
                AssertStatus( be, "build log size" );
                log = AllocT( char, 1+size/sizeof(char) );
                log[size/sizeof(char)] = 0;

                be = clGetProgramBuildInfo (program, devices[i],
                                            CL_PROGRAM_BUILD_LOG,
                                            size, log, &size);
                AssertStatus( be, "build log" );
                fputs ("Failure building for device.\n", out);
                fputs (log, out);
                fputs ("\n", out);
                if (log)  free (log);
            }
        }
    }
    AssertStatus( err, "build program" );

    if (sizes)  free (sizes);

    *ret_program = program;
}

static void
init_opencl_data ()
{
#ifdef Match4dGeom
    static const char kernel_name[] = "view_4d_kernel";
#else
    static const char kernel_name[] = "perturb_kernel";
#endif
    int err;
    SysOpenCL* cl = &opencl_state;

#ifdef EmbedFiles
#include EmbedInclude(perturb.cl)
#else  /* ^^^ defined(EmbedFiles) */
    static const char* files[] = { "perturb.cl" };
    static const uint nfiles = ArraySz( files );
    uint files_nbytes[ArraySz( files )];
    byte* files_bytes[ArraySz( files )];

    readin_files (nfiles, files_nbytes, files_bytes, 0, files);
#endif  /* !defined(EmbedFiles) */

    cl->ndevices = 0;
    compute_devices (&cl->ndevices, &cl->devices, &cl->context, &cl->comqs);
    printf ("I have %u devices!\n", cl->ndevices);

    load_program (&cl->program, cl->context, cl->ndevices, cl->devices,
                  nfiles, files_nbytes, (const byte* const*) files_bytes);

        /* Create the compute kernel in the program we wish to run.*/
    cl->kernel = clCreateKernel (cl->program, kernel_name, &err);
    AssertStatus( err, "create compute kernel" );

#ifndef EmbedFiles
    {:for (i ; nfiles)
        free (files_bytes[i]);
    }
#endif
}

static void
cleanup_opencl_data ()
{
    SysOpenCL* cl = &opencl_state;
    uint i;

        /* Shutdown and cleanup.*/
    clReleaseProgram (cl->program);
    clReleaseKernel (cl->kernel);
    UFor( i, cl->ndevices )
        clReleaseCommandQueue (cl->comqs[i]);
    clReleaseContext (cl->context);
}

static void
perturb_vertices (const Scene* scene, const SceneGL* scenegl)
{
    static uint count = 0;
    SysOpenCL* cl = &opencl_state;
    int err;

    size_t global, local;
    const uint dev_idx = 0;
    cl_mem verts;

    ++ count;

    verts = clCreateFromGLBuffer (cl->context, CL_MEM_READ_WRITE,
                                  scenegl->verts_buffer, &err);
    AssertStatus( err, "create from gl buffer" );

    err = clEnqueueAcquireGLObjects (cl->comqs[dev_idx], 1, &verts, 0, 0, 0);
    AssertStatus( err, "acquire gl objects" );

        /* Set the arguments to our compute kernel.*/
    err  = clSetKernelArg(cl->kernel, 0, sizeof(cl_mem), &verts);
    err |= clSetKernelArg(cl->kernel, 1, sizeof(uint), &count);
    AssertStatus( err, "set kernel arguments" );

        /* Get the maximum work group size for executing the kernel on the device.*/
    err = clGetKernelWorkGroupInfo(cl->kernel, cl->devices[dev_idx],
                                   CL_KERNEL_WORK_GROUP_SIZE,
                                   sizeof(local), &local, 0);
    AssertStatus( err, "retrieve kernel work group info" );

    global = scene->nverts;
        /* err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, &local, 0, NULL, NULL); */
    err = clEnqueueNDRangeKernel (cl->comqs[dev_idx], cl->kernel, 1, 0, &global, 0, 0, 0, 0);
    AssertStatus( err, "enqueue kernel" );

    err = clEnqueueReleaseGLObjects (cl->comqs[dev_idx], 1, &verts, 0, 0, 0);
    AssertStatus( err, "release gl objects" );

        /* Wait for the commands to get serviced before reading back results.*/
    clFinish (cl->comqs[dev_idx]);
}

static void
view_4d_vertices (const Scene* scene,
                  const SceneGL* scenegl,
                  const PointXfrm* xfrm,
                  const Point* xlat)
{
    struct Arg_view_4d_kernel
    {
        PointXfrm xfrm;
        Point xlat;
    } arg;
    const uint dev_idx = 0;
    SysOpenCL* cl = &opencl_state;
    int err = 0;
    uint surfi;
    uint argi = 0;
    cl_mem ret_verts, ret_vnmls;
    cl_mem gl_objects[2];
    cl_mem vidcs, verts, vnmls, arg_mem;

    ret_verts = clCreateFromGLBuffer (cl->context, CL_MEM_WRITE_ONLY,
                                      scenegl->verts_buffer, &err);
    AssertStatus( err, "create from gl verts buffer" );

    ret_vnmls = clCreateFromGLBuffer (cl->context, CL_MEM_WRITE_ONLY,
                                      scenegl->vnmls_buffer, &err);
    AssertStatus( err, "create from gl vnmls buffer" );

    gl_objects[0] = ret_verts;
    gl_objects[1] = ret_vnmls;

    err |= clEnqueueAcquireGLObjects (cl->comqs[dev_idx],
                                      ArraySz( gl_objects ), gl_objects,
                                      0, 0, 0);
    AssertStatus( err, "acquire gl objects" );

    vidcs = clCreateBuffer (cl->context,
                            CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                            scene->nelems * NDimensions * sizeof(uint),
                            scene->vidcs, &err);
    AssertStatus( err, "" );
    verts = clCreateBuffer (cl->context,
                            CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                            scene->nverts * sizeof(Point),
                            scene->verts, &err);
    AssertStatus( err, "" );
    vnmls = clCreateBuffer (cl->context,
                            CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                            scene->nvnmls * sizeof(Point),
                            scene->vnmls, &err);
    AssertStatus( err, "" );

    arg.xfrm = *xfrm;
    arg.xlat = *xlat;

    arg_mem = clCreateBuffer (cl->context,
                              CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                              sizeof(arg),
                              &arg, &err);
    AssertStatus( err, "" );

        /* Set the arguments to our compute kernel.*/
    err |= clSetKernelArg (cl->kernel, argi++, sizeof(ret_verts), &ret_verts);
    err |= clSetKernelArg (cl->kernel, argi++, sizeof(ret_vnmls), &ret_vnmls);
    err |= clSetKernelArg (cl->kernel, argi++, sizeof(vidcs), &vidcs);
    err |= clSetKernelArg (cl->kernel, argi++, sizeof(verts), &verts);
    err |= clSetKernelArg (cl->kernel, argi++, sizeof(vnmls), &vnmls);
    err |= clSetKernelArg (cl->kernel, argi++, sizeof(arg_mem), &arg_mem);
    AssertStatus( err, "set kernel args" );

    UFor( surfi, scene->nsurfs )
    {
        size_t global, local;
        const GeomSurf* surf;
        uint elem_offset;
        uint i;

        surf = &scene->surfs[surfi];
        elem_offset = surf->vidcs_offset / NDimensions;

        i = 0;
        err |= clSetKernelArg (cl->kernel, argi + i++, sizeof(elem_offset), &elem_offset);
        err |= clSetKernelArg (cl->kernel, argi + i++, sizeof(surf->verts_offset), &surf->verts_offset);
        err |= clSetKernelArg (cl->kernel, argi + i++, sizeof(surf->vnmls_offset), &surf->vnmls_offset);
        AssertStatus( err, "set kernel args in loop" );

            /* Get the maximum work group size for executing the kernel on the device.*/
        err = clGetKernelWorkGroupInfo(cl->kernel, cl->devices[dev_idx],
                                       CL_KERNEL_WORK_GROUP_SIZE,
                                       sizeof(local), &local, 0);
        AssertStatus( err, "retrieve kernel work group info" );

        global = surf->nelems;
            /* err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, &local, 0, NULL, NULL); */
        err = clEnqueueNDRangeKernel (cl->comqs[dev_idx], cl->kernel, 1, 0, &global, 0, 0, 0, 0);
        AssertStatus( err, "enqueue kernel" );
    }

    err |= clEnqueueReleaseGLObjects (cl->comqs[dev_idx],
                                      ArraySz( gl_objects ), gl_objects,
                                      0, 0, 0);
    AssertStatus( err, "release gl objects" );

        /* Wait for the commands to get serviced before reading back results.*/
    clFinish (cl->comqs[dev_idx]);

    clReleaseMemObject (vidcs);
    clReleaseMemObject (verts);
    clReleaseMemObject (vnmls);
    clReleaseMemObject (arg_mem);
}

