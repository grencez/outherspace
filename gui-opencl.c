
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
check_cl_status (cl_int err, const char* str)
{
    FILE* out;
    out = stderr;

    if (err != CL_SUCCESS)
    {
        fprintf (out, "Error %d at %s.\n", err, str);
        exit (1);
    }
}

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
    check_cl_status (err, "query platform count");

    platforms = AllocT( cl_platform_id, nplatforms );

    err = clGetPlatformIDs (nplatforms, platforms, 0);
    check_cl_status (err, "fetch platforms");

    UFor( i, nplatforms )
    {
        cl_uint n;
        err = clGetDeviceIDs (platforms[i], CL_DEVICE_TYPE_ALL,
                                 0, 0, &n);
        check_cl_status (err, "query device count");
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
        check_cl_status (err, "fetch devices");
        device_offset += n;
    }

    if (platforms)  free (platforms);

        /* Create a compute context.*/
    context = clCreateContext (context_props, ndevices, devices, NULL, NULL, &err);
    check_cl_status (err, "create context");

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
        check_cl_status (err, "platform");
            /* Create a command queue.*/
        comqs[i] = clCreateCommandQueue (context, devices[i], 0, &err);
        check_cl_status (err, "create command queue");
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
              uint nfnames, const char* const* fnames)
{
    uint i;
    size_t* sizes;
    char** bufs;
    cl_program program;
    cl_int err;

    bufs = AllocT( char*, nfnames );
    sizes = AllocT( size_t, nfnames );

    UFor( i, nfnames )
    {
        long len;
        FILE* in;
        in = fopen (fnames[i], "rb");
        assert (in);
        fseek (in, 0, SEEK_END);
        len = ftell (in);
        fseek (in, 0, SEEK_SET);

        bufs[i] = AllocT( char, (size_t) len );
        assert (bufs[i]);

        sizes[i] = fread (bufs[i], sizeof(char), (size_t)len, in);
        fclose (in);
        assert (sizeof(char) * (size_t)len == sizes[i]);
    }

    program = clCreateProgramWithSource (context, nfnames,
                                         (const char**) bufs,
                                         sizes, &err);
    check_cl_status (err, "create program");

    err = clBuildProgram (program, ndevices, devices, 0, 0, 0);
    if (err != CL_SUCCESS)
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
            check_cl_status (be, "build info");
            if (build_stat != CL_BUILD_SUCCESS) {
                char* log;
                size_t size;
                FILE* out;
                out = stderr;
                be = clGetProgramBuildInfo (program, devices[i],
                                            CL_PROGRAM_BUILD_LOG,
                                            0, 0, &size);
                check_cl_status (be, "build log size");
                log = AllocT( char, 1+size/sizeof(char) );
                log[size/sizeof(char)] = 0;

                be = clGetProgramBuildInfo (program, devices[i],
                                            CL_PROGRAM_BUILD_LOG,
                                            size, log, &size);
                check_cl_status (be, "build log");
                fputs ("Failure building for device.\n", out);
                fputs (log, out);
                fputs ("\n", out);
                if (log)  free (log);
            }
        }
    }
    check_cl_status (err, "build program");

    UFor( i, nfnames )  free (bufs[i]);
    if (bufs)  free (bufs);
    if (sizes)  free (sizes);

    *ret_program = program;
}

static void
init_opencl_data ()
{
    int err;
    SysOpenCL* cl = &opencl_state;
    const char* fnames[] = { "perturb.cl" };
    const uint nfnames = ArraySz( fnames );
    
    cl->ndevices = 0;
    compute_devices (&cl->ndevices, &cl->devices, &cl->context, &cl->comqs);
    printf ("I have %u devices!\n", cl->ndevices);

    load_program (&cl->program, cl->context, cl->ndevices, cl->devices,
                  nfnames, fnames);

        /* Create the compute kernel in the program we wish to run.*/
    cl->kernel = clCreateKernel (cl->program, "perturb_kernel", &err);
    check_cl_status (err, "create compute kernel");
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
    check_cl_status (err, "create from gl buffer");

    err = clEnqueueAcquireGLObjects (cl->comqs[dev_idx], 1, &verts, 0, 0, 0);
    check_cl_status (err, "acquire gl objects");

        /* Set the arguments to our compute kernel.*/
    err  = clSetKernelArg(cl->kernel, 0, sizeof(cl_mem), &verts);
    err |= clSetKernelArg(cl->kernel, 1, sizeof(uint), &count);
    check_cl_status (err, "set kernel arguments");

        /* Get the maximum work group size for executing the kernel on the device.*/
    err = clGetKernelWorkGroupInfo(cl->kernel, cl->devices[dev_idx],
                                   CL_KERNEL_WORK_GROUP_SIZE,
                                   sizeof(local), &local, 0);
    check_cl_status (err, "retrieve kernel work group info");

    global = scene->nverts;
        /* err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, &local, 0, NULL, NULL); */
    err = clEnqueueNDRangeKernel (cl->comqs[dev_idx], cl->kernel, 1, 0, &global, 0, 0, 0, 0);
    check_cl_status (err, "enqueue kernel");

    err = clEnqueueReleaseGLObjects (cl->comqs[dev_idx], 1, &verts, 0, 0, 0);
    check_cl_status (err, "release gl objects");

        /* Wait for the command commands to get serviced before reading back results.*/
    clFinish (cl->comqs[dev_idx]);
}

