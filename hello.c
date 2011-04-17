
#include "main.h"
#include "testcase.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif


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
    cl_int err;
    uint i;
    cl_uint nplatforms = 0, ndevices = 0;
    cl_uint device_offset;

    cl_platform_id* platforms;
    cl_device_id* devices;
    cl_context context;
    cl_command_queue* comqs;

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
    context = clCreateContext (0, ndevices, devices, NULL, NULL, &err);
    check_cl_status (err, "create context");

    comqs = AllocT( cl_command_queue, ndevices );
    UFor( i, ndevices )
    {
        char buf[BUFSIZ];
        err = clGetDeviceInfo (devices[i], CL_DRIVER_VERSION,
                               BUFSIZ-1, buf, 0);
        buf[BUFSIZ-1] = 0;
        fputs (buf, stderr);
        fputc ('\n', stderr);
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

static
    void
run_hello ()
{
    int err;
      
    uint count = 1024*1024;
    real* data = AllocT( real, count );
    real* results = AllocT( real, count );
    uint correct;

    size_t global, local;

    cl_program program;
    cl_kernel kernel;
    
    uint ndevices = 0;
    cl_device_id* devices;
    cl_context context;
    cl_command_queue* comqs;
    const uint dev_idx = 0;

    cl_mem input, output;
    
        /* Fill our data set with random float values.*/
    uint i;

    UFor( i, count )
        data[i] = rand() / (real)RAND_MAX;
    
    compute_devices (&ndevices, &devices, &context, &comqs);
    printf ("I have %u devices!\n", ndevices);

    {
        const char* fnames[] =
        {
            "util.h", "util.c", "space.h", "space.c",
            "scene.h", "kdtree.h", "kdtree.c",
            "xfrm.h", "xfrm.c", "raytrace.h", "raytrace.c",
            "hello.cl"
        };
        const uint nfnames = sizeof(fnames) / sizeof(const char*);
        load_program (&program, context, ndevices, devices, nfnames, fnames);
    }

        /* Create the compute kernel in the program we wish to run.*/
    kernel = clCreateKernel (program, "square_kernel", &err);
    check_cl_status (err, "create compute kernel");

        /* Create the input and output arrays in device memory for our calculation.*/
    input  = clCreateBuffer (context, CL_MEM_READ_ONLY,  sizeof(real) * count, 0, &err);
    check_cl_status (err, "alloc read mem");
    output = clCreateBuffer (context, CL_MEM_WRITE_ONLY, sizeof(real) * count, 0, &err);
    check_cl_status (err, "alloc write mem");
    
        /* Write our data set into the input array in device memory.*/
    err = clEnqueueWriteBuffer(comqs[dev_idx], input, CL_TRUE, 0, sizeof(real) * count, data, 0,  0, 0);
    check_cl_status (err, "write source array");

        /* Set the arguments to our compute kernel.*/
    err  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &input);
    err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &output);
    err |= clSetKernelArg(kernel, 2, sizeof(unsigned int), &count);
    check_cl_status (err, "set kernel arguments");

        /* Get the maximum work group size for executing the kernel on the device.*/
    err = clGetKernelWorkGroupInfo(kernel, devices[dev_idx],
                                   CL_KERNEL_WORK_GROUP_SIZE,
                                   sizeof(local), &local, 0);
    check_cl_status (err, "retrieve kernel work group info");

    global = count;
        /* err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, &local, 0, NULL, NULL); */
    err = clEnqueueNDRangeKernel (comqs[dev_idx], kernel, 1, 0, &global, 0, 0, 0, 0);
    check_cl_status (err, "enqueue kernel");

        /* Wait for the command commands to get serviced before reading back results.*/
    clFinish(comqs[dev_idx]);

        /* Read back the results from the device to verify the output */
    err = clEnqueueReadBuffer (comqs[dev_idx], output, CL_TRUE, 0, sizeof(real) * count, results, 0, NULL, NULL);
    check_cl_status (err, "read output array");
    
#if 1
        /* Validate our results.*/
    correct = 0;
    UFor( i, count )
    {
        if (results[i] == data[i] * data[i])
            correct++;
            /* else printf ("\nexpect:%.15f\nresult:%.15f\n", results[i], data[i] * data[i]); */
    }
    
        /* Print a brief summary detailing the results.*/
    printf("Computed '%d/%d' correct values!\n", correct, count);
#endif
    
        /* Shutdown and cleanup.*/
    clReleaseMemObject(input);
    clReleaseMemObject(output);
    clReleaseProgram(program);
    clReleaseKernel(kernel);
    UFor( i, ndevices )
        clReleaseCommandQueue(comqs[i]);
    clReleaseContext(context);

    free (data);
    free (results);
}

static
    void
run_ray_cast ()
{
    int err;
    uint i;

    cl_program program;
    size_t global_work_size[2];
    cl_kernel kernel;
    
    uint ndevices = 0;
    cl_device_id* devices;
    cl_context context;
    cl_command_queue* comqs;
    const uint dev_idx = 0;
    real view_angle = 2 * M_PI / 3;

    cl_mem ret_hits, inp_params, inp_elems, inp_elemidcs, inp_nodes;

    RaySpace space;
    Point view_origin;
    PointXfrm view_basis;
    MultiRayCastParams params;

    const uint nrows = 5000;
    const uint ncols = 5000;
    uint* hits;
    size_t hits_size;

    global_work_size[0] = nrows;
    global_work_size[1] = ncols;

    setup_testcase_triangles (&space, &view_origin, &view_angle);

    partition_RaySpace (&space);
    identity_PointXfrm (&view_basis);

    hits = AllocT( uint, nrows * ncols );
    hits_size = nrows * ncols * sizeof (uint);

    build_MultiRayCastParams (&params, nrows, ncols, &space,
                              &view_origin, &view_basis,
                              view_angle);


    compute_devices (&ndevices, &devices, &context, &comqs);
    printf ("I have %u devices!\n", ndevices);

    {
        const char* fnames[] =
        {
            "util.h", "util.c", "space.h", "space.c",
            "scene.h", "kdtree.h", "kdtree.c",
            "xfrm.h", "xfrm.c", "raytrace.h", "raytrace.c",
            "hello.cl"
        };
        const uint nfnames = sizeof(fnames) / sizeof(const char*);
        load_program (&program, context, ndevices, devices, nfnames, fnames);
    }

        /* Create the compute kernel in the program we wish to run.*/
    kernel = clCreateKernel (program, "ray_cast_kernel", &err);
    check_cl_status (err, "create compute kernel");


#if 1
    ret_hits = clCreateBuffer (context,
                               CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR,
                               hits_size, hits, &err);
    check_cl_status (err, "alloc hits buffer");
#else
    ret_hits = clCreateBuffer (context, CL_MEM_WRITE_ONLY,
                               hits_size, 0, &err);
    check_cl_status (err, "alloc hits buffer");
#endif

    inp_params = clCreateBuffer (context,
                                 CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                 sizeof (MultiRayCastParams),
                                 &params, &err);
    check_cl_status (err, "alloc params buffer");

    inp_elems = clCreateBuffer (context,
                                CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                space.nelems * sizeof (Triangle),
                                space.elems, &err);
    check_cl_status (err, "alloc elems buffer");

    inp_elemidcs = clCreateBuffer (context,
                                   CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                   space.tree.nelemidcs * sizeof (uint),
                                   space.tree.elemidcs, &err);
    check_cl_status (err, "alloc elemidcs buffer");

    inp_nodes = clCreateBuffer (context,
                                CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                space.tree.nnodes * sizeof (KDTreeNode),
                                space.tree.nodes, &err);
    check_cl_status (err, "alloc nodes buffer");


        /* Set the arguments to our compute kernel.*/
    err  = clSetKernelArg (kernel, 0, sizeof(cl_mem), &ret_hits);
    check_cl_status (err, "set kernel argument 0");

    err = clSetKernelArg (kernel, 1, sizeof(cl_mem), &inp_params);
    check_cl_status (err, "set kernel argument 1");

    err = clSetKernelArg (kernel, 2, sizeof(cl_mem), &inp_elems);
    check_cl_status (err, "set kernel argument 2");

    err = clSetKernelArg (kernel, 3, sizeof(cl_mem), &inp_elemidcs);
    check_cl_status (err, "set kernel argument 3");

    err = clSetKernelArg (kernel, 4, sizeof(cl_mem), &inp_nodes);
    check_cl_status (err, "set kernel argument 4");

#if 1
    UFor( i, 10 )
    {
        uint j;
        UFor( j, 10 )
        {
            size_t work_offset[2];
            size_t work_size[2];

            work_size[0] = nrows / 10;
            work_size[1] = ncols / 10;

            work_offset[0] = i * work_size[0];
            work_offset[1] = j * work_size[1];

            err = clEnqueueNDRangeKernel (comqs[dev_idx], kernel, 2,
                                          work_offset,
                                          work_size, 0, 0, 0, 0);
            check_cl_status (err, "enqueue kernel");
            err = clFinish (comqs[dev_idx]);
            check_cl_status (err, "finish kernel");
        }
    }
#else
    err = clEnqueueNDRangeKernel (comqs[dev_idx], kernel, 2, 0,
                                  global_work_size, 0, 0, 0, 0);
    check_cl_status (err, "enqueue kernel");

        /* Wait for the command commands to get serviced before reading back results.*/
    err = clFinish (comqs[dev_idx]);
    check_cl_status (err, "finish kernel");
#endif

#if 0
        /* Read back the results from the device to verify the output */
    err = clEnqueueReadBuffer (comqs[dev_idx], ret_hits, CL_TRUE,
                               0, hits_size, hits, 0, 0, 0);
    check_cl_status (err, "read output hits");
#endif


        /* Shutdown and cleanup.*/
    clReleaseMemObject (ret_hits);
    clReleaseMemObject (inp_params);
    clReleaseMemObject (inp_elems);
    clReleaseMemObject (inp_elemidcs);
    clReleaseMemObject (inp_nodes);
    clReleaseProgram (program);
    clReleaseKernel (kernel);
    UFor( i, ndevices )
        clReleaseCommandQueue (comqs[i]);
    clReleaseContext (context);

    if (devices)  free (devices);
    if (comqs)  free (comqs);

#ifndef BENCHMARKING
    output_PGM_image ("out.pgm", nrows, ncols, hits, space.nelems);
#endif
    free (hits);
    cleanup_RaySpace (&space);
}

int main ()
{
        /* run_hello (); */
    run_ray_cast ();
    return 0;
}

