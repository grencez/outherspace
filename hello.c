
#include "util.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
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
                check_cl_status (be, "build log");
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

int main(int argc, char** argv)
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

    (void) argc;
    (void) argv;

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
    kernel = clCreateKernel (program, "square", &err);
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
    for(i = 0; i < count; i++)
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
    return 0;
}

