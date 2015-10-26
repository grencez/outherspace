
#include "pnm-image.h"
#include "testcase.h"
#include "track.h"
#include "dynamic-setup.h"
#include "space.h"
#include "point.h"
#include "xfrm.h"
#include "cx/syscx.h"
#include "cx/alphatab.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#define ENABLE_SMOKETEST


#if 0
static
  void
ray_cast_kernel (uint row, uint col,
                 __global unsigned int* hits,
                 __global const RayCastAPriori* known,
                 __global const uint* dims,
                 __global const BBox* box,
                 __global const uint* nelems,
                 __global const Simplex* elems,
                 __global const BarySimplex* simplices,
                 __global const uint* elemidcs,
                 __global const KDTreeNode* nodes)
{
  Ray ray;
  uint hit = Max_uint;
  real mag = Max_real;

  ray.origin = known->origin;
  zero_Point (&ray.direct);
  ray.direct.coords[UpDim] = known->up_scale * (-1 + (2*row+1.0) / dims[0]);
  ray.direct.coords[RtDim] = known->rt_scale * (-1 + (2*col+1.0) / dims[1]);
  ray.direct.coords[FwDim] = 1;
  trxfrm_Point (&ray.direct, &known->basis, &ray.direct);
  normalize_Point (&ray.direct, &ray.direct);

  cast_Ray (&hit, &mag, &ray,
            *nelems,
            elemidcs,
            nodes,
            simplices,
            elems,
            box,
            known->inside_box,
            Yes);
  hits[row * dims[2] + col] = hit;
}
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

    AllocTo( platforms, nplatforms );

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

    /* ndevices = 1; */
    AllocTo( devices, ndevices );

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

    AllocTo( comqs, ndevices );
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

    AllocTo( bufs, nfnames );
    AllocTo( sizes, nfnames );

    UFor( i, nfnames )
    {
        long len;
        FILE* in;

        if (eq_cstr ("color.h", fnames[i])) {
          const char color_pfx[] =
            "#undef NDims\n#define NDims NColors\n#define Point Color\n";
          const char color_sfx[] =
            "\n#undef NDims\n#define NDims NDimensions\n";
          len = strlen(color_pfx) + sizes[i-1] + strlen(color_sfx);
          AllocTo( bufs[i], (size_t) len );
          sizes[i] = len;
          strncpy (bufs[i], color_pfx, strlen(color_pfx));
          strncpy (bufs[i]+strlen(color_pfx), bufs[i-1], sizes[i-1]);
          strncpy (bufs[i]+strlen(color_pfx)+sizes[i-1], color_sfx, strlen(color_sfx));
          continue;
        }

        in = fopen (fnames[i], "rb");
        assert (in);
        fseek (in, 0, SEEK_END);
        len = ftell (in);
        fseek (in, 0, SEEK_SET);

        AllocTo( bufs[i], (size_t) len );
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
    if (false || err != CL_SUCCESS)
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
            if (false || build_stat != CL_BUILD_SUCCESS) {
                char* log;
                size_t size;
                FILE* out;
                out = stderr;
                be = clGetProgramBuildInfo (program, devices[i],
                                            CL_PROGRAM_BUILD_LOG,
                                            0, 0, &size);
                check_cl_status (be, "build log size");
                AllocTo( log, 1+size/sizeof(char) );
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

#ifdef ENABLE_SMOKETEST
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
          "bld/cx/def.h",
          "bld/cx/synhax.h",
          "bld/outherspace/util.h",
          "bld/outherspace/util.c",
          "bld/outherspace/op.h",
          "bld/outherspace/space.h",
          "bld/outherspace/point.h",
          "bld/outherspace/space-junk.h",
          "bld/outherspace/xfrm.h",
          "bld/outherspace/xfrm.c",
          "bld/outherspace/affine.h",
          "bld/outherspace/affine.c",
          "bld/outherspace/simplex.h",
          "bld/outherspace/simplex.c",
          "bld/outherspace/bbox.h",
          "bld/outherspace/bbox.c",
          "bld/outherspace/kdtree.h",
          "bld/outherspace/kdtree.c",
          "bld/cx/table.h",
          "bld/cx/bstree.h",
          "bld/cx/bstree.c",
          "bld/outherspace/material.h",
          "bld/outherspace/scene.h",
          "bld/outherspace/kptree.h",
          "bld/outherspace/raytrace.h",
          "bld/outherspace/raytrace.c",
          "../hello.cl"
        };
        const uint nfnames = ArraySz( fnames );
        /* ndevices = 1; */
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
#endif

#if 1
static
    void
run_ray_cast ()
{
  int err;
  uint i;
  RaySpace space;
  RayImage image;
  RayCastAPriori raycast_apriori;
  Track track;
  real t0, t1;

  cl_program program;
  size_t global_work_size[2];
  cl_kernel kernel;
  uint dims[4];

  uint ndevices = 0;
  cl_device_id* devices;
  cl_context context;
  cl_command_queue* comqs;
  const uint dev_idx = 0;

  cl_mem kernel_args[20];
  uint n_kernel_args = 0;

  size_t hits_size;
  ObjectRaySpace* object;

  init_Track (&track);
  init_RayImage (&image);

  image.nrows = 500;
  image.ncols = 500;
  AllocTo( image.hits, 1 );

  if (!readin_Track (&track, &space, "data", "rgtest.txt")) {
    DBog0( "Can't read file!" );
    return;
  }
  {
    Point view_origin;
    PointXfrm view_basis;
    view_origin = track.camloc.xlat;
    transpose_PointXfrm (&view_basis, &track.camloc.xfrm);
    image.nrows = track.nimgrows;
    image.ncols = track.nimgcols;
    setup_RayCastAPriori (&raycast_apriori, &image, &view_origin, &view_basis,
                          &space.box);
  }

  global_work_size[0] = image.nrows;
  global_work_size[1] = image.ncols;
  hits_size = image.nrows * image.ncols * sizeof (uint);

  compute_devices (&ndevices, &devices, &context, &comqs);
  printf ("I have %u devices!\n", ndevices);

  {
    const char* fnames[] =
    {
      "bld/cx/def.h",
      "bld/cx/synhax.h",
      "bld/outherspace/util.h",
      "bld/outherspace/util.c",
      "bld/outherspace/op.h",
      "bld/outherspace/space.h",
      "bld/outherspace/point.h",
      "bld/outherspace/space-junk.h",
      "bld/outherspace/xfrm.h",
      "bld/outherspace/xfrm.c",
      "bld/outherspace/affine.h",
      "bld/outherspace/affine.c",
      "bld/outherspace/simplex.h",
      "bld/outherspace/simplex.c",
      "bld/outherspace/bbox.h",
      "bld/outherspace/bbox.c",
      "bld/outherspace/kdtree.h",
      "bld/outherspace/kdtree.c",
      "bld/cx/table.h",
      "bld/cx/bstree.h",
      "bld/cx/bstree.c",
      "bld/outherspace/material.h",
      "bld/outherspace/scene.h",
      "bld/outherspace/kptree.h",
      "bld/outherspace/raytrace.h",
      "bld/outherspace/raytrace.c",
      "../hello.cl"
    };
    const uint nfnames = ArraySz( fnames );
    /* ndevices = 1; */
    load_program (&program, context, ndevices, devices, nfnames, fnames);
    /* load_program (&program, context, 1, devices, nfnames, fnames); */
  }

  /* Create the compute kernel in the program we wish to run.*/
  kernel = clCreateKernel (program, "ray_cast_kernel", &err);
  check_cl_status (err, "create compute kernel");

  object = &space.main;
  resize_RayImage (&image);
  t0 = monotime ();
  update_dynamic_RaySpace (&space);
  t1 = monotime ();
  printf ("Kd-tree build sec:%f\n", t1 - t0);
  t0 = t1;


#if 1
  kernel_args[n_kernel_args++] =
    clCreateBuffer (context,
                    CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR,
                    hits_size, image.hits, &err);
  check_cl_status (err, "alloc hits buffer");
#else
  ret_hits = clCreateBuffer (context, CL_MEM_WRITE_ONLY,
                             hits_size, 0, &err);
  check_cl_status (err, "alloc hits buffer");
#endif

  kernel_args[n_kernel_args++] =
    clCreateBuffer (context,
                    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                    sizeof (RayCastAPriori),
                    &raycast_apriori, &err);
  check_cl_status (err, "alloc params buffer");

  dims[0] = image.nrows;
  dims[1] = image.ncols;
  dims[2] = image.stride;
  dims[3] = object->nelems;
  kernel_args[n_kernel_args++] =
    clCreateBuffer (context,
                    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                    sizeof(dims),
                    dims, &err);
  check_cl_status (err, "alloc dims buffer");

  kernel_args[n_kernel_args++] =
    clCreateBuffer (context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                    sizeof (BBox),
                    &object->box, &err);
  check_cl_status (err, "alloc bounding box");

  kernel_args[n_kernel_args++] =
    clCreateBuffer (context,
                    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                    object->nelems * sizeof (Simplex),
                    object->elems, &err);
  check_cl_status (err, "alloc elems buffer");

  kernel_args[n_kernel_args++] =
    clCreateBuffer (context,
                    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                    object->nelems * sizeof (BarySimplex),
                    object->simplices, &err);
  check_cl_status (err, "alloc barycentric elems buffer");

  kernel_args[n_kernel_args++] =
    clCreateBuffer (context,
                    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                    object->tree.nelemidcs * sizeof (uint),
                    object->tree.elemidcs, &err);
  check_cl_status (err, "alloc elemidcs buffer");

  kernel_args[n_kernel_args++] =
    clCreateBuffer (context,
                    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                    object->tree.nnodes * sizeof (KDTreeNode),
                    object->tree.nodes, &err);
  check_cl_status (err, "alloc nodes buffer");


  UFor( i, n_kernel_args )
  {
    char msg[100];
    /* Set the arguments to our compute kernel.*/
    err  = clSetKernelArg (kernel, i, sizeof(cl_mem), &kernel_args[i]);
    sprintf (msg, "set kernel argument %u", i);
    check_cl_status (err, msg);
  }

  t0 = monotime ();
#if 0
  {
    const uint n_work_rows = 2;
    const uint n_work_cols = 2;
  UFor( i, n_work_rows )
  {
    uint j;
    UFor( j, n_work_cols )
    {
      size_t work_offset[2];
      size_t work_size[2];

      work_size[0] = image.nrows / n_work_rows;
      work_size[1] = image.ncols / n_work_cols;

      work_offset[0] = i * work_size[0];
      work_offset[1] = j * work_size[1];
      /* if ((i+j) % 2 == 0) continue; */

      err = clEnqueueNDRangeKernel (comqs[dev_idx], kernel, 2,
                                    work_offset,
                                    work_size, 0, 0, 0, 0);
      check_cl_status (err, "enqueue kernel");
      err = clFinish (comqs[dev_idx]);
      check_cl_status (err, "finish kernel");
    }
  }
  }
#else
  err = clEnqueueNDRangeKernel (comqs[dev_idx], kernel, 2, 0,
                                global_work_size, 0, 0, 0, 0);
  check_cl_status (err, "enqueue kernel");

  /* Wait for the command commands to get serviced before reading back results.*/
  err = clFlush (comqs[dev_idx]);
  err = clFinish (comqs[dev_idx]);
  check_cl_status (err, "finish kernel");
#endif
  t1 = monotime ();
  printf ("Render sec:%f\n", t1 - t0);

#if 1
  /* Read back the results from the device to verify the output */
  err = clEnqueueReadBuffer (comqs[dev_idx], kernel_args[0], CL_TRUE,
                             0, hits_size, image.hits, 0, 0, 0);
  check_cl_status (err, "read output hits");
#endif

  /* Shutdown and cleanup.*/
  UFor( i, n_kernel_args )
  {
    clReleaseMemObject (kernel_args[i]);
  }
  clReleaseProgram (program);
  clReleaseKernel (kernel);
  UFor( i, ndevices )
    clReleaseCommandQueue (comqs[i]);
  clReleaseContext (context);

  if (devices)  free (devices);
  if (comqs)  free (comqs);

#if 0
#pragma omp parallel for
  for (i = 0; i < image.nrows; ++i) {
    for (uint j = 0; j < image.ncols; ++j) {
      ray_cast_kernel (i, j,
                       image.hits,
                       &raycast_apriori,
                       dims,
                       &object->box,
                       &object->nelems,
                       object->elems,
                       object->simplices,
                       object->tree.elemidcs,
                       object->tree.nodes);
    }
  }
#elif 0
#pragma omp parallel for
  for (i = 0; i < image.nrows; ++i) {
    cast_row_perspective (&image, i, &space, &raycast_apriori);
  }
#elif 0
  cast_RayImage (&image, &space, &raycast_apriori.origin, &raycast_apriori.basis);
#endif

#ifndef BENCHMARKING
  output_PGM_image ("out.pgm", image.nrows, image.ncols, image.hits, object->nelems);
#endif
  cleanup_RayImage (&image);
  cleanup_RaySpace (&space);
}
#endif

int main (int argc, char** argv)
{
  int argi =
    (init_sysCx (&argc, &argv),
     1);
  (void) argi;
#ifdef ENABLE_SMOKETEST
  run_hello ();
#endif
  run_ray_cast ();
  lose_sysCx ();
  return 0;
}

