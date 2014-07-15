
__kernel
    void
square_kernel (__global float* input,
               __global float* output,
               const uint count)
{
    int i = get_global_id(0);
    output[i] = input[i] * input[i];
}

__kernel
  void
ray_cast_kernel (__write_only __global unsigned int* hits,
                 __read_only __global const RayCastAPriori* known,
                 __read_only __global const uint* dims,
                 __read_only __global const BBox* box,
                 __read_only __global const Simplex* elems,
                 __read_only __global const BarySimplex* simplices,
                 __read_only __global const uint* elemidcs,
                 __read_only __global const KDTreeNode* nodes)
{
  uint row = get_global_id(0);
  uint col = get_global_id(1);
  const uint nelems = dims[3];
  Ray ray;
  uint hit = Max_uint;
  real mag = Max_real;
  uint i;

  ray.origin = known->origin;
  zero_Point (&ray.direct);
  ray.direct.coords[UpDim] = known->up_scale * (-1 + (2*row+1.0) / dims[0]);
  ray.direct.coords[RtDim] = known->rt_scale * (-1 + (2*col+1.0) / dims[1]);
  ray.direct.coords[FwDim] = 1;
  {
    PointXfrm basis = known->basis;
    trxfrm_Point (&ray.direct, &basis, &ray.direct);
  }
  normalize_Point (&ray.direct, &ray.direct);

  cast_Ray (&hit, &mag, &ray,
            nelems,
            elemidcs,
            nodes,
            simplices,
            elems,
            box,
            known->inside_box,
            Yes);
  hits[row * dims[2] + col] = hit;
}

