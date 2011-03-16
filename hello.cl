
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
                 __read_only __global const MultiRayCastParams* params,
                 __read_only __global const Triangle* elems,
                 __read_only __global const uint* elemidcs,
                 __read_only __global const KDTreeNode* nodes)
{
    uint row, col;
    Point partial_dir, dir, origin;
    uint hit; real mag;

    row = get_global_id(0);
    col = get_global_id(1);

    dir = params->dir_start;

    partial_dir = params->dir_delta[0];
    scale_Point (&partial_dir, &partial_dir, params->npixels[0] - row -1);
    summ_Point (&dir, &dir, &partial_dir);

    partial_dir = params->dir_delta[1];
    scale_Point (&partial_dir, &partial_dir, col);
    summ_Point (&dir, &dir, &partial_dir);

    normalize_Point (&dir, &dir);

    origin = params->origin;

    cast_ray (&hit, &mag, &origin, &dir,
              params->nelems, elems,
              elemidcs, nodes,
              &params->box, params->inside_box);
    hits[row * params->npixels[1] + col] = hit;
}

