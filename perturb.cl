
__kernel
    void
perturb_kernel (__global float* verts, const uint count)
{
    int i = get_global_id(0);
    verts[3*i] += (((count + i) % 41) - 20.0) / 20;
}

