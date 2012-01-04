
typedef float real;

__kernel
    void
perturb_kernel (__global real* verts, const uint count)
{
    int i = get_global_id(0);
    verts[3*i+0] += i / 50.0 * (((count + 1*i) % 41) - 20.0) / 20;
    verts[3*i+1] += i / 50.0 * (((count + 2*i) % 41) - 20.0) / 20;
    verts[3*i+2] += i / 50.0 * (((count + 3*i) % 41) - 20.0) / 20;
}

