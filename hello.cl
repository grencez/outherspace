
__kernel
    void
square (__global float* input,
        __global float* output,
        const uint count)
{
    int i = get_global_id(0);
    output[i] = input[i] * input[i];
}

