
#include "cx/def.h"
#include "pnm-image.h"
#include "space.h"

#include <assert.h>
#include <stdio.h>

int main (int argc, const char* const* argv)
{
    uint i, n;
    uint lhs_nrows, lhs_ncols;
    byte* lhs_bytes;
    const char* lhs_file;
    uint rhs_nrows, rhs_ncols;
    byte* rhs_bytes;
    const char* rhs_file;
    const char* dst_file;
    FILE* out = stderr;

    if (argc != 4)
    {
        fprintf (out, "Usage: %s LHS RHS DST\n", argv[0]);
        fputs ("  Where LHS and RHS are .ppm images to diff,\n", out);
        fputs ("  and DST is a .ppm image to output.\n", out);
        exit (1);
    }

    lhs_file = argv[1];
    rhs_file = argv[2];
    dst_file = argv[3];

    lhs_bytes = readin_PPM_image (&lhs_nrows, &lhs_ncols, ".", lhs_file);
    rhs_bytes = readin_PPM_image (&rhs_nrows, &rhs_ncols, ".", rhs_file);

    assert (lhs_nrows == rhs_nrows);
    assert (lhs_ncols == rhs_ncols);

    n = NColors * lhs_nrows * lhs_ncols;
    UFor( i, n )
    {
        if (lhs_bytes[i] >= rhs_bytes[i])
            lhs_bytes[i] -= rhs_bytes[i];
        else
            lhs_bytes[i] = rhs_bytes[i] - lhs_bytes[i];
    }

    output_PPM_image (dst_file, lhs_nrows, lhs_ncols, lhs_bytes);

    free (lhs_bytes);
    free (rhs_bytes);

    return 0;
}

