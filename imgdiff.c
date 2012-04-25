
#include "cx/def.h"
#include "pnm-image.h"
#include "space.h"

#include <assert.h>
#include <stdio.h>

int main (int argc, const char* const* argv)
{
    uint n;
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

    lhs_bytes = readin_PPM_image (&lhs_nrows, &lhs_ncols, 0, lhs_file);
    rhs_bytes = readin_PPM_image (&rhs_nrows, &rhs_ncols, 0, rhs_file);

    fprintf (stderr, "nrows:%u ncols:%u\n", lhs_nrows, lhs_ncols);
    Claim2( lhs_nrows ,==, rhs_nrows );
    Claim2( lhs_ncols ,==, rhs_ncols );

    n = NColors * lhs_nrows * lhs_ncols;
    { BLoop( i, lhs_nrows )
        { BLoop( j, lhs_ncols )
            Color diff;
            byte* c0 = &lhs_bytes[(j + i * lhs_ncols) * NColors];
            byte* c1 = &rhs_bytes[(j + i * lhs_ncols) * NColors];
            { BLoop( k, NColors )
                diff.coords[k] = ((real) c1[k] - (real) c0[k]) / 255;

                c0[k] = ((c0[k] >= c1[k])
                         ? c0[k] - c1[k]
                         : c1[k] - c0[k]);
            } BLose()
        } BLose()
    } BLose()

    output_PPM_image (dst_file, lhs_nrows, lhs_ncols, lhs_bytes);

    free (lhs_bytes);
    free (rhs_bytes);

    return 0;
}

