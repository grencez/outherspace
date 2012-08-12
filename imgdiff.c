
#include "cx/syscx.h"

#include "color.h"
#include "pnm-image.h"
#include "space.h"
#include "util.h"
#include "cx/def.h"

#include <assert.h>
#include <stdio.h>

int main (int argc, char** argv)
{
    int argi =
        (init_sysCx (&argc, &argv),
         1);
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

    lhs_file = argv[argi++];
    rhs_file = argv[argi++];
    dst_file = argv[argi++];

    lhs_bytes = readin_PPM_image (&lhs_nrows, &lhs_ncols, 0, lhs_file);
    rhs_bytes = readin_PPM_image (&rhs_nrows, &rhs_ncols, 0, rhs_file);

    fprintf (stderr, "nrows:%u ncols:%u\n", lhs_nrows, lhs_ncols);
    Claim2( lhs_nrows ,==, rhs_nrows );
    Claim2( lhs_ncols ,==, rhs_ncols );

    n = NColors * lhs_nrows * lhs_ncols;
    { BLoop( i, lhs_nrows )
        { BLoop( j, lhs_ncols )
            Color c0, c1;
            byte* cb0 = &lhs_bytes[(j + i * lhs_ncols) * NColors];
            byte* cb1 = &rhs_bytes[(j + i * lhs_ncols) * NColors];
            Color diff;
            real mag0, mag1;
            { BLoop( k, NColors )
                c0.coords[k] = (1.0 / 255) * cb0[k];
                c1.coords[k] = (1.0 / 255) * cb1[k];
                cb0[k] = 0;
            } BLose()

            diff_Color (&diff, &c1, &c0);

            mag0 = maxmag_Color (&c0);
            mag1 = maxmag_Color (&c1);

            if (mag0 >= mag1)
                cb0[0] = clamp_real (255.5 * (mag0 - mag1), 0, 255.5);
            else
                cb0[1] = clamp_real (255.5 * (mag1 - mag0), 0, 255.5);
        } BLose()
    } BLose()

    output_PPM_image (dst_file, lhs_nrows, lhs_ncols, lhs_bytes);

    free (lhs_bytes);
    free (rhs_bytes);

    lose_sysCx ();
    return 0;
}

