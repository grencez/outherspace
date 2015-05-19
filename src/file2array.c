
#include "cx/syscx.h"
#include <stdlib.h>

int main (int argc, char** argv)
{
    int argi =
        (init_sysCx (&argc, &argv),
         1);
    const char nfilessym[] = "nfiles";
    const char arrsym[] = "files_bytes";
    const char lensym[] = "files_nbytes";
    const char arrtype[] = "unsigned char";
    const char lentype[] = "unsigned int";
    FILE* out = stdout;
    FILE* err = stderr;
    uint i, nfiles;
    uint* files_nbytes;

    if (argi < argc)
    {
        out = fopen (argv[argi], "wb");
        if (!out)
        {
            fprintf (err, "%s: Cannot write:%s\n", argv[0], argv[argi]);
            return 1;
        }
    }
    else
    {
        fprintf (err, "Usage: %s out.h in1 [... inN]\n", argv[0]);
        return 1;
    }
    ++ argi;

    nfiles = (argc - argi);
    files_nbytes = AllocT( uint, nfiles );
    fprintf (out, "static const %s %s=%u;\n", lentype, nfilessym, nfiles);

    UFor( i, nfiles )
    {
        FILE* in;
        byte buf[BUFSIZ];
        const uint buflen = BUFSIZ;
        uint n;

        in = fopen (argv[argi + i], "rb");
        if (!in)
        {
            fprintf (err, "%s: Cannot read:%s\n", argv[0], argv[argi + i]);
            return 1;
        }

        fprintf (out, "static const %s %s_%u[]={", arrtype, arrsym, i);
        files_nbytes[i] = 0;
        for (n = fread (buf, 1, buflen, in);
             n > 0;
             n = fread (buf, 1, buflen, in))
        {
            uint j;
            files_nbytes[i] += n;
            UFor( j, n )  fprintf (out, "%u,", buf[j]);
        }

        fputs ("0};\n", out);

        fclose (in);
    }

    fprintf (out, "static const %s %s[]={", lentype, lensym);
    UFor( i, nfiles )
    {
        if (i > 0)  fputc (',', out);
        fprintf (out, "%u", files_nbytes[i]);
    }
    fputs ("};", out);

    fputc ('\n', out);

    fprintf (out, "static const %s* const %s[]={", arrtype, arrsym);
    UFor( i, nfiles )
    {
        if (i > 0)  fputc (',', out);
        fprintf (out, "%s_%u", arrsym, i);
    }
    fputs ("};\n", out);

    free (files_nbytes);
    lose_sysCx ();
    return 0;
}

