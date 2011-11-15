
#include "track.h"

#include "slist.h"
#include "wavefront-file.h"

#include <string.h>

    void
init_Track (Track* track)
{
    track->ncheckplanes = 0;
    track->nstartlocs = 0;
    init_Scene (&track->scene);
}

    void
cleanup_Track (Track* track)
{
    if (track->ncheckplanes > 0)
    {
        free (track->checkplanes);
        free (track->checkpoints);
    }
    if (track->nstartlocs > 0)
    {
        free (track->startlocs);
        free (track->startdirs);
    }
    cleanup_Scene (&track->scene);
}

    bool
readin_Track (Track* track, RaySpace* space,
              const char* pathname, const char* filename)
{
    const uint ndims = 3;
    uint line_no = 0;
    const uint len = BUFSIZ;
    char buf[BUFSIZ];
    bool good = true;
    const char* line;
    FILE* in;
    real scale = 1;

    in = fopen_path (pathname, filename, "rb");
    if (!in)  return false;

    init_Track (track);
    init_RaySpace (space);

    space->nlights = 2;
    space->lights = AllocT( PointLightSource, space->nlights );
    init_PointLightSource (&space->lights[0]);
    init_PointLightSource (&space->lights[1]);
    Op_s( real, NColors, space->lights[0].intensity , .5 );
    Op_s( real, NColors, space->lights[1].intensity , 0 );

    for (line = fgets (buf, len, in);
         good && line;
         line = fgets (buf, len, in))
    {
        uint i;
        line_no += 1;

        strstrip_eol (buf);
        line = strskip_ws (line);

        if (0 == strncmp (line, "scale", 5))
        {
            line = &line[5];
            if (line[0] == ':')  line = &line[1];

            line = strto_real (&scale, line);
            if (!line)
            {
                good = false;
                fprintf (stderr, "Line:%u  Need scale value!\n", line_no);
            }
        }
        else if (0 == strncmp (line, "model:", 6))
        {
            PointXfrm fix;
            line = &line[6];
            readin_wavefront (&track->scene, pathname, line);
            condense_Scene (&track->scene);
            fixup_wavefront_Scene (&track->scene);
            identity_PointXfrm (&fix);
            scale_PointXfrm (&fix, &fix, scale);
            xfrm_Scene (&track->scene, &fix);
        }
        else if (0 == strncmp (line, "sky:", 4))
        {
            Scene* scene;

            line = &line[4];

            scene = &track->scene;
            i = scene->ntxtrs;
            if (i == 0)  scene->txtrs = 0;  /* Assure this is NULL.*/
            scene->ntxtrs = i+1;
            ResizeT( Texture, scene->txtrs, i+1 );
            good = readin_Texture (&scene->txtrs[i], pathname, line);
            if (good)  space->skytxtr = i;

            if (!good)
                fprintf (stderr, "Line:%u  Sky failed!\n", line_no);
        }
        else if (0 == strncmp (line, "checkplanes:", 12))
        {
            good = readin_checkplanes (&track->ncheckplanes,
                                       &track->checkplanes,
                                       &track->checkpoints,
                                       pathname, &line[12]);
            if (!good)
                fprintf (stderr, "Line:%u  Checkplanes failed!\n", line_no);
        }
        else if (0 == strncmp (line, "light", 5))
        {
            line = &line[5];
            if (line[0] == ':')  line = &line[1];

            UFor( i, ndims )
            {
                real* x;
                x = &space->lights[0].location.coords[i];
                line = strto_real (x, line);
                if (!line)
                {
                    good = false;
                    fprintf (stderr, "Line:%u  Not enough coordinates!\n",
                             line_no);
                    break;
                }
            }
        }
    }
    fclose (in);
    if (good)
    {
        copy_Scene (&space->main.scene, &track->scene);
        init_filled_RaySpace (space);
    }
    return good;
}


    bool
readin_checkplanes (uint* ret_nplanes, Plane** ret_planes, Point** ret_points,
                    const char* pathname, const char* filename)
{
    FILE* in;
    bool good = true;
    const uint len = BUFSIZ;
    char buf[BUFSIZ];
    const char* line;
    uint nplanes = 0;
    uint line_no = 1;
    SList planelist, pointlist;

    in = fopen_path (pathname, filename, "rb");
    if (!in)  return false;

    init_SList (&planelist);
    init_SList (&pointlist);

    for (line = fgets (buf, len, in);
         good && line;
         line = fgets (buf, len, in))
    {
        Point loc, normal;
        uint i;

        strstrip_eol (buf);
        line = strskip_ws (line);
        if (line[0] == '#' || line[0] == '\0')  continue;

        zero_Point (&loc);
        UFor( i, 3 )
            if (line)  line = strto_real (&loc.coords[i], line);

        zero_Point (&normal);
        UFor( i, 3 )
            if (line)  line = strto_real (&normal.coords[i], line);

        if (!line)
        {
            good = false;
            fprintf (stderr, "Line:%u  Not enough coordinates, need 6.\n",
                     line_no);
        }
        else
        {
            Plane plane;
            init_Plane (&plane, &normal, &loc);
            app_SList (&planelist, DuplicaT( Plane, &plane, 1 ));
            app_SList (&pointlist, DuplicaT( Point, &loc, 1 ));
            nplanes += 1;
        }
        line_no += 1;
    }
    fclose (in);

    if (!good)
    {
        cleanup_SList (&planelist);
        cleanup_SList (&pointlist);
    }
    else
    {
        *ret_nplanes = nplanes;
        *ret_points = AllocT( Point, nplanes );
        *ret_planes = AllocT( Plane, nplanes );
        unroll_SList (*ret_planes, &planelist, sizeof (Plane));
        unroll_SList (*ret_points, &pointlist, sizeof (Point));
    }

    return good;
}

