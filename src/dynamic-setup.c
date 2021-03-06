
#include "dynamic-setup.h"

#include "affine.h"
#include "bbox.h"
#include "color.h"
#include "point.h"
#include "serial.h"
#include "simplex.h"
#include "slist.h"
#include "wavefront-file.h"
#include "xfrm.h"
#include "cx/fileb.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static bool
read_racer (Scene* scene, uint idx, const char* pathname);

    bool
interpolate_by_file (Scene* dst,
                     uint nscenes,
                     const char* pathname,
                     const char* const* filenames,
                     const real* dcoords)
{
    bool good = true;
    uint i;
    uint prev_idx = 0;
    Scene* scenes;

    assert (NDimensions ==  4);
    if (NDimensions != 4)  return false;
    if (nscenes == 0)  return false;

    AllocTo( scenes, nscenes );

    UFor( i, nscenes )
    {
        uint j;
        if (!filenames[i])
        {
            assert (i > 0);
            continue;
        }

        good = readin_wavefront (&scenes[i], pathname, filenames[i]);
        if (!good)  return false;
        UFor( j, scenes[i].nverts )
            scenes[i].verts[j].coords[NDimensions-1] = dcoords[i];

        for (j = prev_idx+1; j < i; ++j)
        {
            real alpha;
            alpha =((dcoords[j] - dcoords[prev_idx]) /
                    (dcoords[i] - dcoords[prev_idx]));
            interpolate1_Scene (&scenes[j], alpha,
                                &scenes[prev_idx], &scenes[i]);
        }

        prev_idx = i;
    }

    interpolate_Scene (dst, NDimensions-1, nscenes, scenes);

    UFor( i, nscenes )
        cleanup_Scene (&scenes[i]);
    free (scenes);
    return good;
}

static
    void
interpolate_Track (Track* track, const Point* map_scale)
{
    const real max_drift = 1000;
    real scale;
    uint i;

    assert (NDimensions == 4);
    assert (track->nmorphs > 1);

    scale = max_drift / track->morph_dcoords[track->nmorphs-1];

    UFor( i, track->nmorphs )
    {
        uint j;
        real dcoord;
        Scene* scene;

        scene = &track->morph_scenes[i];

        track->morph_dcoords[i] *= scale;
        dcoord = track->morph_dcoords[i] / map_scale->coords[i];

        UFor( j, scene->nverts )
            scene->verts[j].coords[NDimensions-1] = dcoord;
    }

    interpolate_Scene (&track->scene, NDimensions-1,
                       track->nmorphs, track->morph_scenes);
}

    bool
add_racers (RaySpace* space, uint nracers, const Track* track,
            const char* pathname)
{
    uint i, off;
    bool good = true;

    off = space->nobjects;
    if (off == 0)  space->objects = 0;
    space->nobjects += nracers;
    ResizeT( ObjectRaySpace, space->objects, space->nobjects );

    for (i = 0; i < space->nobjects; ++i)
    {
        uint q;
        real back;
        Point loc, dir;
        ObjectRaySpace* object;

        object = &space->objects[i+off];
        init_ObjectRaySpace (object);

        good = read_racer (&object->scene, 0, pathname);
        if (!good)  return false;
        init_filled_ObjectRaySpace (object);

        if (track->startlocs.sz == 0)
        {
            q = i;
            zero_Point (&loc);
            zero_Point (&dir);
            dir.coords[ForwardDim] = 1;
        }
        else
        {
            uint r;
            q = i / track->startlocs.sz;
            r = i % track->startlocs.sz;
            loc = track->startlocs.s[r].origin;
            dir = track->startlocs.s[r].direct;
        }

            /* This will be the amount a racer should move backwards.
             * It is a negative value.
             */
        back = 2 * q * (+ object->box.min.coords[ForwardDim]
                        - object->box.max.coords[ForwardDim]);
        follow_Point (&object->centroid, &loc, &dir, - back);

            /* Orientation starts as identity,
             * simply turn the craft's basis to fit.
             */
        stable_orthorotate_PointXfrm (&object->orientation,
                                      &object->orientation,
                                      &dir, ForwardDim);
    }
    return good;
}

static
  bool
readin_wavefront_Track (Track* track, RaySpace* space,
                        const char* pathname,
                        const char* filename)
{
  bool good;
  init_Track (track);
  init_RaySpace (space);

  space->nlights = 1;
  AllocTo( space->lights, space->nlights );
  init_PointLightSource (&space->lights[0]);

  {
    XFileB xfb[] = default;
    const char* fname = strrchr (filename, '/');
    if (fname) {
      fname = &fname[1];
    }
    else {
      fname = filename;
    }
    if (!open_FileB (&xfb->fb, pathname, filename))
      BailOut( false, "couldn't read file" );
    good = readin_wavefront (&track->scene, xfb->fb.pathname.s, fname);
    lose_XFileB (xfb);
  }
  if (!good)
    BailOut( false, "it's just no good!");

  copy_Scene (&space->main.scene, &track->scene);
  init_filled_RaySpace (space);
  {
    IAMap map;
    identity_IAMap (&map);
    zero_PointXfrm (&map.xfrm);
    map_Scene (&track->scene, &map);
    parse_coord_system (&map.xfrm, "right up back");
  }
  return good;
}

    bool
readin_Track (Track* track, RaySpace* space,
              const char* pathname, const char* filename)
{
  const uint ndims = 3;
  uint line_no = 0;
  bool good = true;
  const char* line;
  XFileB xfb[] = default;
  XFile* xf = &xfb->xf;
  FILE* out = stderr;
  PointXfrm coord_system;
  IAMap model_map;  /* Use to save model transformation.*/
  IAMap map[1];
  Point scale;
  Point location;
  Scene* scene = 0;
  Texture* skytex = 0;
  uint i;

  if (sfxeq_cstr (filename, ".obj")) {
    return readin_wavefront_Track (track, space, pathname, filename);
  }

  if (!open_FileB (&xfb->fb, pathname, filename))
    return false;

  identity_PointXfrm (&coord_system);
  identity_IAMap (map);
  model_map = *map;
  zero_Point (&location);
  set_Point (&scale, 1);

  init_Track (track);
  init_RaySpace (space);

  space->nlights = 2;
  AllocTo( space->lights, space->nlights );
  init_PointLightSource (&space->lights[0]);
  init_PointLightSource (&space->lights[1]);
  /* Op_s( real, NColors, space->lights[0].intensity , .5 ); */
  space->lights[1].on = false;

  for (line = getline_XFile (xf);
       good && line;
       line = getline_XFile (xf))
  {
    XFile olay[1];
    bool recalc_map = false;
    line_no += 1;
    olay_txt_XFile (olay, xf, IdxEltTable( xf->buf, line ));

    skipds_XFile (olay, NULL);
    line = cstr_XFile (olay);

    if (AccepTok( line, "scale" ))
    {
      if (line[0] == ':')  line = &line[1];

      set_Point (&scale, 1);
      for (i = 0; i < 3; ++i)
      {
        if (line)
          line = strto_real (&scale.coords[i], line);
        if (!line)
        {
          good = (i > 0);
          if (!good)  break;
          scale.coords[i] = scale.coords[i-1];
        }
      }

      if (good)
      {
        xfrm_Point (&scale, &coord_system, &scale);
        recalc_map = true;
      }
      else
      {
        fprintf (out, "Line:%u  Need scale value!\n", line_no);
      }
    }
    else if (AccepTok( line, "loc" ))
    {
      if (line[0] == ':')  line = &line[1];

      UFor( i, 3 )
        if (line)  line = strto_real (&location.coords[i], line);

      good = !!(line);
      if (good)
      {
        xfrm_Point (&location, &coord_system, &location);
        recalc_map = true;
      }
      else
      {
        fprintf (out, "Line:%u  Need 3 location values!\n", line_no);
      }
    }
    else if (AccepTok( line, "camloc" ))
    {
      if (line[0] == ':')  line = &line[1];
      offto_XFile (olay, line);
      xget_IAMap (olay, &track->camloc);
    }
    else if (AccepTok( line, "model:" ))
    {
      bool doit = true;
      model_map = *map;
      if (NDimensions == 4)
      {
        real dcoord = 0;
        if (track->nmorphs > 0)
          dcoord = track->morph_dcoords[track->nmorphs-1] + 1;

        ++ track->nmorphs;
        ResizeT( real, track->morph_dcoords, track->nmorphs );
        ResizeT( Scene, track->morph_scenes, track->nmorphs );
        track->morph_dcoords[track->nmorphs-1] = dcoord;
        scene = &track->morph_scenes[track->nmorphs-1];
      }
      else if (!scene)
      {
        scene = &track->scene;
      }
      else
      {
        /* Ignore because we are not in 4d mode.*/
        doit = false;
      }
      if (doit)  good = readin_wavefront (scene, xfb->fb.pathname.s, line);
    }
    else if (AccepTok( line, "concat-model:" ))
    {
      Scene tmp;
      good = !!(scene);
      if (good)  good = readin_wavefront (&tmp, xfb->fb.pathname.s, line);
      else       fprintf (out, "Line:%u  No scene for concat!\n",
                          line_no);
      /* TODO: Some sort of scene mapping should happen?*/
      if (good)  concat0_Scene (scene, &tmp);
    }
    else if (AccepTok( line, "sky:" ))
    {
      AllocTo( skytex, 1 );
      good = readin_Texture (skytex, xfb->fb.pathname.s, line);
      if (!good)
        fprintf (out, "Line:%u  Sky failed!\n", line_no);
    }
    else if (AccepTok( line, "checkplanes:" ))
    {
      good = readin_checkplanes (&track->ncheckplanes,
                                 &track->checkplanes,
                                 &track->checkpoints,
                                 map, xfb->fb.pathname.s, line);
      if (!good)
        fprintf (out, "Line:%u  Checkplanes failed!\n", line_no);
    }
    else if (AccepTok( line, "light" ))
    {
      Point* loc;

      if (line[0] == ':')  line = &line[1];
      loc = &space->lights[0].location;

      zero_Point (loc);
      UFor( i, ndims )
        if (line)  line = strto_real (&loc->coords[i], line);

      good = !!(line);
      if (good)
        map_Point (loc, map, loc);
      else
        fprintf (out, "Line:%u  Not enough coordinates for light!\n",
                 line_no);
    }
    else if (AccepTok( line, "start" ))
    {
      Point loc, dir;
      if (line[0] == ':')  line = &line[1];

      zero_Point (&loc);
      UFor( i, ndims )
        if (line)  line = strto_real (&loc.coords[i], line);

      zero_Point (&dir);
      UFor( i, ndims )
        if (line)  line = strto_real (&dir.coords[i], line);

      good = !!(line);
      if (good)
      {
        DeclGrow1Table( Ray, ray, track->startlocs );
        map_Point (&loc, map, &loc);
        map_Point (&dir, map, &dir);
        ray->origin = loc;
        ray->direct = dir;
      }
      else
      {
        fprintf (out, "Line:%u  Not enough coordinates for start!\n",
                 line_no);
      }
    }
    else if (AccepTok( line, "coords" ))
    {
      if (line[0] == ':')  line = &line[1];
      good = parse_coord_system (&coord_system, line);
      if (good)  recalc_map = true;
    }
    else if (AccepTok( line, "vpls" ))
    {
      if (line[0] == ':')  line = &line[1];
      line = strto_uint (&track->nphotons, line);
      if (good && (good = !!line))
        line = strto_uint (&track->nbounces, line);
      if (good)
        good = !!line;
    }
    else if (AccepTok( line, "img_dims" ))
    {
      if (line[0] == ':')  line = &line[1];
      line = strto_uint (&track->nimgrows, line);
      if (good && (good = !!line))
        line = strto_uint (&track->nimgcols, line);
      if (good)
        good = !!line;
    }

    if (recalc_map)
    {
      identity_IAMap (map);
      xlat0_IAMap (map, &location);
      xfrm0_IAMap (map, &coord_system);
      prod0_IAMap (map, &scale);
    }
  }

  lose_XFileB (xfb);

  if (good && !scene)
  {
    good = false;
    fputs ("No model provided for track!\n", out);
  }

  if (good)
  {
    if (track->nmorphs > 0)
    {
      uint i;
      interpolate_Track (track, &model_map.scale);
      scene = &track->scene;
      UFor( i, track->nmorphs )
        map_Scene (&track->morph_scenes[i], &model_map);
    }
    condense_Scene (scene);
    map_Scene (scene, &model_map);

    space->skytxtr = scene->ntxtrs;
    if (skytex)
    {
      if (scene->ntxtrs == 0)  scene->txtrs = 0;  /* Assure NULL.*/
      ConcaT( Texture, scene->txtrs, skytex, scene->ntxtrs, 1 );
      free (skytex);
    }

    copy_Scene (&space->main.scene, scene);
    init_filled_RaySpace (space);
  }
  return good;
}


    bool
readin_checkplanes (uint* ret_nplanes, Plane** ret_planes, Point** ret_points,
                    const IAMap* map,
                    const char* pathname, const char* filename)
{
    FILE* in;
    bool good = true;
    const uint len = BUFSIZ;
    char buf[BUFSIZ];
    const char* line;
    uint nplanes = 0;
    uint line_no = 1;
    SList planelist = default;
    SList pointlist = default;

    in = fopen_path (pathname, filename, "rb");
    if (!in)  return false;

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
            map_Point (&loc, map, &loc);
            mapvec_Point (&normal, map, &normal);
            init_Plane (&plane, &normal, &loc);
            app_SList (&planelist, DupliT( Plane, &plane, 1 ));
            app_SList (&pointlist, DupliT( Point, &loc, 1 ));
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
        AllocTo( *ret_points, nplanes );
        AllocTo( *ret_planes, nplanes );
        unroll_SList (*ret_planes, &planelist, sizeof (Plane));
        unroll_SList (*ret_points, &pointlist, sizeof (Point));
    }

    return good;
}


    bool
parse_coord_system (PointXfrm* a, const char* line)
{
    bool cover[NDimensions];
    uint perms[NDimensions];
    bool good = true;
    uint i;

    UFor( i, NDimensions )  cover[i] = false;
    UFor( i, NDimensions )  perms[i] = 2*i;

    for (i = 0; i < 3 && good; ++i)
    {
        uint d = UINT_MAX;

        line = strskip_ws (line);
        good = !!line;
        if (!good)  break;

        if      (AccepTok( line, "up" ))     d = 2 * UpDim;
        else if (AccepTok( line, "down" ))   d = 2 * UpDim + 1;
        else if (AccepTok( line, "right" ))  d = 2 * RtDim;
        else if (AccepTok( line, "left" ))   d = 2 * RtDim + 1;
        else if (AccepTok( line, "for" ))    d = 2 * FwDim;
        else if (AccepTok( line, "fwd" ))    d = 2 * FwDim;
        else if (AccepTok( line, "back" ))   d = 2 * FwDim + 1;

        good = (d < UINT_MAX);
        if (good)
        {
            cover[d/2] = true;
            perms[d/2] = 2 * i + (d % 2);
        }
    }

    if (good)
    {
        assert (cover[UpDim]);
        assert (cover[RtDim]);
        assert (cover[FwDim]);
        permutation_PointXfrm (a, perms);
    }

    return good;
}

    bool
read_racer (Scene* scene, uint idx, const char* pathname)
{
    bool good;
    char fname[20];

    sprintf (fname, "machine%u.obj", idx);

    if (NDimensions == 3)
    {
        good = readin_wavefront (scene, pathname, fname);
    }
    else
    {
        const char* fnames[2];
        const real dcoords[2] = { 0, 1.0/16 };
        fnames[0] = fname;
        fnames[1] = fname;
        good = interpolate_by_file (scene, 2, pathname, fnames, dcoords);
    }
    if (!good)  return false;

    condense_Scene (scene);

    {
        IAMap map;
        BBox box;
        Point meas;
        real vol, a;

        init_BBox (&box, scene->nverts, scene->verts);
        measure_BBox (&meas, &box);
        vol = (meas.coords[UpDim] *
               meas.coords[RightDim] *
               meas.coords[ForwardDim]);
        a = pow (30000 / vol, 1.0 / 3);

        identity_IAMap (&map);
        parse_coord_system (&map.xfrm, "right up back");
        recenter_Scene (&map, scene, 0);
        scale0_IAMap (&map, a);
        map_Scene (scene, &map);
    }

    return good;
}

    /** Setup lights at the corners of the bounding box.
     * WARNING: This was used (at some point) in a testcase which was removed.
     *          It's neat code, but I'm not sure how well it works!
     **/
    void
setup_box_lights (RaySpace* space,
                  const PointLightSource* light,
                  const BBox* box)
{
    PointLightSource dflt_light;
    BBox dflt_box;
    uint i, ndims = 0;
    uint dims[NDimensions];

    if (!light)
    {
        init_PointLightSource (&dflt_light);
        quot1_Color (&dflt_light.intensity, &dflt_light.intensity,
                     exp2_uint (NDimensions-1));
        dflt_light.diffuse = true;
        light = &dflt_light;
    }

    if (!box)
    {
        dflt_box = space->main.box;
        dflt_box.min.coords[UpDim] = dflt_box.max.coords[UpDim];
        dflt_box.min.coords[UpDim] += 1000;
        dflt_box.max.coords[UpDim] += 2000;
        box = &dflt_box;
    }

    UFor( i, NDimensions )
    {
        if (box->min.coords[i] != box->max.coords[i])
        {
            dims[ndims] = i;
            ndims += 1;
        }
    }

    space->nlights = exp2_uint (ndims);
    AllocTo( space->lights, space->nlights );

    UFor( i, space->nlights )
    {
        uint di, flags;
        Point* loc;

        flags = i;
        loc = &space->lights[i].location;

        copy_PointLightSource (&space->lights[i], light);
        copy_Point (loc, &box->max);

        UFor( di, ndims )
        {
            uint dim;
            dim = dims[di];
            if (even_uint (flags))
                loc->coords[dim] = box->min.coords[dim];
            else
                loc->coords[dim] = box->max.coords[dim];
            flags >>= 1;
        }
    }
}

