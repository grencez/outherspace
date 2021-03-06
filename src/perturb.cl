
#define NDimensions 4
#define UFor( i, bel )  for (i = 0; i < (bel); ++i)


typedef float real;
typedef float3 real3;
typedef float4 real4;

typedef union Point Point;
typedef struct PointXfrm PointXfrm;
typedef PointXfrm Simplex;

union Point
{
    real coords[4];
    real4 m;
    real3 m3;
};
struct PointXfrm { Point pts[NDimensions]; };

__kernel
    void
perturb_kernel (__global real* verts, const uint count)
{
    int i = get_global_id(0);
    verts[3*i+0] += i / 50.0 * (((count + 1*i) % 41) - 20.0) / 20;
    verts[3*i+1] += i / 50.0 * (((count + 2*i) % 41) - 20.0) / 20;
    verts[3*i+2] += i / 50.0 * (((count + 3*i) % 41) - 20.0) / 20;
}

inline Point
xfrm_Point (const PointXfrm A, const Point x)
{
    Point b;
    b.m = (real4)
        (dot (A.pts[0].m, x.m),
         dot (A.pts[1].m, x.m),
         dot (A.pts[2].m, x.m),
         dot (A.pts[3].m, x.m));
    return b;
}

inline void
set_3_reals (__global real* dst, const Point p)
{
    dst[0] = p.coords[0];
    dst[1] = p.coords[1];
    dst[2] = p.coords[2];
}

typedef struct Arg_view_4d_kernel Arg_view_4d_kernel;
struct Arg_view_4d_kernel
{
    PointXfrm xfrm;
    Point xlat;
};

__kernel
    void
view_4d_kernel (__global __write_only real* ret_verts,
                __global __write_only real* ret_vnmls,
                __global __read_only uint* vidcs,
                __global __read_only Point* verts,
                __global __read_only Point* vnmls,
                __constant Arg_view_4d_kernel* arg,
                const uint elem_offset,
                const uint verts_offset,
                const uint vnmls_offset)
{
    const uint PcIdx = get_global_id(0);
    uint i, k = 0;
    uchar inds[4][2];
    Simplex tet;

    ret_verts = &ret_verts[(PcIdx + elem_offset) * 6 * 3];
    ret_vnmls = &ret_vnmls[(PcIdx + elem_offset) * 6 * 3];
    vidcs = &vidcs[(PcIdx + elem_offset) * NDimensions];
    verts = &verts[verts_offset];
    vnmls = &vnmls[vnmls_offset];

    UFor( i, NDimensions )
    {
        const Point p = xfrm_Point (arg->xfrm, verts[vidcs[i]]);
        tet.pts[i].m = p.m + arg->xlat.m;
    }

    UFor( i, NDimensions-1 )
    {
        const real x = tet.pts[i].coords[3];
        uint j;
        for (j = i+1; j < NDimensions; ++j)
        {
            const real y = tet.pts[j].coords[3];
            if ((y <= 0) == (0 < x))
            {
                inds[k][0] = i;
                inds[k][1] = j;
                ++ k;
            }
        }
    }

    UFor( i, NDimensions )
    {
        Point a, b;
        if (k < 3 || i >= k)
        {
            a.m3 = (real3) (0.0, 0.0, 0.0);
            b.m3 = (real3) (1.0, 0.0, 0.0);
        }
        else
        {
            real alpha;
            a = tet.pts[inds[i][0]];
            b = tet.pts[inds[i][1]];
            alpha = fabs (a.coords[3] / (a.coords[3] - b.coords[3]));

            a.m3 = mix (a.m3, b.m3, alpha);

            b.m = mix (vnmls[vidcs[inds[i][0]]].m,
                       vnmls[vidcs[inds[i][1]]].m,
                       alpha);
            b = xfrm_Point (arg->xfrm, b);
            b.m3 = normalize (b.m3);
        }

        set_3_reals (&ret_verts[3*i], a);
        set_3_reals (&ret_vnmls[3*i], b);

        if (i < NDimensions-1)
        {
            if (k == 3)
            {
                a.m3 = (real3) (0.0, 0.0, 0.0);
                b.m3 = (real3) (1.0, 0.0, 0.0);
            }

            set_3_reals (&ret_verts[3*(i+3)], a);
            set_3_reals (&ret_vnmls[3*(i+3)], b);
        }
    }
}

