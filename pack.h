
#ifndef Op_0

#define Op_0( T, N, dst, u )  do \
{ \
    uint i_sOp_0; \
    T* dst_sOp_0; \
    const T* u_sOp_0; \
    dst_sOp_0 = dst; \
    u_sOp_0 = u; \
    UFor( i_sOp_0, N ) \
        dst_sOp_0[i_sOp_0] = (u_sOp_0)[i_sOp_0]; \
} while (0)

#define Op_s( T, N, dst, a )  do \
{ \
    uint i_sOp_s; \
    T* dst_sOp_s; \
    T a_sOp_s; \
    dst_sOp_s = dst; \
    a_sOp_s = a; \
    UFor( i_sOp_s, N )  dst_sOp_s[i_sOp_s] = a_sOp_s; \
} while (0)

#define Op_10( T, N, dst, f, u )  do \
{ \
    uint i_sOp_10; \
    T* dst_sOp_10; \
    const T* u_sOp_10; \
    dst_sOp_10 = dst; \
    u_sOp_10 = u; \
    UFor( i_sOp_10, N ) \
        dst_sOp_10[i_sOp_10] = f u_sOp_10[i_sOp_10]; \
} while (0)

#define Op_200( T, N, dst, f, u, v )  do \
{ \
    uint i_sOp_200; \
    T* dst_sOp_200; \
    const T* u_sOp_200; \
    const T* v_sOp_200; \
    dst_sOp_200 = dst; \
    u_sOp_200 = u; \
    v_sOp_200 = v; \
    UFor( i_sOp_200, N ) \
    { \
        dst_sOp_200[i_sOp_200] \
        = (u_sOp_200[i_sOp_200] \
           f \
           v_sOp_200[i_sOp_200]); \
    } \
} while (0)

#define Op_20s( T, N, dst, f, u, a )  do \
{ \
    uint i_sOp_20s; \
    T* dst_sOp_20s; \
    const T* u_sOp_20s; \
    T a_sOp_20s; \
    dst_sOp_20s = dst; \
    u_sOp_20s = u; \
    a_sOp_20s = a; \
    UFor( i_sOp_20s, N ) \
    { \
        dst_sOp_20s[i_sOp_20s] \
        = (u_sOp_20s[i_sOp_20s] f a_sOp_20s); \
    } \
} while (0)

#define Op_1200( T, N, dst, f, g, u, v )  do \
{ \
    T tmp_sOp_1200[N]; \
    Op_200( T, N, tmp_sOp_1200, g, u, v ); \
    Op_10( T, N, dst, f, tmp_sOp_1200 ); \
} while (0)

#define Op_2010( T, N, dst, f, u, g, v )  do \
{ \
    T tmp_sOp_2010[N]; \
    Op_10( T, N, tmp_sOp_2010, g, v ); \
    Op_200( T, N, dst, f, u, tmp_sOp_2010 ); \
} while (0)

#define Op_2100( T, N, dst, f, g, u, v )  do \
{ \
    T tmp_sOp_2100[N]; \
    Op_10( T, N, tmp_sOp_2100, g, u ); \
    Op_200( T, N, dst, f, tmp_sOp_2100, v ); \
} while (0)

#define Op_21010( T, N, dst, f, g, u, h, v )  do \
{ \
    T tmp_sOp_21010[N]; \
    Op_10( T, N, tmp_sOp_21010, g, u ); \
    Op_2010( T, N, dst, f, tmp_sOp_21010, h, v ); \
} while (0)

#define Op_201200( T, N, dst, f1, u, f2, f3, v, w ) do \
{ \
    T tmp_sOp_201200[N]; \
    Op_1200( T, N, tmp_sOp_201200, f2, f3, v, w ); \
    Op_200( T, N, dst, f1, u, tmp_sOp_201200 ); \
} while (0)

#define Op_202100( T, N, dst, f, u, g, h, v, w ) do \
{ \
    T tmp_sOp_202100[N]; \
    Op_2100( T, N, tmp_sOp_202100, g, h, v, w ); \
    Op_200( T, N, dst, f, u, tmp_sOp_202100 ); \
} while (0)

#define Op_2021010( T, N, dst, f1, u, f2, f3, v, f4, w ) do \
{ \
    T tmp_sOp_2021010[N]; \
    Op_10( T, N, tmp_sOp_2021010, f4, w ); \
    Op_202100( T, N, dst, f1, u, f2, f3, v, tmp_sOp_2021010 ); \
} while (0)


#endif

