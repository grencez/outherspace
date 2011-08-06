
#ifndef Op_0

#define Op_0( T, N, dst, a )  do \
{ \
    uint i_sOp_0; \
    T* dst_sOp_0; \
    const T* a_sOp_0; \
    dst_sOp_0 = dst; \
    a_sOp_0 = a; \
    UFor( i_sOp_0, N ) \
        dst_sOp_0[i_sOp_0] = (a_sOp_0)[i_sOp_0]; \
} while (0)

#define Op_10( T, N, dst, f, a )  do \
{ \
    uint i_sOp_10; \
    T* dst_sOp_10; \
    const T* a_sOp_10; \
    dst_sOp_10 = dst; \
    a_sOp_10 = a; \
    UFor( i_sOp_10, N ) \
        dst_sOp_10[i_sOp_10] = f a_sOp_10[i_sOp_10]; \
} while (0)

#define Op_200( T, N, dst, f, a, b )  do \
{ \
    uint i_sOp_200; \
    T* dst_sOp_200; \
    const T* a_sOp_200; \
    const T* b_sOp_200; \
    dst_sOp_200 = dst; \
    a_sOp_200 = a; \
    b_sOp_200 = b; \
    UFor( i_sOp_200, N ) \
    { \
        dst_sOp_200[i_sOp_200] \
        = (a_sOp_200[i_sOp_200] \
           f \
           b_sOp_200[i_sOp_200]); \
    } \
} while (0)

#define Op_1200( T, N, dst, f, g, a, b )  do \
{ \
    T tmp_sOp_1200[N]; \
    Op_200( T, N, tmp_sOp_1200, g, a, b ); \
    Op_10( T, N, dst, f, tmp_sOp_1200 ); \
} while (0)

#define Op_2010( T, N, dst, f, a, g, b )  do \
{ \
    T tmp_sOp_2010[N]; \
    Op_10( T, N, tmp_sOp_2010, g, b ); \
    Op_200( T, N, dst, f, a, tmp_sOp_2010 ); \
} while (0)

#define Op_2100( T, N, dst, f, g, a, b )  do \
{ \
    T tmp_sOp_2100[N]; \
    Op_10( T, N, tmp_sOp_2100, g, a ); \
    Op_200( T, N, dst, f, tmp_sOp_2100, b ); \
} while (0)

#define Op_21010( T, N, dst, f, g, a, h, b )  do \
{ \
    T tmp_sOp_21010[N]; \
    Op_10( T, N, tmp_sOp_21010, g, a ); \
    Op_2010( T, N, dst, f, tmp_sOp_21010, h, b ); \
} while (0)

#define Op_201200( T, N, dst, f1, a, f2, f3, b, c ) do \
{ \
    T tmp_sOp_201200[N]; \
    Op_1200( T, N, tmp_sOp_201200, f2, f3, b, c ); \
    Op_200( T, N, dst, f1, a, tmp_sOp_201200 ); \
} while (0)

#define Op_202100( T, N, dst, f, a, g, h, b, c ) do \
{ \
    T tmp_sOp_202100[N]; \
    Op_2100( T, N, tmp_sOp_202100, g, h, b, c ); \
    Op_200( T, N, dst, f, a, tmp_sOp_202100 ); \
} while (0)

#define Op_2021010( T, N, dst, f1, a, f2, f3, b, f4, c ) do \
{ \
    T tmp_sOp_2021010[N]; \
    Op_10( T, N, tmp_sOp_2021010, f4, c ); \
    Op_202100( T, N, dst, f1, a, f2, f3, b, tmp_sOp_2021010 ); \
} while (0)


#endif

