
#ifndef Op_Point_10


#define Op_Point_0( dst, a )  do \
{ \
    uint i_sOp_Point_0; \
    Point* dst_sOp_Point_0; \
    const Point* a_sOp_Point_0; \
    dst_sOp_Point_0 = dst; \
    a_sOp_Point_0 = a; \
    UFor( i_sOp_Point_0, NDimensions ) \
    { \
        dst_sOp_Point_0->coords[i_sOp_Point_0] \
        = a_sOp_Point_0->coords[i_sOp_Point_0]; \
    } \
} while (0)

#define Op_Point_10( dst, f, a )  do \
{ \
    uint i_sOp_Point_10; \
    Point* dst_sOp_Point_10; \
    const Point* a_sOp_Point_10; \
    dst_sOp_Point_10 = dst; \
    a_sOp_Point_10 = a; \
    UFor( i_sOp_Point_10, NDimensions ) \
    { \
        dst_sOp_Point_10->coords[i_sOp_Point_10] \
        = f a_sOp_Point_10->coords[i_sOp_Point_10]; \
    } \
} while (0)

#define Op_Point_200( dst, f, a, b )  do \
{ \
    uint i_sOp_Point_200; \
    Point* dst_sOp_Point_200; \
    const Point* a_sOp_Point_200; \
    const Point* b_sOp_Point_200; \
    dst_sOp_Point_200 = dst; \
    a_sOp_Point_200 = a; \
    b_sOp_Point_200 = b; \
    UFor( i_sOp_Point_200, NDimensions ) \
    { \
        dst_sOp_Point_200->coords[i_sOp_Point_200] \
        = (a_sOp_Point_200->coords[i_sOp_Point_200] \
           f \
           b_sOp_Point_200->coords[i_sOp_Point_200]); \
    } \
} while (0)

#define Op_Point_1200( dst, f, g, a, b )  do \
{ \
    Point tmp_sOp_Point_1200; \
    Op_Point_200( &tmp_sOp_Point_1200, g, a, b ); \
    Op_Point_10( dst, f, &tmp_sOp_Point_1200 ); \
} while (0)

#define Op_Point_2010( dst, f, a, g, b )  do \
{ \
    Point tmp_sOp_Point_2010; \
    Op_Point_10( &tmp_sOp_Point_2010, g, b ); \
    Op_Point_200( dst, f, a, &tmp_sOp_Point_2010 ); \
} while (0)

#define Op_Point_2100( dst, f, g, a, b )  do \
{ \
    Point tmp_sOp_Point_2100; \
    Op_Point_10( &tmp_sOp_Point_2100, g, a ); \
    Op_Point_200( dst, f, &tmp_sOp_Point_2100, b ); \
} while (0)

#define Op_Point_21010( dst, f, g, a, h, b )  do \
{ \
    Point tmp_sOp_Point_21010; \
    Op_Point_10( &tmp_sOp_Point_21010, g, a ); \
    Op_Point_2010( dst, f, &tmp_sOp_Point_21010, h, b ); \
} while (0)

#define Op_Point_202100( dst, f, a, g, h, b, c ) \
{ \
    Point tmp_sOp_Point_202100; \
    Op_Point_2100( &tmp_sOp_Point_202100, g, h, b, c ); \
    Op_Point_200( dst, f, a, &tmp_sOp_Point_202100 ); \
} while (0)

#define Op_Point_2021010( dst, f1, a, f2, f3, b, f4, c ) \
{ \
    Point tmp_sOp_Point_2021010; \
    Op_Point_10( &tmp_sOp_Point_2021010, f4, c ); \
    Op_Point_202100( dst, f1, a, f2, f3, b, &tmp_sOp_Point_2021010 ); \
} while (0)


#endif

