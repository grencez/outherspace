
#ifndef Op_Point_10


#define Op_Point_10( dst, f, a )  do \
{ \
    uint i_sOp_Point_10; \
    Point* dst_sOp_Point_10; \
    const Point* a_sOp_Point_10; \
    dst_sOp_Point_10 = dst; \
    a_sOp_Point_10 = a; \
    UFor( i_sOp_Point_10, NDimensions ) \
    { \
        uint i; \
        i = i_sOp_Point_10; \
        dst_sOp_Point_10->coords[i] = f a_sOp_Point_10->coords[i]; \
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
        uint i; \
        i = i_sOp_Point_200; \
        dst_sOp_Point_200->coords[i] = (a_sOp_Point_200->coords[i] \
                                       f \
                                       b_sOp_Point_200->coords[i]); \
    } \
} while (0)

#define Op_Point_1200( dst, f, g, a, b )  do \
{ \
    Point tmp_sOp_Point_1200; \
    Op_Point_200( &tmp_sOp_Point_1200, g, a, b ); \
    Op_Point_10( dst, f, &tmp_sOp_Point_1200 ); \
} while (0)

#define Op_Point_00022( dst, a, b, c, op1, op2 )  do \
{ \
    Point* dst_sOp_Point_00022; \
    dst_sOp_Point_00022 = dst; \
    Op_Point_200( dst_sOp_Point_00022, op1, b, c ); \
    Op_Point_200( dst_sOp_Point_00022, op1, a, dst_sOp_Point_00022 ); \
} while (0)


#define Op_Point_202100( dst, f, a, g, h, b, c ) \
{ \
    Point tmp_sOp_Point_202100; \
    Op_Point_10( &tmp_sOp_Point_202100, h, b ); \
    Op_Point_200( &tmp_sOp_Point_202100, g, &tmp_sOp_Point_202100, c ); \
    Op_Point_200( dst, f, a, &tmp_sOp_Point_202100 ); \
} while (0)


#endif

