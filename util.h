
#ifndef UTIL_H_
#ifndef __OPENCL_VERSION__
#define UTIL_H_
#include "cx/def.h"

#include <stdlib.h>
#include <stdio.h>

#define Stringify(a) #a
#define StringifyPath(a,b) Stringify(a/b)

#ifndef EmbedPathnamePfx
#define EmbedPathnamePfx .
#endif
#ifdef EmbedFilenamePfx
#define EmbedInclude(s) \
    StringifyPath(EmbedPathnamePfx,ConcatifyDef(EmbedFilenamePfx,s.embed.h))
#else  /* ^^^ defined(EmbedFilenamePfx) */
#define EmbedInclude(s) \
    StringifyPath(EmbedPathnamePfx,s.embed.h)
#endif  /* !defined(EmbedFilenamePfx) */


#define ResizeT( Type, arr, capacity ) \
    ((arr) = (Type*) ((arr) \
                      ? realloc (arr, (capacity) * sizeof (Type)) \
                      : AllocT( Type, capacity )))

#define CopyT( Type, dst, src, lo, count ) \
    (array_cpy (dst, src, lo, count, sizeof (Type)))

#define ZeroT( Type, a, count ) \
    (memset (a, 0, (count) * sizeof (Type)))

#define ConcaT( Type, dst, src, end, count ) \
    ((dst) = (Type*) array_cat (dst, src, &end, count, sizeof (Type)))
        

#ifndef uint32
#define uint32 uint
#endif


#define __global
#else  /* ^^^ !defined(__OPENCL_VERSION__) */
#define assert (void)
#define static
#endif  /* defined(__OPENCL_VERSION__) */

    /* Does not scope /j/.*/
#define UUFor( i, ibel, j, jbel )  UFor( i, ibel )  UFor( j, jbel )


typedef int tristate;

    /* Note: All functions below are ideally inline'd!*/
bool even_uint (uint a);
uint
ceil_uint (uint a, uint b);
uint
log2_uint (uint a);
uint
exp2_uint (uint p);
uint
incmod_uint (uint a, uint b, uint m);
uint
decmod_uint (uint a, uint b, uint m);
uint
assoc_index (uint n, uint a, uint b);
void
swap_uint (uint* x, uint* y);
void
swap_real (real* x, real* y);
tristate compare_real (real a, real b);
uint
max_uint (uint a, uint b);
uint
min_uint (uint a, uint b);
real match_real (real a, real b);
real
clamp_real (real x, real lo, real hi);
real
abs_real (real x);
real
atan2_real (real y, real x);
real absolute_error (real expect, real result);
real relative_error (real expect, real result, real large);
bool
approx_eql (real expect, real result, real large, real mul);
tristate signum_real (real a);
tristate mul_signum (tristate a, tristate b);

#ifdef NDEBUG
#define AssertApprox( expect, result, large, mul )
#define AssertStatus( stat, msg )
#else
#define AssertApprox( expect, result, large, mul ) \
    assert (approx_eql (expect, result, large, mul))
#define AssertStatus( stat, msg ) \
    assert_status (stat, msg, __FILE__, __LINE__)
#endif

#define AssertEq( a, b )  Claim2( a ,==, b )
#define AssertEqA( N, expect, result, off_expect, off_result )  do \
{ \
    uint AssertEqA_i; \
    uint AssertEqA_off_expect; \
    uint AssertEqA_off_result; \
    AssertEqA_off_expect = off_expect; \
    AssertEqA_off_result = off_result; \
    UFor( AssertEqA_i, N ) \
    { \
        AssertEq( (expect)[AssertEqA_i + AssertEqA_off_expect], \
                  (result)[AssertEqA_i + AssertEqA_off_result] ); \
    } \
} while (0)

#ifndef __OPENCL_VERSION__
uint index_of (const void* e, const void* arr, size_t size);
void array_set (void* arr, uint i, const void* e, size_t size);
void
array_cpy (void* dst, const void* src, uint lo, uint count, size_t size);
void*
array_dup (const void* src, uint count, size_t size);
void*
array_cat (void* dst, const void* src, uint* end, uint count, size_t size);
char* strto_uint (uint* ret, const char* in);
char* strto_real (real* ret, const char* in);
uint strcount_ws (const char* s);
uint strcount_non_ws (const char* s);
const char* strskip_ws (const char* line);
void strstrip_eol (char* line);
uint readin_whitesep (char* buf, FILE* in, uint capacity, uint len);
char*
cat_strings (uint n, const char* const* a);
char*
cat_filepath (const char* pathname, const char* filename);
bool
strends_with (const char* str, const char* sfx);
FILE*
fopen_path (const char* pathname, const char* filename, const char* mode);
bool
readin_files (uint nfiles, uint* files_nbytes, byte** files_bytes,
              const char* pathname,
              const char* const* files);
real monotime ();
void assert_status (int stat, const char* msg, const char* file, int line);
#ifdef IncludeC
#include "util.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

