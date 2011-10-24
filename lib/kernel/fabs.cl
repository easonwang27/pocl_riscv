/* OpenCL built-in library: fabs()

   Copyright (c) 2011 Universidad Rey Juan Carlos
   
   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:
   
   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.
   
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/

#undef fabs

// Import Intel/AMD vector instructions
#ifdef __SSE__
#  define extern
#  define static
#  include <xmmintrin.h>
#endif

#ifdef __SSE2__
#  define extern
#  define static
#  include <emmintrin.h>
#endif

#ifdef __AVX__
#  define extern
#  define static
#  include <immintrin.h>
#endif

float fabsf(float a);
double fabs(double a);



float __attribute__ ((overloadable))
cl_fabs(float a)
{
  return fabsf(a);
}

float2 __attribute__ ((overloadable))
cl_fabs(float2 a)
{
#ifdef __SSE__
  return ((float4)cl_fabs((float4)(a, 0.0f, 0.0f))).s01;
#else
  return (float2)(cl_fabs(a.s0), cl_fabs(a.s1));
#endif
}

float3 __attribute__ ((overloadable))
cl_fabs(float3 a)
{
#ifdef __SSE__
  return ((float4)cl_fabs((float4)(a, 0.0f))).s012;
#else
  return (float3)(cl_fabs(a.s01), cl_fabs(a.s2));
#endif
}

float4 __attribute__ ((overloadable))
cl_fabs(float4 a)
{
#ifdef __SSE__
  const float4 sign_mask =
    as_float4((uint4)(0x80000000U, 0x80000000U, 0x80000000U, 0x80000000U));
  return _mm_andnot_ps(sign_mask, a);
#else
  return (float4)(cl_fabs(a.s01), cl_fabs(a.s23));
#endif
}

float8 __attribute__ ((overloadable))
cl_fabs(float8 a)
{
#ifdef __AVX__
  const float8 sign_mask =
    as_float8((uint8)(0x80000000U, 0x80000000U, 0x80000000U, 0x80000000U,
                      0x80000000U, 0x80000000U, 0x80000000U, 0x80000000U));
  return _mm256_andnot_ps(sign_mask, a);
#else
  return (float8)(cl_fabs(a.s0123), cl_fabs(a.s4567));
#endif
}

float16 __attribute__ ((overloadable))
cl_fabs(float16 a)
{
  return (float16)(cl_fabs(a.s01234567), cl_fabs(a.s89abcdef));
}

double __attribute__ ((overloadable))
cl_fabs(double a)
{
  return fabs(a);
}

double2 __attribute__ ((overloadable))
cl_fabs(double2 a)
{
#ifdef __SSE2__
  const double2 sign_mask =
    as_double2((ulong2)(0x8000000000000000UL, 0x8000000000000000UL));
  return _mm_andnot_pd(sign_mask, a);
#else
  return (double2)(cl_fabs(a.s0), cl_fabs(a.s1));
#endif
}

double3 __attribute__ ((overloadable))
cl_fabs(double3 a)
{
#ifdef __AVX__
  return ((double4)cl_fabs((double4)(a, 0.0))).s012;
#else
  return (double3)(cl_fabs(a.s01), cl_fabs(a.s2));
#endif
}

double4 __attribute__ ((overloadable))
cl_fabs(double4 a)
{
#ifdef __AVX__
  const double4 sign_mask =
    as_double4((ulong4)(0x8000000000000000UL, 0x8000000000000000UL,
                        0x8000000000000000UL, 0x8000000000000000UL));
  return _mm256_andnot_pd(sign_mask, a);
#else
  return (double4)(cl_fabs(a.s01), cl_fabs(a.s23));
#endif
}

double8 __attribute__ ((overloadable))
cl_fabs(double8 a)
{
  return (double8)(cl_fabs(a.s0123), cl_fabs(a.s4567));
}

double16 __attribute__ ((overloadable))
cl_fabs(double16 a)
{
  return (double16)(cl_fabs(a.s01234567), cl_fabs(a.s89abcdef));
}
