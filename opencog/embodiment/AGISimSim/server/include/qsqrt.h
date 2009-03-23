/*
    Copyright (C) 2000 by Andrew Zabolotny (Intel version)
    Copyright (C) 2002 by Matthew Reda <reda@mac.com> (PowerPC version)
    Fast computation of sqrt(x) and 1/sqrt(x)
  
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Library General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.
  
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Library General Public License for more details.
  
    You should have received a copy of the GNU Library General Public
    License along with this library; if not, write to the Free
    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/**\file
 * Fast computation of sqrt(x) and 1/sqrt(x).<p>
 * Define CS_NO_QSQRT if you experience mysterious problems with CS which 
 * you think are related to this version of sqrt not behaving properly. If 
 * you find something like that I'd like to be notified of this so we can 
 * make sure this really is the problem.
 */

#ifndef __CS_QSQRT_H__
#define __CS_QSQRT_H__

/**
 * \addtogroup floating_point
 * @{ */

/**
 * This routine computes sqrt(x) very quickly on Intel and PowerPC platforms.
 */
static inline float qsqrt (float x);

/**
 * This routine is basically equivalent to qsqrt() except that it returns
 * 1/sqrt(x) rather than the proper square root. It should be used anywhere
 * you need the inverse root (in 3D graphics it is a common situation),
 * because the routine is a little faster than qsqrt() and also you avoid
 * a division.
 */
static inline float qisqrt (float x);

/** @} */

#if (!defined (CS_NO_QSQRT)) && defined (PROC_X86) && defined (COMP_GCC)

/** \page SingleFloatFormat: Single-precision floating-point format (32 bits)
    \internal 	
	SEEEEEEE.EMMMMMMM.MMMMMMMM.MMMMMMMM
	S: Sign (0 - positive, 1 - negative)
	E: Exponent (plus 127, 8 bits)
	M: Mantissa (23 bits)
*/

/**
 * This is a relatively small inline routine which will compute sqrt(x)
 * very quick, taking a square root is now just a little longer than
 * doing a division. The function is inline so that it runs at maximal
 * possible speed. It does eight multiplications but multiplication is
 * cheap on P5+ processors (3 clocks). On a Celeron CPU it takes
 * approximatively 50 clocks, while a division is 42 clocks.
 */
 
static inline float qsqrt (float x)
{
  float ret;
  static float c1=0.5f;
  static float c2=1.5f;


// Original C++ formulae:
// float tmp = x;
// *((unsigned *)&tmp) = (0xbe6f0000 - *((unsigned *)&tmp)) >> 1;
// double h = x * 0.5;
// double a = tmp;
// a *= 1.5 - a * a * h;
// a *= 1.5 - a * a * h;
// return a * x;

  // Use __volatile__ so that the compiler will not mess with this
  // code. Under some versions of gcc in combination with -O2 optimize
  // mode not using __volatile__ can cause errors.
  __asm__ __volatile__ (
		"flds	%1\n"			// x
		"movl	$0xbe6f0000,%%eax\n"
		"subl	%1,%%eax\n"
		"shrl	$1,%%eax\n"
		"movl	%%eax,%1\n"
		"flds	%2\n"			// x 0.5
		"fmul	%%st(1)\n"		// x h
		"flds	%3\n"			// x h 1.5
		"flds	%1\n"			// x h 1.5 a
		"fld	%%st\n"			// x h 1.5 a a
		"fmul	%%st\n"			// x h 1.5 a a*a
		"fmul	%%st(3)\n"		// x h 1.5 a a*a*h
		"fsubr	%%st(2)\n"		// x h 1.5 a 1.5-a*a*h
		"fmulp	%%st(1)\n"		// x h 1.5 a
		"fld	%%st\n"			// x h 1.5 a a
		"fmul	%%st\n"			// x h 1.5 a a*a
		"fmulp	%%st(3)\n"		// x a*a*h 1.5 a
		"fxch\n"			// x a*a*h a 1.5
		"fsubp  %%st,%%st(2)\n"		// x 1.5-a*a*h a
		"fmulp	%%st(1)\n"		// x a
		"fmulp	%%st(1)\n"		// a
	: "=&t" (ret), "+m" (x) : "m" (c1), "m" (c2)
	: "eax", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)"
  );
  return ret;
}

/**
 * This routine is basically equivalent to qsqrt() except that it returns
 * 1/sqrt(x) rather than the proper square root. It should be used anywhere
 * you need the inverse root (in 3D graphics it is a common situation),
 * because the routine is a little faster than qsqrt() and also you avoid
 * a division (which gives you an overall 2X speedup).
 */
static inline float qisqrt (float x)
{
  float ret;
  static float c1=0.5f;
  static float c2=1.5f;
  // Use __volatile__ so that the compiler will not mess with this
  // code. Under some versions of gcc in combination with -O2 optimize
  // mode not using __volatile__ can cause errors.
  __asm__ __volatile__ (
		"flds	%1\n"			// x
		"movl	$0xbe6f0000,%%eax\n"
		"subl	%1,%%eax\n"
		"shrl	$1,%%eax\n"
		"movl	%%eax,%1\n"
		"flds	%2\n"			// x 0.5
		"fmulp	%%st(1)\n"		// h
		"flds	%3\n"			// h 1.5
		"flds	%1\n"			// h 1.5 a
		"fld	%%st\n"			// h 1.5 a a
		"fmul	%%st\n"			// h 1.5 a a*a
		"fmul	%%st(3)\n"		// h 1.5 a a*a*h
		"fsubr	%%st(2)\n"		// h 1.5 a 1.5-a*a*h
		"fmulp	%%st(1)\n"		// h 1.5 a
		"fld	%%st\n"			// h 1.5 a a
		"fmul	%%st\n"			// h 1.5 a a*a
		"fmulp	%%st(3)\n"		// a*a*h 1.5 a
		"fxch\n"			// a*a*h a 1.5
		"fsubp  %%st,%%st(2)\n"		// 1.5-a*a*h a
		"fmulp	%%st(1)\n"		// a
	: "=t" (ret), "+m" (x) : "m" (c1), "m" (c2)
	: "eax", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)"
  );
  return ret;
}

#elif (!defined (CS_NO_QSQRT)) && defined (PROC_POWERPC) && defined (COMP_GCC)

/**
 * Use the PowerPC fsqrte to get an estimate of 1/sqrt(x) Then apply two
 * Newton-Rhaphson refinement steps to get a more accurate response Finally
 * multiply by x to get x/sqrt(x) = sqrt(x).  Add additional refinement steps
 * to get a more accurate result.  Zero is treated as a special case, otherwise
 * we end up returning NaN (Not a Number).
 */
static inline float qsqrt(float x)
{
  float y0 = 0.0;

  if (x != 0.0)
  {
    float x0 = x * 0.5f;

    __asm__ __volatile__ ("frsqrte %0,%1" : "=f" (y0) : "f" (x));
    
    y0 = y0 * (1.5f - x0 * y0 * y0);
    y0 = (y0 * (1.5f - x0 * y0 * y0)) * x;
  };
    
  return y0;
};

/**
 * Similar to qsqrt() above, except we do not multiply by x at the end, and
 * return 1/sqrt(x).
 */
static inline float qisqrt(float x)
{
  float x0 = x * 0.5f;
  float y0;
  __asm__ __volatile__ ("frsqrte %0,%1" : "=f" (y0) : "f" (x));
    
  y0 = y0 * (1.5f - x0 * y0 * y0);
  y0 = y0 * (1.5f - x0 * y0 * y0);

  return y0;
};

#elif (!defined (CS_NO_QSQRT)) && defined (PROC_X86) && defined (COMP_VC)

#include <math.h>
static inline float qsqrt (float x) { return sqrtf(x); }
static inline float qisqrt(float x) { return 1.0f / sqrtf(x); }

#else

#include <math.h>
static inline float qsqrt (float x) { return (float)sqrt(x); }
static inline float qisqrt(float x) { return (float)(1.0 / sqrt(x)); }

#endif

#endif // __CS_QSQRT_H__
