/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* This header added to the top of multivector.h, interpreter.h, 
 * fortran_interpreter.h, and fortran_matrix.h
 */

/* Default BlopexInt to int
 * Use -DBlopexInt=long in compiler options to override
 * when 64bit integer array sizes are needed, such as in the latest
 * releases of Matlab or for 64bit versions of Lapack.
 */
#ifndef BlopexInt
#define BlopexInt int
#endif

#define BlopexAssert assert
