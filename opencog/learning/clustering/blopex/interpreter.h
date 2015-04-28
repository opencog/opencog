/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

#include "fortran_options.h"

#ifndef LOBPCG_INTERFACE_INTERPRETER
#define LOBPCG_INTERFACE_INTERPRETER

typedef struct
{
  /* vector operations */
  void*  (*CreateVector)  ( void *vector );
  BlopexInt    (*DestroyVector) ( void *vector );

  BlopexInt    (*InnerProd)     ( void *x, void *y, void *result );
  BlopexInt    (*CopyVector)    ( void *x, void *y );
  BlopexInt    (*ClearVector)   ( void *x );
  BlopexInt    (*SetRandomValues)   ( void *x, BlopexInt seed );
  BlopexInt    (*ScaleVector)   ( double alpha, void *x );
  BlopexInt    (*Axpy)          ( void * alpha, void *x, void *y );
  BlopexInt    (*VectorSize)    (void * vector);

  /* multivector operations */
  /* do we need the following entry? */
  void*  (*CreateMultiVector)  ( void*, BlopexInt n, void *vector );
  void*  (*CopyCreateMultiVector)  ( void *x, BlopexInt );
  void    (*DestroyMultiVector) ( void *x );

  BlopexInt    (*Width)  ( void *x );
  BlopexInt    (*Height) ( void *x );

  void   (*SetMask) ( void *x, BlopexInt *mask );

  void   (*CopyMultiVector)    ( void *x, void *y );
  void   (*ClearMultiVector)   ( void *x );
  void   (*SetRandomVectors)   ( void *x, BlopexInt seed );
  void   (*MultiInnerProd)     ( void *x, void *y, BlopexInt, BlopexInt, BlopexInt, void* );
  void   (*MultiInnerProdDiag) ( void *x, void *y, BlopexInt*, BlopexInt, void* );
  void   (*MultiVecMat)        ( void *x, BlopexInt, BlopexInt, BlopexInt, void*, void *y );
  void   (*MultiVecMatDiag)    ( void *x, BlopexInt*, BlopexInt, void*, void *y );
  void   (*MultiAxpy)          ( double alpha, void *x, void *y );
  void   (*MultiPrint)         ( void *x, char * tag,BlopexInt limit );

  /* do we need the following 2 entries? */
  void   (*MultiXapy)          ( void *x, BlopexInt, BlopexInt, BlopexInt, void*, void *y );
  void   (*Eval)               ( void (*f)( void*, void*, void* ), void*, void *x, void *y );

} mv_InterfaceInterpreter;

#endif
