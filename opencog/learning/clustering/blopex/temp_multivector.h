/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

#ifndef TEMPORARY_MULTIVECTOR_FUNCTION_PROTOTYPES
#define TEMPORARY_MULTIVECTOR_FUNCTION_PROTOTYPES

#include "interpreter.h"

typedef struct
{
  long   numVectors;
  BlopexInt*   mask;
  void** vector;
  BlopexInt    ownsVectors;
  BlopexInt    ownsMask;

  mv_InterfaceInterpreter* interpreter;

} mv_TempMultiVector;

typedef mv_TempMultiVector* mv_TempMultiVectorPtr;

/*******************************************************************/
/*
The above is a temporary implementation of the hypre_MultiVector
data type, just to get things going with LOBPCG eigensolver.

A more proper implementation would be to define hypre_MultiParVector,
hypre_MultiStructVector and hypre_MultiSStructVector by adding a new
record

BlopexInt numVectors;

in hypre_ParVector, hypre_StructVector and hypre_SStructVector,
and increasing the size of data numVectors times. Respective
modifications of most vector operations are straightforward
(it is strongly suggested that BLAS routines are used wherever
possible), efficient implementation of matrix-by-multivector
multiplication may be more difficult.

With the above implementation of hypre vectors, the definition
of hypre_MultiVector becomes simply (cf. multivector.h)

typedef struct
{
  void* multiVector;
  HYPRE_InterfaceInterpreter* interpreter;
} hypre_MultiVector;

with pointers to abstract multivector functions added to the structure
HYPRE_InterfaceInterpreter (cf. HYPRE_interpreter.h; particular values
are assigned to these pointers by functions
HYPRE_ParCSRSetupInterpreter, HYPRE_StructSetupInterpreter and
BlopexInt HYPRE_SStructSetupInterpreter),
and the abstract multivector functions become simply interfaces
to the actual multivector functions of the form (cf. multivector.c):

void
hypre_MultiVectorCopy( hypre_MultiVectorPtr src_, hypre_MultiVectorPtr dest_ ) {

  hypre_MultiVector* src = (hypre_MultiVector*)src_;
  hypre_MultiVector* dest = (hypre_MultiVector*)dest_;
  assert( src != NULL && dest != NULL );
  (src->interpreter->CopyMultiVector)( src->data, dest->data );
}


*/
/*********************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------- Generic ---------- */

void*
mv_TempMultiVectorCreateFromSampleVector( void*, BlopexInt n, void* sample );

void*
mv_TempMultiVectorCreateCopy( void*, BlopexInt copyValues );

void
mv_TempMultiVectorDestroy( void* );

BlopexInt
mv_TempMultiVectorWidth( void* v );

BlopexInt
mv_TempMultiVectorHeight( void* v );

void
mv_TempMultiVectorSetMask( void* v, BlopexInt* mask );

void
mv_TempMultiVectorClear( void* );

void
mv_TempMultiVectorSetRandom( void* v, BlopexInt seed );

void
mv_TempMultiVectorCopy( void* src, void* dest );

void
mv_TempMultiVectorEval( void (*f)( void*, void*, void* ), void* par,
               void* x, void* y );

/* ---------------------------- Double ---------- */

void
mv_TempMultiVectorAxpy( double, void*, void* );

void
mv_TempMultiVectorByMultiVector( void*, void*,
                    BlopexInt gh, BlopexInt h, BlopexInt w, void* v );

void
mv_TempMultiVectorByMultiVectorDiag( void* x, void* y,
                    BlopexInt* mask, BlopexInt n, void* diag );

void
mv_TempMultiVectorByMatrix( void*,
                   BlopexInt gh, BlopexInt h, BlopexInt w, void* v,
                   void* );

void
mv_TempMultiVectorXapy( void* x,
               BlopexInt gh, BlopexInt h, BlopexInt w, void* v,
               void* y );

void mv_TempMultiVectorByDiagonal( void* x,
                      BlopexInt* mask, BlopexInt n, void* diag,
                      void* y );


/* ---------------------------- Complex ---------- */

void
mv_TempMultiVectorAxpy_complex( double, void*, void* );

void
mv_TempMultiVectorByMultiVector_complex( void*, void*,
                    BlopexInt gh, BlopexInt h, BlopexInt w, void* v );

void
mv_TempMultiVectorByMultiVectorDiag_complex( void* x, void* y,
                    BlopexInt* mask, BlopexInt n, void* diag );

void
mv_TempMultiVectorByMatrix_complex( void*,
                   BlopexInt gh, BlopexInt h, BlopexInt w, void* v,
                   void* );

void
mv_TempMultiVectorXapy_complex( void* x,
               BlopexInt gh, BlopexInt h, BlopexInt w, void* v,
               void* y );

void mv_TempMultiVectorByDiagonal_complex( void* x,
                      BlopexInt* mask, BlopexInt n, void* diag,
                      void* y );

#ifdef __cplusplus
}
#endif

#endif /* MULTIVECTOR_FUNCTION_PROTOTYPES */
