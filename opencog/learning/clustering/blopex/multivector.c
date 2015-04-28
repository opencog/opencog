/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

#include <assert.h>
#include <math.h>
#include <stdlib.h>

#include "multivector.h"

/* -----------------------------------------------------
   abstract multivector
   ----------------------------------------------------- */
struct mv_MultiVector
{
  void* data;      /* the pointer to the actual multivector */
  BlopexInt   ownsData;

  mv_InterfaceInterpreter* interpreter; /* a structure that defines
                                           multivector operations */
} ;
/* ------------------------------------------------------
   mv_MultiVectorGetData                          generic
   ------------------------------------------------------ */
void *
mv_MultiVectorGetData (mv_MultiVectorPtr x)
{
  BlopexAssert (x!=NULL);
  return x->data;
}
/* -------------------------------------------------------
   mv_MultiVectorWrap                             generic
   ------------------------------------------------------- */
mv_MultiVectorPtr
mv_MultiVectorWrap( mv_InterfaceInterpreter* ii, void * data, BlopexInt ownsData )
{
  mv_MultiVectorPtr x;

  x = (mv_MultiVectorPtr) malloc(sizeof(struct mv_MultiVector));
  BlopexAssert( x != NULL );

  x->interpreter = ii;
  x->data = data;
  x->ownsData = ownsData;

  return x;
}
/* ---------------------------------------------------------------
   mv_MultiVectorCreateFromSampleVector      not used      generic
   --------------------------------------------------------------- */
mv_MultiVectorPtr
mv_MultiVectorCreateFromSampleVector( void* ii_, BlopexInt n, void* sample ) {

  mv_MultiVectorPtr x;
  mv_InterfaceInterpreter* ii = (mv_InterfaceInterpreter*)ii_;

  x = (mv_MultiVectorPtr) malloc(sizeof(struct mv_MultiVector));
  BlopexAssert( x != NULL );

  x->interpreter = ii;
  x->data = (ii->CreateMultiVector)( ii, n, sample );
  x->ownsData = 1;

  return x;
}
/* ------------------------------------------------------------------
   mv_MultiVectorCreateCopy                                   generic
   ------------------------------------------------------------------ */
mv_MultiVectorPtr
mv_MultiVectorCreateCopy( mv_MultiVectorPtr x, BlopexInt copyValues ) {

  mv_MultiVectorPtr y;
  void* data;
  mv_InterfaceInterpreter* ii;

  BlopexAssert( x != NULL );
  ii = x->interpreter;

  y = (mv_MultiVectorPtr) malloc(sizeof(struct mv_MultiVector));
  BlopexAssert( y != NULL );

  data = (ii->CopyCreateMultiVector)( x->data, copyValues );

  y->interpreter = ii;
  y->data = data;
  y->ownsData = 1;

  return y;
}
/* ------------------------------------------------------------------
   mv_MultiVectorDestroy                                      generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorDestroy( mv_MultiVectorPtr v) {

  if ( v == NULL )
    return;

  if ( v->ownsData )
    (v->interpreter->DestroyMultiVector)( v->data );
  free( v );
}
/* ------------------------------------------------------------------
   mv_MultiVectorSetMask                                      generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorSetMask( mv_MultiVectorPtr v, BlopexInt* mask ) {

  BlopexAssert( v != NULL );
  (v->interpreter->SetMask)( v->data, mask );
}
/* ------------------------------------------------------------------
   mv_MultiVectorWidth                                        generic
   ------------------------------------------------------------------ */
BlopexInt
mv_MultiVectorWidth( mv_MultiVectorPtr v ) {

  if ( v == NULL )
    return 0;

  return (v->interpreter->Width)( v->data );
}
/* ------------------------------------------------------------------
   mv_MultiVectorHeight                 not used              generic
   ------------------------------------------------------------------ */
BlopexInt
mv_MultiVectorHeight( mv_MultiVectorPtr v ) {

  if ( v == NULL )
    return 0;

  return (v->interpreter->Height)(v->data);
}
/* ------------------------------------------------------------------
   mv_MultiVectorClear                                        generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorClear( mv_MultiVectorPtr v ) {

  BlopexAssert( v != NULL );
  (v->interpreter->ClearMultiVector)( v->data );
}
/* ------------------------------------------------------------------
   mv_MultiVectorSetRandom                                    generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorSetRandom( mv_MultiVectorPtr v, BlopexInt seed ) {

  BlopexAssert( v != NULL );
  (v->interpreter->SetRandomVectors)( v->data, seed );
}
/* ------------------------------------------------------------------
   mv_MultiVectorCopy                                         generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorCopy( mv_MultiVectorPtr src, mv_MultiVectorPtr dest ) {

  BlopexAssert( src != NULL && dest != NULL );
  (src->interpreter->CopyMultiVector)( src->data, dest->data );
}
/* ------------------------------------------------------------------
   mv_MultiVectorAxpy        y=y+a*x                          generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorAxpy( double a, mv_MultiVectorPtr x, mv_MultiVectorPtr y ) {

  BlopexAssert( x != NULL && y != NULL );
  (x->interpreter->MultiAxpy)( a, x->data, y->data );
}
/* ------------------------------------------------------------------
   mv_MultiVectorByMultiVector    xy=x'*y                     generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorByMultiVector( mv_MultiVectorPtr x,
                             mv_MultiVectorPtr y,
                             BlopexInt xyGHeight, BlopexInt xyHeight,
                             BlopexInt xyWidth, void* xy ) {

  BlopexAssert( x != NULL && y != NULL );
  (x->interpreter->MultiInnerProd)
    ( x->data, y->data, xyGHeight, xyHeight, xyWidth, xy );
}
/* ------------------------------------------------------------------
   mv_MultiVectorByMultiVectorDiag     d=diag(x'*y)           generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorByMultiVectorDiag( mv_MultiVectorPtr x,
                                 mv_MultiVectorPtr y,
                                 BlopexInt* mask, BlopexInt n, void* d ) {

  BlopexAssert( x != NULL && y != NULL );
  (x->interpreter->MultiInnerProdDiag)( x->data, y->data, mask, n, d );
}
/* ------------------------------------------------------------------
   mv_MultiVectorByMatrix              y=x*r                  generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorByMatrix( mv_MultiVectorPtr x,
                        BlopexInt rGHeight, BlopexInt rHeight, BlopexInt rWidth,
                        void* rVal,
                        mv_MultiVectorPtr y ) {

  BlopexAssert( x != NULL && y != NULL );
  (x->interpreter->MultiVecMat)
    ( x->data, rGHeight, rHeight, rWidth, rVal, y->data );
}
/* ------------------------------------------------------------------
   mv_MultiVectorXapy                y=y+x*a      not used    generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorXapy( mv_MultiVectorPtr x,
                    BlopexInt rGHeight, BlopexInt rHeight, BlopexInt rWidth,
                    void* rVal,
                    mv_MultiVectorPtr y ) {

  BlopexAssert( x != NULL && y != NULL );
  (x->interpreter->MultiXapy)
    ( x->data, rGHeight, rHeight, rWidth, rVal, y->data );
}
/* ------------------------------------------------------------------
   mv_MultiVectorByDiagonal            y=x*d(mask)            generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorByDiagonal( mv_MultiVectorPtr x,
                          BlopexInt* mask, BlopexInt n, void* d,
                          mv_MultiVectorPtr y ) {

  /* y = x*d */

  BlopexAssert( x != NULL && y != NULL );
  (x->interpreter->MultiVecMatDiag)( x->data, mask, n, d, y->data );
}
/* ------------------------------------------------------------------
   mv_MultiVectorEval                     not used            generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorEval( void (*f)( void*, void*, void* ), void* par,
               mv_MultiVectorPtr x, mv_MultiVectorPtr y ) {

  /* y = f(x) computed vector-wise */

  BlopexAssert( x != NULL && y != NULL );
  (x->interpreter->Eval)( f, par, x->data, y->data );
}
/* ------------------------------------------------------------------
   mv_MultiVectorPrint                                        generic
   ------------------------------------------------------------------ */
void
mv_MultiVectorPrint( mv_MultiVectorPtr x,char * tag,BlopexInt limit ) {

  BlopexAssert( x != NULL );
  (x->interpreter->MultiPrint)( x->data, tag, limit );
}
