/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

#include <assert.h>
#include <math.h>
#include <stdlib.h>

#include "temp_multivector.h"

/* ----------- complex definition ----------- */
typedef struct {double real, imag;} komplex;

static void
mv_collectVectorPtr( BlopexInt* mask, mv_TempMultiVector* x, void** px ) {

  BlopexInt ix, jx;

  if ( mask != NULL ) {
    for ( ix = 0, jx = 0; ix < x->numVectors; ix++ )
      if ( mask[ix] )
    px[jx++] = x->vector[ix];
  }
  else
    for ( ix = 0; ix < x->numVectors; ix++ )
      px[ix] = x->vector[ix];

}

static BlopexInt
aux_maskCount( BlopexInt n, BlopexInt* mask ) {

  BlopexInt i, m;

  if ( mask == NULL )
    return n;

  for ( i = m = 0; i < n; i++ )
    if ( mask[i] )
      m++;

  return m;
}

static void
aux_indexFromMask( BlopexInt n, BlopexInt* mask, BlopexInt* index ) {

  long i, j;

  if ( mask != NULL ) {
    for ( i = 0, j = 0; i < n; i++ )
      if ( mask[i] )
    index[j++] = i + 1;
  }
  else
    for ( i = 0; i < n; i++ )
      index[i] = i + 1;

}

/* ------- here goes simple random number generator --------- */

static unsigned long next = 1;

/* RAND_MAX assumed to be 32767 */
static BlopexInt myrand(void) {
   next = next * 1103515245 + 12345;
   return((unsigned)(next/65536) % 32768);
}

static void mysrand(unsigned seed) {
   next = seed;
}


void*
mv_TempMultiVectorCreateFromSampleVector( void* ii_, BlopexInt n, void* sample ) {

  BlopexInt i;
  mv_TempMultiVector* x;
  mv_InterfaceInterpreter* ii = (mv_InterfaceInterpreter*)ii_;

  x = (mv_TempMultiVector*) malloc(sizeof(mv_TempMultiVector));
  BlopexAssert( x != NULL );

  x->interpreter = ii;
  x->numVectors = n;

  x->vector = (void**) calloc( n, sizeof(void*) );
  BlopexAssert( x->vector != NULL );

  x->ownsVectors = 1;
  x->mask = NULL;
  x->ownsMask = 0;

  for ( i = 0; i < n; i++ )
    x->vector[i] = (ii->CreateVector)(sample);

  return x;

}

void*
mv_TempMultiVectorCreateCopy( void* src_, BlopexInt copyValues ) {

  BlopexInt i, n;

  mv_TempMultiVector* src;
  mv_TempMultiVector* dest;

  src = (mv_TempMultiVector*)src_;
  BlopexAssert( src != NULL );

  n = src->numVectors;

  dest = mv_TempMultiVectorCreateFromSampleVector( src->interpreter,
                              n, src->vector[0] );
  if ( copyValues )
    for ( i = 0; i < n; i++ ) {
      (dest->interpreter->CopyVector)(src->vector[i],dest->vector[i]);
  }

  return dest;
}

void
mv_TempMultiVectorDestroy( void* x_ ) {

  BlopexInt i;
  mv_TempMultiVector* x = (mv_TempMultiVector*)x_;

  if ( x == NULL )
    return;

  if ( x->ownsVectors && x->vector != NULL ) {
    for ( i = 0; i < x->numVectors; i++ )
      (x->interpreter->DestroyVector)(x->vector[i]);
    free(x->vector);
  }
  if ( x->mask && x->ownsMask )
    free(x->mask);
  free(x);
}

BlopexInt
mv_TempMultiVectorWidth( void* x_ ) {

  mv_TempMultiVector* x = (mv_TempMultiVector*)x_;

  if ( x == NULL )
    return 0;

  return x->numVectors;
}

BlopexInt
mv_TempMultiVectorHeight( void* x_ ) {

  mv_TempMultiVector* x = (mv_TempMultiVector*)x_;

  if ( x == NULL )
   return 0;

  return (x->interpreter->VectorSize)(x->vector[0]);
}

/* this shallow copy of the mask is convenient but not safe;
   a proper copy should be considered */
void
mv_TempMultiVectorSetMask( void* x_, BlopexInt* mask ) {

  mv_TempMultiVector* x = (mv_TempMultiVector*)x_;

  BlopexAssert( x != NULL );
  x->mask = mask;
  x->ownsMask = 0;
}

void
mv_TempMultiVectorClear( void* x_ ) {

  BlopexInt i;
  mv_TempMultiVector* x = (mv_TempMultiVector*)x_;

  BlopexAssert( x != NULL );

  for ( i = 0; i < x->numVectors; i++ )
    if ( x->mask == NULL || (x->mask)[i] )
      (x->interpreter->ClearVector)(x->vector[i]);
}

void
mv_TempMultiVectorSetRandom( void* x_, BlopexInt seed ) {

  BlopexInt i;
  mv_TempMultiVector* x = (mv_TempMultiVector*)x_;

  BlopexAssert( x != NULL );

  mysrand(seed);

  for ( i = 0; i < x->numVectors; i++ ) {
    if ( x->mask == NULL || (x->mask)[i] ) {
      seed=myrand();
      (x->interpreter->SetRandomValues)(x->vector[i], seed);
    }
  }
}



void
mv_TempMultiVectorCopy( void* src_, void* dest_ ) {

  BlopexInt i, ms, md;
  void** ps;
  void** pd;
  mv_TempMultiVector* src = (mv_TempMultiVector*)src_;
  mv_TempMultiVector* dest = (mv_TempMultiVector*)dest_;

  BlopexAssert( src != NULL && dest != NULL );

  ms = aux_maskCount( src->numVectors, src->mask );
  md = aux_maskCount( dest->numVectors, dest->mask );
  BlopexAssert( ms == md );

  ps = (void**) calloc( ms, sizeof(void*) );
  BlopexAssert( ps != NULL );
  pd = (void**) calloc( md, sizeof(void*) );
  BlopexAssert( pd != NULL );

  mv_collectVectorPtr( src->mask, src, ps );
  mv_collectVectorPtr( dest->mask, dest, pd );

  for ( i = 0; i < ms; i++ )
    (src->interpreter->CopyVector)(ps[i],pd[i]);

  free(ps);
  free(pd);
}

void
mv_TempMultiVectorAxpy( double a, void* x_, void* y_ ) {

  BlopexInt i, mx, my;
  void** px;
  void** py;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );
  BlopexAssert( mx == my );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  for ( i = 0; i < mx; i++ )
    (x->interpreter->Axpy)(&a,px[i],py[i]);

  free(px);
  free(py);
}

void
mv_TempMultiVectorAxpy_complex( double a, void* x_, void* y_ ) {

  BlopexInt i, mx, my;
  void** px;
  void** py;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;
  komplex alpha;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );
  BlopexAssert( mx == my );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  alpha.real = a;
  alpha.imag = 0;

  for ( i = 0; i < mx; i++ )
    (x->interpreter->Axpy)(&alpha,px[i],py[i]);

  free(px);
  free(py);
}

void
mv_TempMultiVectorByMultiVector( void* x_, void* y_,
                     BlopexInt xyGHeight, BlopexInt xyHeight,
                     BlopexInt xyWidth, void* xyVal ) {
/* xy = x'*y */

  BlopexInt ix, iy, mx, my, jxy;
  double* p;
  void** px;
  void** py;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  BlopexAssert( mx == xyHeight );

  my = aux_maskCount( y->numVectors, y->mask );
  BlopexAssert( my == xyWidth );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  jxy = xyGHeight - xyHeight;
  for ( iy = 0, p = (double *)xyVal; iy < my; iy++ ) {
    for ( ix = 0; ix < mx; ix++, p++ )
      (x->interpreter->InnerProd)(px[ix],py[iy],p);
    p += jxy;
  }

  free(px);
  free(py);

}

void
mv_TempMultiVectorByMultiVector_complex( void* x_, void* y_,
                     BlopexInt xyGHeight, BlopexInt xyHeight,
                     BlopexInt xyWidth, void* xyVal ) {
/* xy = x'*y */

  BlopexInt ix, iy, mx, my, jxy;
  komplex* p;
  void** px;
  void** py;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  BlopexAssert( mx == xyHeight );

  my = aux_maskCount( y->numVectors, y->mask );
  BlopexAssert( my == xyWidth );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  jxy = xyGHeight - xyHeight;
  for ( iy = 0, p = (komplex *) xyVal; iy < my; iy++ ) {
    for ( ix = 0; ix < mx; ix++, p++ )
/* reverse py and px here to insure transpose is on px */
      (x->interpreter->InnerProd)(py[iy],px[ix],p);
    p += jxy;
  }

  free(px);
  free(py);

}

void
mv_TempMultiVectorByMultiVectorDiag( void* x_, void* y_,
                    BlopexInt* mask, BlopexInt n, void* diag ) {
/* diag = diag(x'*y) */

  BlopexInt i, mx, my, m;
  void** px;
  void** py;
  BlopexInt* index;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;
  double * dp;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );
  m = aux_maskCount( n, mask );
  BlopexAssert( mx == my && mx == m );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  index = (BlopexInt*)calloc( m, sizeof(BlopexInt) );
  aux_indexFromMask( n, mask, index );

  dp = (double *)diag;

  for ( i = 0; i < m; i++ )
     (x->interpreter->InnerProd)(px[i],py[i],dp+index[i]-1);

  free(index);
  free(px);
  free(py);

}

void
mv_TempMultiVectorByMultiVectorDiag_complex( void* x_, void* y_,
                    BlopexInt* mask, BlopexInt n, void* diag ) {
/* diag = diag(x'*y) */

  BlopexInt i, mx, my, m;
  void** px;
  void** py;
  BlopexInt* index;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;
  komplex * dp;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );
  m = aux_maskCount( n, mask );
  BlopexAssert( mx == my && mx == m );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  index = (BlopexInt*)calloc( m, sizeof(BlopexInt) );
  aux_indexFromMask( n, mask, index );

  dp = (komplex *)diag;

  for ( i = 0; i < m; i++ )
  /* reverse py and px here to insure transpose is on px */
     (x->interpreter->InnerProd)(py[i],px[i],dp+index[i]-1);

  free(index);
  free(px);
  free(py);

}

void
mv_TempMultiVectorByMatrix( void* x_,
                   BlopexInt rGHeight, BlopexInt rHeight,
                   BlopexInt rWidth, void* rVal,
                   void* y_ ) {

  BlopexInt i, j, jump;
  BlopexInt mx, my;
  double* p;
  void** px;
  void** py;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );

  BlopexAssert( mx == rHeight && my == rWidth );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  jump = rGHeight - rHeight;
  for ( j = 0, p = (double *) rVal; j < my; j++ ) {
    (x->interpreter->ClearVector)( py[j] );
    for ( i = 0; i < mx; i++, p++ )
      (x->interpreter->Axpy)(p,px[i],py[j]);
    p += jump;
  }

  free(px);
  free(py);
}

void
mv_TempMultiVectorByMatrix_complex( void* x_,
                   BlopexInt rGHeight, BlopexInt rHeight,
                   BlopexInt rWidth, void* rVal,
                   void* y_ ) {

  BlopexInt i, j, jump;
  BlopexInt mx, my;
  komplex* p;
  void** px;
  void** py;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );

  BlopexAssert( mx == rHeight && my == rWidth );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  jump = rGHeight - rHeight;
  for ( j = 0, p = (komplex *) rVal; j < my; j++ ) {
    (x->interpreter->ClearVector)( py[j] );
    for ( i = 0; i < mx; i++, p++ )
      (x->interpreter->Axpy)(p,px[i],py[j]);
    p += jump;
  }

  free(px);
  free(py);
}

void
mv_TempMultiVectorXapy( void* x_,
               BlopexInt rGHeight, BlopexInt rHeight,
               BlopexInt rWidth, void* rVal,
               void* y_ ) {

  BlopexInt i, j, jump;
  BlopexInt mx, my;
  double* p;
  void** px;
  void** py;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );

  BlopexAssert( mx == rHeight && my == rWidth );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  jump = rGHeight - rHeight;
  for ( j = 0, p = (double *)rVal; j < my; j++ ) {
    for ( i = 0; i < mx; i++, p++ )
      (x->interpreter->Axpy)(p,px[i],py[j]);
    p += jump;
  }

  free(px);
  free(py);
}

void
mv_TempMultiVectorXapy_complex( void* x_,
               BlopexInt rGHeight, BlopexInt rHeight,
               BlopexInt rWidth, void* rVal,
               void* y_ ) {

  BlopexInt i, j, jump;
  BlopexInt mx, my;
  komplex* p;
  void** px;
  void** py;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );

  BlopexAssert( mx == rHeight && my == rWidth );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  jump = rGHeight - rHeight;
  for ( j = 0, p = (komplex *)rVal; j < my; j++ ) {
    for ( i = 0; i < mx; i++, p++ )
      (x->interpreter->Axpy)(p,px[i],py[j]);
    p += jump;
  }

  free(px);
  free(py);
}

void
mv_TempMultiVectorByDiagonal( void* x_,
                BlopexInt* mask, BlopexInt n, void* diag,
                void* y_ ) {

  BlopexInt j;
  BlopexInt mx, my, m;
  void** px;
  void** py;
  BlopexInt* index;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;
  double * dp;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );
  m = aux_maskCount( n, mask );

  BlopexAssert( mx == m && my == m );

  if ( m < 1 )
    return;

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  index = (BlopexInt*)calloc( m, sizeof(BlopexInt) );
  aux_indexFromMask( n, mask, index );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  dp = (double *)diag;

  for ( j = 0; j < my; j++ ) {
    (x->interpreter->ClearVector)(py[j]);
    (x->interpreter->Axpy)(dp+index[j]-1,px[j],py[j]);
  }

  free(px);
  free(py);
  free( index );
}

void
mv_TempMultiVectorByDiagonal_complex( void* x_,
                BlopexInt* mask, BlopexInt n, void* diag,
                void* y_ ) {

  BlopexInt j;
  BlopexInt mx, my, m;
  void** px;
  void** py;
  BlopexInt* index;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;
  komplex * dp;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );
  m = aux_maskCount( n, mask );

  BlopexAssert( mx == m && my == m );

  if ( m < 1 )
    return;

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  index = (BlopexInt*)calloc( m, sizeof(BlopexInt) );
  aux_indexFromMask( n, mask, index );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  dp = (komplex *)diag;

  for ( j = 0; j < my; j++ ) {
    (x->interpreter->ClearVector)(py[j]);
    (x->interpreter->Axpy)(dp+index[j]-1,px[j],py[j]);
  }

  free(px);
  free(py);
  free( index );
}

void
mv_TempMultiVectorEval( void (*f)( void*, void*, void* ), void* par,
               void* x_, void* y_ ) {

  long i, mx, my;
  void** px;
  void** py;
  mv_TempMultiVector* x;
  mv_TempMultiVector* y;

  x = (mv_TempMultiVector*)x_;
  y = (mv_TempMultiVector*)y_;
  BlopexAssert( x != NULL && y != NULL );

  if ( f == NULL ) {
    mv_TempMultiVectorCopy( x, y );
    return;
  }

  mx = aux_maskCount( x->numVectors, x->mask );
  my = aux_maskCount( y->numVectors, y->mask );
  BlopexAssert( mx == my );

  px = (void**) calloc( mx, sizeof(void*) );
  BlopexAssert( px != NULL );
  py = (void**) calloc( my, sizeof(void*) );
  BlopexAssert( py != NULL );

  mv_collectVectorPtr( x->mask, x, px );
  mv_collectVectorPtr( y->mask, y, py );

  for ( i = 0; i < mx; i++ )
    f( par, (void*)px[i], (void*)py[i] );

  free(px);
  free(py);
}
