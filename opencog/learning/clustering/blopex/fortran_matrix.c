/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "fortran_matrix.h"

typedef struct {

  double    absolute;
  double    relative;

} lobpcg_Tolerance;


/* ----------------------------------------------------------------
   complex_multiply                A=B*C                    complex
   ---------------------------------------------------------------- */
void
complex_multiply(komplex* A, komplex* B, komplex* C) {

  komplex D;

  D.real = B->real * C->real - B->imag * C->imag;
  D.imag = B->real * C->imag + C->real * B->imag;

  *A=D;
}

/* ----------------------------------------------------------------
   complex_add                   A=B+C                    complex
   ---------------------------------------------------------------- */
void
complex_add(komplex* A, komplex* B, komplex* C) {

  A->real = B->real + C->real;
  A->imag = B->imag + C->imag;
}
/* ----------------------------------------------------------------
   complex_subtract                A=B-C                    complex
   ---------------------------------------------------------------- */
void
complex_subtract(komplex* A, komplex* B, komplex* C) {

  A->real = B->real - C->real;
  A->imag = B->imag - C->imag;
}
/* ----------------------------------------------------------------
   complex_divide                A=B/C                    complex
   ---------------------------------------------------------------- */
void
complex_divide(komplex* A, komplex* B, komplex* C) {

  double denom;
  komplex D;

  denom = C->real * C->real + C->imag * C->imag;

  D.real = ( B->real * C->real + B->imag * C->imag ) / denom;
  D.imag = ( C->real * B->imag - B->real * C->imag ) / denom;

  *A=D;
}


/* ------------------------------------------------------------
   utilities_FortranMatrixCreate                        generic
   ------------------------------------------------------------ */
utilities_FortranMatrix*
utilities_FortranMatrixCreate(void) {

  utilities_FortranMatrix* mtx;

  mtx = (utilities_FortranMatrix*) malloc( sizeof(utilities_FortranMatrix) );
  BlopexAssert( mtx != NULL );

  mtx->globalHeight = 0;
  mtx->height = 0;
  mtx->width = 0;
  mtx->value = NULL;
  mtx->ownsValues = 0;

  return mtx;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixAllocateData                   double
   ------------------------------------------------------------ */
void
utilities_FortranMatrixAllocateData( long  h, long w,
                                     utilities_FortranMatrix* mtx ) {

  BlopexAssert( h > 0 && w > 0 );
  BlopexAssert( mtx != NULL );

  if ( mtx->value != NULL && mtx->ownsValues )
    free( mtx->value );

  mtx->value = calloc( h*w, sizeof(double) );
  BlopexAssert ( mtx->value != NULL );

  mtx->globalHeight = h;
  mtx->height = h;
  mtx->width = w;
  mtx->ownsValues = 1;
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixAllocateData                 complex
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixAllocateData( long  h, long w,
                                      utilities_FortranMatrix* mtx ) {

  BlopexAssert( h > 0 && w > 0 );
  BlopexAssert( mtx != NULL );

  if ( mtx->value != NULL && mtx->ownsValues )
    free( mtx->value );

  mtx->value = calloc( h*w, sizeof(komplex) );
  BlopexAssert ( mtx->value != NULL );

  mtx->globalHeight = h;
  mtx->height = h;
  mtx->width = w;
  mtx->ownsValues = 1;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixWrap                          generic
   ------------------------------------------------------------ */
void
utilities_FortranMatrixWrap( void* v, long gh, long  h, long w,
                             utilities_FortranMatrix* mtx ) {

  BlopexAssert( h > 0 && w > 0 );
  BlopexAssert( mtx != NULL );

  if ( mtx->value != NULL && mtx->ownsValues )
    free( mtx->value );

  mtx->value = v;
  BlopexAssert ( mtx->value != NULL );

  mtx->globalHeight = gh;
  mtx->height = h;
  mtx->width = w;
  mtx->ownsValues = 0;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixDestroy                       generic
   ------------------------------------------------------------ */
void
utilities_FortranMatrixDestroy( utilities_FortranMatrix* mtx ) {

  if ( mtx == NULL )
    return;

  if ( mtx->ownsValues && mtx->value != NULL )
    free(mtx->value);

  free(mtx);
}
/* ------------------------------------------------------------
   utilities_FortranMatrixGlobalHeight                  generic
   ------------------------------------------------------------ */
long
utilities_FortranMatrixGlobalHeight( utilities_FortranMatrix* mtx ) {

  BlopexAssert( mtx != NULL );

  return mtx->globalHeight;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixHeight                        generic
   ------------------------------------------------------------ */
long
utilities_FortranMatrixHeight( utilities_FortranMatrix* mtx ) {

  BlopexAssert( mtx != NULL );

  return mtx->height;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixWidth                         generic
   ------------------------------------------------------------ */
long
utilities_FortranMatrixWidth( utilities_FortranMatrix* mtx ) {

  BlopexAssert( mtx != NULL );

  return mtx->width;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixValues                        generic
   ------------------------------------------------------------ */
void*
utilities_FortranMatrixValues( utilities_FortranMatrix* mtx ) {

  BlopexAssert( mtx != NULL );

  return mtx->value;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixClear        mtx=0.0           double
   ------------------------------------------------------------ */
void
utilities_FortranMatrixClear( utilities_FortranMatrix* mtx ) {

  long i, j, h, w, jump;
  double* p;

  BlopexAssert( mtx != NULL );

  h = mtx->height;
  w = mtx->width;

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (double *) mtx->value; j < w; j++ ) {
    for ( i = 0; i < h; i++, p++ )
      *p = 0.0;
    p += jump;
  }
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixClear        mtx=0.0         complex
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixClear( utilities_FortranMatrix* mtx ) {

  long i, j, h, w, jump;
  komplex* p;

  BlopexAssert( mtx != NULL );

  h = mtx->height;
  w = mtx->width;

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (komplex *) mtx->value; j < w; j++ ) {
    for ( i = 0; i < h; i++, p++ ) {
      p->real = 0.0;   /* real part */
      p->imag = 0.0;   /* imag part */
    }
    p += jump;
  }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixClearL      lower(mtx)=0.0     double
   ------------------------------------------------------------ */
void
utilities_FortranMatrixClearL( utilities_FortranMatrix* mtx ) {

  long i, j, k, h, w, jump;
  double* p;

  BlopexAssert( mtx != NULL );

  h = mtx->height;
  w = mtx->width;

  if ( w > h )
    w = h;

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (double *) mtx->value; j < w - 1; j++ ) {
    k = j + 1;
    p += k;
    for ( i = k; i < h; i++, p++ )
      *p = 0.0;
    p += jump;
  }
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixClearL      lower(mtx)=0.0   complex
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixClearL( utilities_FortranMatrix* mtx ) {

  long i, j, k, h, w, jump;
  komplex* p;

  BlopexAssert( mtx != NULL );

  h = mtx->height;
  w = mtx->width;

  if ( w > h )
    w = h;

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (komplex *) mtx->value; j < w - 1; j++ ) {
    k = j + 1;
    p += k;
    for ( i = k; i < h; i++, p++ ) {
      p->real = 0.0;   /* real part */
      p->imag = 0.0;   /* imag part */
    }
    p += jump;
  }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixSetToIdentity    mtx=eye       double
   ------------------------------------------------------------ */
void
utilities_FortranMatrixSetToIdentity( utilities_FortranMatrix* mtx ) {

  long j, h, w, jump;
  double* p;

  BlopexAssert( mtx != NULL );

  utilities_FortranMatrixClear( mtx );

  h = mtx->height;
  w = mtx->width;

  jump = mtx->globalHeight;

  for ( j = 0, p = (double *) mtx->value; j < w && j < h; j++, p += jump )
    *p++ = 1.0;

}
/* ------------------------------------------------------------
   zutilities_FortranMatrixSetToIdentity    mtx=eye     complex
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixSetToIdentity( utilities_FortranMatrix* mtx ) {

  long j, h, w, jump;
  komplex* p;

  BlopexAssert( mtx != NULL );

  utilities_FortranMatrixClear( mtx );

  h = mtx->height;
  w = mtx->width;

  jump = mtx->globalHeight;

  for ( j = 0, p = (komplex *) mtx->value; j < w && j < h; j++, p += jump+1 ) {
    p->real = 1.0;  /* real part */
    p->imag = 0.0;  /* imag part */
    }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixTransposeSquare    mtx=mtx'    double
   ------------------------------------------------------------ */
void
utilities_FortranMatrixTransposeSquare( utilities_FortranMatrix* mtx ) {

  long i, j, g, h, w, jump;
  double* p;
  double* q;
  double tmp;

  BlopexAssert( mtx != NULL );

  g = mtx->globalHeight;
  h = mtx->height;
  w = mtx->width;

  BlopexAssert( h == w );

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (double *) mtx->value; j < w; j++ ) {
    q = p;
    p++;
    q += g;
    for ( i = j + 1; i < h; i++, p++, q += g ) {
      tmp = *p;
      *p = *q;
      *q = tmp;
    }
    p += ++jump;
  }
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixTransposeSquare    mtx=mtx'  complex
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixTransposeSquare( utilities_FortranMatrix* mtx ) {

  long i, j, g, h, w, jump;
  komplex* p;
  komplex* q;
  komplex  tmp;

  BlopexAssert( mtx != NULL );

  g = mtx->globalHeight;
  h = mtx->height;
  w = mtx->width;

  BlopexAssert( h == w );

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (komplex *) mtx->value; j < w; j++ ) {
    q = p;
    q->imag = -(q->imag);
    p++;
    q += g;
    for ( i = j + 1; i < h; i++, p++, q += g ) {
      tmp = *p;
      tmp.imag = -tmp.imag;
      *p = *q;
      p->imag = -(p->imag);
      *q = tmp;
    }
    p += ++jump;
  }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixSymmetrize                     double
                                     mtx=(mtx+transpose(mtx))/2
   ------------------------------------------------------------ */
void
utilities_FortranMatrixSymmetrize( utilities_FortranMatrix* mtx ) {

  long i, j, g, h, w, jump;
  double* p;
  double* q;

  BlopexAssert( mtx != NULL );

  g = mtx->globalHeight;
  h = mtx->height;
  w = mtx->width;

  BlopexAssert( h == w );

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (double *) mtx->value; j < w; j++ ) {
    q = p;
    p++;
    q += g;
    for ( i = j + 1; i < h; i++, p++, q += g )
      *p = *q = (*p + *q)*0.5;
    p += ++jump;
  }
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixSymmetrize                   complex
                                     mtx=(mtx+transpose(mtx))/2
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixSymmetrize( utilities_FortranMatrix* mtx ) {

  long i, j, g, h, w, jump;
  komplex* p;
  komplex* q;
  komplex conj_q;

  BlopexAssert( mtx != NULL );

  g = mtx->globalHeight;
  h = mtx->height;
  w = mtx->width;

  BlopexAssert( h == w );

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (komplex *) mtx->value; j < w; j++ ) {
    q = p;
    for ( i = j ; i < h; i++, p++, q += g ) {
      conj_q.real = q->real;
      conj_q.imag = - (q->imag);
      complex_add(p,p,&conj_q);
      p->real = p->real/2;
      p->imag = p->imag/2;
      q->real = p->real;
      q->imag = -(p->imag);
    }
    p += ++jump;
  }


}
/* ------------------------------------------------------------
   utilities_FortranMatrix                               double
                                 t eq 0   src=dest
                                 t ne 0   src=dest'
   ------------------------------------------------------------ */
void
utilities_FortranMatrixCopy( utilities_FortranMatrix* src,
                             BlopexInt t,
                             utilities_FortranMatrix* dest ) {

  long i, j, h, w;
  long jp, jq, jr;
  double* p;
  double* q;
  double* r;

  BlopexAssert( src != NULL && dest != NULL );

  h = dest->height;
  w = dest->width;

  jp = dest->globalHeight - h;

  if ( t == 0 ) {
    BlopexAssert( src->height == h && src->width == w );
    jq = 1;
    jr = src->globalHeight;
  }
  else {
    BlopexAssert( src->height == w && src->width == h );
    jr = 1;
    jq = src->globalHeight;
  }

  for ( j = 0, p = (double *) dest->value, r = (double *) src->value;
        j < w; j++, p += jp, r += jr )
    for ( i = 0, q = r; i < h; i++, p++, q += jq )
      *p = *q;
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixCopy                         complex
                                 t eq 0   src=dest
                                 t ne 0   src=dest'
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixCopy( utilities_FortranMatrix* src,
                              BlopexInt t,
                              utilities_FortranMatrix* dest ) {

  long i, j, h, w;
  long jp, jq, jr;
  komplex* p;
  komplex* q;
  komplex* r;

  BlopexAssert( src != NULL && dest != NULL );

  h = dest->height;
  w = dest->width;

  jp = dest->globalHeight - h;

  if ( t == 0 ) {
    BlopexAssert( src->height == h && src->width == w );
    jq = 1;
    jr = src->globalHeight;
  }
  else {
    BlopexAssert( src->height == w && src->width == h );
    jr = 1;
    jq = src->globalHeight;
  }

  for ( j = 0, p = (komplex *) dest->value, r = (komplex *) src->value;
        j < w; j++, p += jp, r += jr )
    for ( i = 0, q = r; i < h; i++, p++, q += jq ) {
      p->real = q->real;   /* real part */
      if (t==0)
        p->imag = q->imag; /* imag part */
      else
        p->imag = -(q->imag);
    }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixIndexCopy                      double
                                    t eq 0   dest=index(src)
                                    t ne 0   dest=index(src')
   ------------------------------------------------------------ */
void
utilities_FortranMatrixIndexCopy( BlopexInt* index,
                                  utilities_FortranMatrix* src,
                                  BlopexInt t,
                                  utilities_FortranMatrix* dest ) {

  long i, j, h, w;
  long jp, jq, jr;
  double* p;
  double* q;
  double* r;

  BlopexAssert( src != NULL && dest != NULL );

  h = dest->height;
  w = dest->width;

  jp = dest->globalHeight - h;

  if ( t == 0 ) {
    BlopexAssert( src->height == h && src->width == w );
    jq = 1;
    jr = src->globalHeight;
  }
  else {
    BlopexAssert( src->height == w && src->width == h );
    jr = 1;
    jq = src->globalHeight;
  }

  for ( j = 0, p = (double *) dest->value; j < w; j++, p += jp ) {
    r = (double *) src->value + (index[j]-1)*jr;
    for ( i = 0, q = r; i < h; i++, p++, q += jq )
      *p = *q;
  }
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixIndexCopy                    complex
                                    t eq 0   dest=index(src)
                                    t ne 0   dest=index(src')
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixIndexCopy( BlopexInt* index,
                                   utilities_FortranMatrix* src,
                                   BlopexInt t,
                                   utilities_FortranMatrix* dest ) {

  long i, j, h, w;
  long jp, jq, jr;
  komplex* p;
  komplex* q;
  komplex* r;

  BlopexAssert( src != NULL && dest != NULL );

  h = dest->height;
  w = dest->width;

  jp = dest->globalHeight - h;

  if ( t == 0 ) {
    BlopexAssert( src->height == h && src->width == w );
    jq = 1;
    jr = src->globalHeight;
  }
  else {
    BlopexAssert( src->height == w && src->width == h );
    jr = 1;
    jq = src->globalHeight;
  }

  for ( j = 0, p = (komplex *) dest->value; j < w; j++, p += jp ) {
    r = (komplex *) src->value + (index[j]-1)*jr;
    for ( i = 0, q = r; i < h; i++, p++, q += jq ) {
      p->real = q->real;    /* real part */
      if (t==0)
        p->imag = q->imag;  /* imag part */
      else
        p->imag = -(q->imag);
    }
  }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixSetDiagonal  diag(mtx)=vec     double
   ------------------------------------------------------------ */
void
utilities_FortranMatrixSetDiagonal( utilities_FortranMatrix* mtx,
                                    utilities_FortranMatrix* vec ) {

  long j, h, w, jump;
  double* p;
  double* q;

  BlopexAssert( mtx != NULL && vec != NULL );

  h = mtx->height;
  w = mtx->width;

  BlopexAssert( vec->height >= h );

  jump = mtx->globalHeight + 1;

  for ( j = 0, p = (double *) mtx->value, q = (double *) vec->value;
        j < w && j < h; j++, p += jump, q++ )
    *p = *q;

}
/* ------------------------------------------------------------
   zutilities_FortranMatrixSetDiagonal  diag(mtx)=vec   complex
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixSetDiagonal( utilities_FortranMatrix* mtx,
                                     utilities_FortranMatrix* vec ) {

  long j, h, w, jump;
  komplex* p;
  komplex* q;

  BlopexAssert( mtx != NULL && vec != NULL );

  h = mtx->height;
  w = mtx->width;

  BlopexAssert( vec->height >= h );

  jump = mtx->globalHeight + 1;

  for ( j = 0, p = (komplex *) mtx->value, q = (komplex *) vec->value;
        j < w && j < h; j++, p += jump, q++ ) {
    *p = *q;
  }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixGetDiagonal  vec=diag(mtx)     double
   ------------------------------------------------------------ */
void
utilities_FortranMatrixGetDiagonal( utilities_FortranMatrix* mtx,
                                    utilities_FortranMatrix* vec ) {

  long j, h, w, jump;
  double* p;
  double* q;

  BlopexAssert( mtx != NULL && vec != NULL );

  h = mtx->height;
  w = mtx->width;

  BlopexAssert( vec->height >= h );

  jump = mtx->globalHeight + 1;

  for ( j = 0, p = (double *) mtx->value, q = (double *) vec->value;
        j < w && j < h; j++, p += jump, q++ )
    *q = *p;

}
/* ------------------------------------------------------------
   zutilities_FortranMatrixGetDiagonal  vec=diag(mtx)   complex
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixGetDiagonal( utilities_FortranMatrix* mtx,
                                     utilities_FortranMatrix* vec ) {

  long j, h, w, jump;
  komplex* p;
  komplex* q;

  BlopexAssert( mtx != NULL && vec != NULL );

  h = mtx->height;
  w = mtx->width;

  BlopexAssert( vec->height >= h );

  jump = mtx->globalHeight + 1;

  for ( j = 0, p = (komplex *) mtx->value, q = (komplex *) vec->value;
        j < w && j < h; j++, p += jump, q++ ) {
    *q = *p;
  }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixAdd  mtxC=mtxB+a*mtxA          double
   ------------------------------------------------------------ */
void
utilities_FortranMatrixAdd( double a,
                            utilities_FortranMatrix* mtxA,
                            utilities_FortranMatrix* mtxB,
                            utilities_FortranMatrix* mtxC ) {

  long i, j, h, w, jA, jB, jC;
  double *pA;
  double *pB;
  double *pC;

  BlopexAssert( mtxA != NULL && mtxB != NULL && mtxC != NULL );

  h = mtxA->height;
  w = mtxA->width;

  BlopexAssert( mtxB->height == h && mtxB->width == w );
  BlopexAssert( mtxC->height == h && mtxC->width == w );

  jA = mtxA->globalHeight - h;
  jB = mtxB->globalHeight - h;
  jC = mtxC->globalHeight - h;

  pA = (double *) mtxA->value;
  pB = (double *) mtxB->value;
  pC = (double *) mtxC->value;

  if ( a == 0.0 ) {
    for ( j = 0; j < w; j++ ) {
      for ( i = 0; i < h; i++, pA++, pB++, pC++ )
        *pC = *pB;
      pA += jA;
      pB += jB;
      pC += jC;
    }
  }
  else if ( a == 1.0 ) {
    for ( j = 0; j < w; j++ ) {
      for ( i = 0; i < h; i++, pA++, pB++, pC++ )
        *pC = *pA + *pB;
      pA += jA;
      pB += jB;
      pC += jC;
    }
  }
  else if ( a == -1.0 ) {
    for ( j = 0; j < w; j++ ) {
      for ( i = 0; i < h; i++, pA++, pB++, pC++ )
        *pC = *pB - *pA;
      pA += jA;
      pB += jB;
      pC += jC;
    }
  }
  else {
    for ( j = 0; j < w; j++ ) {
      for ( i = 0; i < h; i++, pA++, pB++, pC++ )
        *pC = *pA * a + *pB;
      pA += jA;
      pB += jB;
      pC += jC;
    }
  }
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixAdd  mtxC=mtxB+a*mtxA        complex
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixAdd( double a,
                             utilities_FortranMatrix* mtxA,
                             utilities_FortranMatrix* mtxB,
                             utilities_FortranMatrix* mtxC ) {

  long i, j, h, w, jA, jB, jC;
  komplex *pA;
  komplex *pB;
  komplex *pC;

  BlopexAssert( mtxA != NULL && mtxB != NULL && mtxC != NULL );

  h = mtxA->height;
  w = mtxA->width;

  BlopexAssert( mtxB->height == h && mtxB->width == w );
  BlopexAssert( mtxC->height == h && mtxC->width == w );

  jA = mtxA->globalHeight - h;
  jB = mtxB->globalHeight - h;
  jC = mtxC->globalHeight - h;

  pA = (komplex *) mtxA->value;
  pB = (komplex *) mtxB->value;
  pC = (komplex *) mtxC->value;

  if ( a == 0.0 ) {
    for ( j = 0; j < w; j++ ) {
      for ( i = 0; i < h; i++, pB++, pC++ )
        *pC = *pB;
      pB += jB;
      pC += jC;
    }
  }
  else if ( a == 1.0 ) {
    for ( j = 0; j < w; j++ ) {
      for ( i = 0; i < h; i++, pA++, pB++, pC++ )
        complex_add(pC,pA,pB);
      pA += jA;
      pB += jB;
      pC += jC;
    }
  }
  else if ( a == -1.0 ) {
    for ( j = 0; j < w; j++ ) {
      for ( i = 0; i < h; i++, pA++, pB++, pC++ )
        complex_subtract(pC,pB,pA);
      pA += jA;
      pB += jB;
      pC += jC;
    }
  }
  else {
    for ( j = 0; j < w; j++ ) {
      for ( i = 0; i < h; i++, pA++, pB++, pC++ ) {
        pC->real = pA->real * a + pB->real;  /* real part */
        pC->imag = pA->imag * a + pB->imag;  /* imag part */
      }
      pA += jA;
      pB += jB;
      pC += jC;
    }
  }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixDMultiply   row(mtx)=row(mtx)*vec   double
   ------------------------------------------------------------ */
void
utilities_FortranMatrixDMultiply( utilities_FortranMatrix* vec,
                                  utilities_FortranMatrix* mtx ) {

  long i, j, h, w, jump;
  double* p;
  double* q;

  BlopexAssert( mtx != NULL && vec != NULL );

  h = mtx->height;
  w = mtx->width;

  BlopexAssert( vec->height == h );

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (double *) mtx->value; j < w; j++ ) {
    for ( i = 0, q = (double *) vec->value; i < h; i++, p++, q++ )
      *p = *p * (*q);
    p += jump;
  }
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixDMultiply   row(mtx)=row(mtx)*vec   complex

   not used
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixDMultiply( utilities_FortranMatrix* vec,
                                   utilities_FortranMatrix* mtx ) {

  long i, j, h, w, jump;
  komplex* p;
  komplex* q;

  BlopexAssert( mtx != NULL && vec != NULL );

  h = mtx->height;
  w = mtx->width;

  BlopexAssert( vec->height == h );

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (komplex *) mtx->value; j < w; j++ ) {
    for ( i = 0, q = (komplex *) vec->value; i < h; i++, p++, q++ )
      complex_multiply(p,p,q);
    p += jump;
  }
}

/* ----------------------------------------------------------------
   utilities_FortranMatrixMultiplyD   col(mtx)=col(mtx)*vec  double

   not used
   ---------------------------------------------------------------- */
void
utilities_FortranMatrixMultiplyD( utilities_FortranMatrix* mtx,
                                  utilities_FortranMatrix* vec ) {

  long i, j, h, w, jump;
  double* p;
  double* q;

  BlopexAssert( mtx != NULL && vec != NULL );

  h = mtx->height;
  w = mtx->width;

  BlopexAssert( vec->height == w );

  jump = mtx->globalHeight - h;

  for ( j = 0, q = (double *) vec->value, p = (double *) mtx->value;
        j < w; j++, q++ ) {
    for ( i = 0; i < h; i++, p++)
      *p = *p * (*q);
    p += jump;
  }
}
/* ----------------------------------------------------------------
   zutilities_FortranMatrixMultiplyD  col(mtx)=col(mtx)*vec  complex
   ---------------------------------------------------------------- */
void
zutilities_FortranMatrixMultiplyD( utilities_FortranMatrix* mtx,
                                   utilities_FortranMatrix* vec ) {

  long i, j, h, w, jump;
  komplex* p;
  komplex* q;

  BlopexAssert( mtx != NULL && vec != NULL );

  h = mtx->height;
  w = mtx->width;

  BlopexAssert( vec->height == w );

  jump = mtx->globalHeight - h;

  for ( j = 0, q = (komplex *) vec->value, p = (komplex *)mtx->value;
        j < w; j++, q++ ) {
    for ( i = 0; i < h; i++, p++)
      complex_multiply(p,p,q);
    p += jump;
  }
}
/* ------------------------------------------------------------
   utilities_FortranMatrixMultiply                       double
                                  ta  tb
                                  0   0   C=A*B
                                  0   1   C=A*B'
                                  1   0   C=A'*B
                                  1   1   C=A'*B'
   ------------------------------------------------------------ */
void
utilities_FortranMatrixMultiply( utilities_FortranMatrix* mtxA, BlopexInt tA,
                                 utilities_FortranMatrix* mtxB, BlopexInt tB,
                                 utilities_FortranMatrix* mtxC ) {
  long h, w;
  long i, j, k, l;
  long iA, kA;
  long kB, jB;
  long iC, jC;

  double* pAi0;
  double* pAik;
  double* pB0j;
  double* pBkj;
  double* pC0j;
  double* pCij;

  double s;

  BlopexAssert( mtxA != NULL && mtxB != NULL && mtxC != NULL );

  h = mtxC->height;
  w = mtxC->width;
  iC = 1;
  jC = mtxC->globalHeight;

  if ( tA == 0 ) {
    BlopexAssert( mtxA->height == h );
    l = mtxA->width;
    iA = 1;
    kA = mtxA->globalHeight;
  }
  else {
    l = mtxA->height;
    BlopexAssert( mtxA->width == h );
    kA = 1;
    iA = mtxA->globalHeight;
  }

  if ( tB == 0 ) {
    BlopexAssert( mtxB->height == l );
    BlopexAssert( mtxB->width == w );
    kB = 1;
    jB = mtxB->globalHeight;
  }
  else {
    BlopexAssert( mtxB->width == l );
    BlopexAssert( mtxB->height == w );
    jB = 1;
    kB = mtxB->globalHeight;
  }

  for ( j = 0, pB0j = (double *) mtxB->value, pC0j = (double *) mtxC->value;
        j < w; j++, pB0j += jB, pC0j += jC  )
    for ( i = 0, pCij = pC0j, pAi0 = (double *) mtxA->value;
          i < h; i++, pCij += iC, pAi0 += iA ) {
      s = 0.0;
      for ( k = 0, pAik = pAi0, pBkj = pB0j; k < l;
            k++, pAik += kA, pBkj += kB )
        s += *pAik * (*pBkj);
      *pCij = s;
    }
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixMultiply                     complex
                                  ta  tb
                                  0   0   C=A*B
                                  0   1   C=A*B'
                                  1   0   C=A'*B
                                  1   1   C=A'*B'
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixMultiply( utilities_FortranMatrix* mtxA, BlopexInt tA,
                                  utilities_FortranMatrix* mtxB, BlopexInt tB,
                                  utilities_FortranMatrix* mtxC ) {
  long h, w;
  long i, j, k, l;
  long iA, kA;
  long kB, jB;
  long iC, jC;

  komplex* pAi0;
  komplex* pAik;
  komplex* pB0j;
  komplex* pBkj;
  komplex* pC0j;
  komplex* pCij;

  komplex s;
  komplex t;
  komplex ta;
  komplex tb;


  BlopexAssert( mtxA != NULL && mtxB != NULL && mtxC != NULL );

  h = mtxC->height;
  w = mtxC->width;
  iC = 1;
  jC = mtxC->globalHeight;

  if ( tA == 0 ) {
    BlopexAssert( mtxA->height == h );
    l = mtxA->width;
    iA = 1;
    kA = mtxA->globalHeight;
  }
  else {
    l = mtxA->height;
    BlopexAssert( mtxA->width == h );
    kA = 1;
    iA = mtxA->globalHeight;
  }

  if ( tB == 0 ) {
    BlopexAssert( mtxB->height == l );
    BlopexAssert( mtxB->width == w );
    kB = 1;
    jB = mtxB->globalHeight;
  }
  else {
    BlopexAssert( mtxB->width == l );
    BlopexAssert( mtxB->height == w );
    jB = 1;
    kB = mtxB->globalHeight;
  }

  for ( j = 0, pB0j = (komplex *) mtxB->value, pC0j = (komplex *) mtxC->value;
        j < w; j++, pB0j += jB, pC0j += jC  )
    for ( i = 0, pCij = pC0j, pAi0 = (komplex *) mtxA->value;
          i < h; i++, pCij += iC, pAi0 += iA ) {
      s.real = 0.0;
      s.imag = 0.0;
      for ( k = 0, pAik = pAi0, pBkj = pB0j; k < l;
            k++, pAik += kA, pBkj += kB ) {
        ta = *pAik;
        if ( tA != 0 )
          ta.imag = -(ta.imag);
        tb = *pBkj;
        if ( tB != 0 )
          tb.imag = -(tb.imag);
        complex_multiply(&t, &ta, &tb);
        complex_add(&s, &s, &t);
      }
      *pCij = s;
    }
}
/* ---------------------------------------------------------------
   utilities_FortranMatrixFNorm  Frobenius norm  not used   double
   --------------------------------------------------------------- */
double
utilities_FortranMatrixFNorm( utilities_FortranMatrix* mtx ) {

  long i, j, h, w, jump;
  double* p;

  double norm;

  BlopexAssert( mtx != NULL );

  h = mtx->height;
  w = mtx->width;

  jump = mtx->globalHeight - h;

  norm = 0.0;

  for ( j = 0, p = (double *)mtx->value; j < w; j++ ) {
    for ( i = 0; i < h; i++, p++ )
      norm += (*p) * (*p);
    p += jump;
  }

  norm = sqrt(norm);
  return norm;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixAbs     ret=abs(mtx(i,j))      double
                                  always returns double
   ------------------------------------------------------------ */
double
utilities_FortranMatrixAbs( utilities_FortranMatrix* mtx,
                            long i, long j ) {

  long k;
  double* p;

  BlopexAssert( mtx != NULL );

  BlopexAssert( 1 <= i && i <= mtx->height );
  BlopexAssert( 1 <= j && j <= mtx->width );

  k = i - 1 + (j - 1)*mtx->globalHeight;

  p=(double *)mtx->value;
  return p[k];
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixAbs     ret=abs(mtx(i,j))    complex
                                  always returns double
   ------------------------------------------------------------ */
double
zutilities_FortranMatrixAbs( utilities_FortranMatrix* mtx,
                             long i, long j ) {

  long k;
  komplex* p;
  double pabs;

  BlopexAssert( mtx != NULL );

  BlopexAssert( 1 <= i && i <= mtx->height );
  BlopexAssert( 1 <= j && j <= mtx->width );

  k = i - 1 + (j - 1)*mtx->globalHeight;

  p=(komplex *)mtx->value;
  pabs = p[k].real * p[k].real + p[k].imag * p[k].imag;
  pabs = sqrt( pabs );
  return pabs;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixValuePtr   ret=&mtx(i,j)       double
   ------------------------------------------------------------ */
void*
utilities_FortranMatrixValuePtr( utilities_FortranMatrix* mtx,
                                 long i, long j ) {

  long k;
  double *p;

  BlopexAssert( mtx != NULL );

  BlopexAssert( 1 <= i && i <= mtx->height );
  BlopexAssert( 1 <= j && j <= mtx->width );

  p = (double *)mtx->value;

  k = i - 1 + (j - 1)*mtx->globalHeight;
  return p + k;
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixValuePtr   ret=&mtx(i,j)     complex
   ------------------------------------------------------------ */
void*
zutilities_FortranMatrixValuePtr( utilities_FortranMatrix* mtx,
                                  long i, long j ) {

  long k;
  komplex *p;

  BlopexAssert( mtx != NULL );

  BlopexAssert( 1 <= i && i <= mtx->height );
  BlopexAssert( 1 <= j && j <= mtx->width );

  p = (komplex *)mtx->value;
  k = i - 1 + (j - 1)*mtx->globalHeight;

  return p+k;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixMaxValue   ret=max(mtx)        double
                                     only called with double!
   ------------------------------------------------------------ */
double
utilities_FortranMatrixMaxValue( utilities_FortranMatrix* mtx ) {

  long i, j, jump;
  long h, w;
  double* p;
  double maxVal;

  BlopexAssert( mtx != NULL );

  h = mtx->height;
  w = mtx->width;

  jump = mtx->globalHeight - h;

  p = (double *) mtx->value;
  maxVal = p[0];

  for ( j = 0, p = (double *)mtx->value; j < w; j++ ) {
    for ( i = 0; i < h; i++, p++ )
      if ( *p > maxVal )
    maxVal = *p;
    p += jump;
  }

  return maxVal;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixSelectBlock                    double
                           block=mtx(iFrom:iTo,jFrom:jTo)
   ------------------------------------------------------------ */
void
utilities_FortranMatrixSelectBlock( utilities_FortranMatrix* mtx,
                                    long iFrom, long iTo,
                                    long jFrom, long jTo,
                                    utilities_FortranMatrix* block ) {

  double* p;

  if ( block->value != NULL && block->ownsValues )
    free( block->value );

  block->globalHeight = mtx->globalHeight;
  if ( iTo < iFrom || jTo < jFrom ) {
    block->height = 0;
    block->width = 0;
    block->value = NULL;
    return;
  }
  block->height = iTo - iFrom + 1;
  block->width = jTo - jFrom + 1;
  p = (double *) mtx->value;
  block->value = p + iFrom - 1 + (jFrom - 1)*mtx->globalHeight;
  block->ownsValues = 0;
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixSelectBlock                complex
                           block=mtx(iFrom:iTo,jFrom:jTo)
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixSelectBlock( utilities_FortranMatrix* mtx,
                                     long iFrom, long iTo,
                                     long jFrom, long jTo,
                                     utilities_FortranMatrix* block ) {

  komplex* p;

  if ( block->value != NULL && block->ownsValues )
    free( block->value );

  block->globalHeight = mtx->globalHeight;
  if ( iTo < iFrom || jTo < jFrom ) {
    block->height = 0;
    block->width = 0;
    block->value = NULL;
    return;
  }
  block->height = iTo - iFrom + 1;
  block->width = jTo - jFrom + 1;
  p = (komplex *) mtx->value;
  block->value = p + iFrom - 1 + (jFrom - 1)*mtx->globalHeight;
  block->ownsValues = 0;
}
/* ------------------------------------------------------------
   utilities_FortranMatrixUpperInv                       double
                    invert an upper triangular matrix
   ------------------------------------------------------------ */
void
utilities_FortranMatrixUpperInv( utilities_FortranMatrix* u ) {

  long i, j, k;
  long n, jc, jd;
  double v;
  double* diag; /* diag(i) = u(i,i)_original */
  double* pin;  /* &u(i-1,n) */
  double* pii;  /* &u(i,i) */
  double* pij;  /* &u(i,j) */
  double* pik;  /* &u(i,k) */
  double* pkj;  /* &u(k,j) */
  double* pd;   /* &diag(i) */

  n = u->height;
  BlopexAssert( u->width == n );

  diag = (double*)calloc( n, sizeof(double) );
  BlopexAssert( diag != NULL );

  jc = u->globalHeight;
  jd = jc + 1;

  pii = (double *) u->value;
  pd = diag;
  for ( i = 0; i < n; i++, pii += jd, pd++ ) {
    v = *pd = *pii;
    *pii = 1.0/v;
  }

  pii -= jd;
  pin = pii - 1;
  pii -= jd;
  pd -= 2;
  for ( i = n - 1; i > 0; i--, pii -= jd, pin--, pd-- ) {
    pij = pin;
    for ( j = n; j > i; j--, pij -= jc ) {
      v = 0;
      pik = pii + jc;
      pkj = pij + 1;
      for ( k = i + 1; k <= j; k++, pik += jc, pkj++  ) {
         v -= (*pik) * (*pkj);
      }
      *pij = v/(*pd);
    }
  }

  free( diag );

}
/* ------------------------------------------------------------
   zutilities_FortranMatrixUpperInv                    complex
                    invert an upper triangular matrix
   ------------------------------------------------------------ */
void
zutilities_FortranMatrixUpperInv( utilities_FortranMatrix* u ) {

  long i, j, k;
  long n, jc, jd;
  komplex v;
  komplex w;
  komplex* diag; /* diag(i) = u(i,i)_original */
  komplex* pin;  /* &u(i-1,n) */
  komplex* pii;  /* &u(i,i) */
  komplex* pij;  /* &u(i,j) */
  komplex* pik;  /* &u(i,k) */
  komplex* pkj;  /* &u(k,j) */
  komplex* pd;   /* &diag(i) */

  komplex one = {1.0, 0.0};

  n = u->height;
  BlopexAssert( u->width == n );

  diag = (komplex*)calloc( n, sizeof(komplex) );
  BlopexAssert( diag != NULL );

  jc = u->globalHeight;
  jd = jc + 1;

  pii = (komplex *) u->value;
  pd = diag;
  for ( i = 0; i < n; i++, pii += jd, pd++ ) {
    v = *pd = *pii;
    complex_divide(pii,&one,&v);
  }

  pii -= jd;
  pin = pii - 1;
  pii -= jd;
  pd -= 2;
  for ( i = n - 1; i > 0; i--, pii -= jd, pin--, pd-- ) {
    pij = pin;
    for ( j = n; j > i; j--, pij -= jc ) {
      v.real = 0.0;
      v.imag = 0.0;
      pik = pii + jc;
      pkj = pij + 1;
      for ( k = i + 1; k <= j; k++, pik += jc, pkj++ ) {
        complex_multiply(&w,pik,pkj);
        complex_subtract(&v,&v,&w);
      }
      complex_divide(pij,&v,pd);
    }
  }

  free( diag );

}
/* ------------------------------------------------------------
   utilities_FortranMatrixPrint                          double
   ------------------------------------------------------------ */
BlopexInt
utilities_FortranMatrixPrint( utilities_FortranMatrix* mtx,
                              char fileName[] )
{
  long i, j, h, w, jump;
  double* p;
  FILE* fp;

  BlopexAssert( mtx != NULL );

  if ( !(fp = fopen(fileName,"w")) )
    return 1;

  h = mtx->height;
  w = mtx->width;

  fprintf(fp,"%ld\n",h);
  fprintf(fp,"%ld\n",w);

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (double *) mtx->value; j < w; j++ ) {
    for ( i = 0; i < h; i++, p++ )
      fprintf(fp,"%22.14e\n",*p);
    p += jump;
  }

  fclose(fp);
  return 0;
}
/* ------------------------------------------------------------
   zutilities_FortranMatrixPrint                       complex
   ------------------------------------------------------------ */
BlopexInt
zutilities_FortranMatrixPrint( utilities_FortranMatrix* mtx,
                               char fileName[] )
{
  long i, j, h, w, jump;
  komplex* p;
  FILE* fp;

  BlopexAssert( mtx != NULL );

  if ( !(fp = fopen(fileName,"w")) )
    return 1;

  h = mtx->height;
  w = mtx->width;

  fprintf(fp,"%ld\n",h);
  fprintf(fp,"%ld\n",w);

  jump = mtx->globalHeight - h;

  for ( j = 0, p = (komplex *) mtx->value; j < w; j++ ) {
    for ( i = 0; i < h; i++, p++ )
      fprintf(fp,"%22.14e  %22.14e \n",p->real,p->imag);
    p += jump;
  }

  return 0;
}
