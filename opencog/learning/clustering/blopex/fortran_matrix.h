/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

#include "fortran_options.h"

#ifndef FORTRAN_STYLE_MATRIX
#define FORTRAN_STYLE_MATRIX

typedef struct
{
  long  globalHeight;
  long  height;
  long  width;
  void* value;
  BlopexInt   ownsValues;
} utilities_FortranMatrix;

#ifdef __cplusplus
extern "C" {
#endif

/* ----------- complex definition ----------- */

typedef struct {double real, imag;} komplex;


/*  ---------- generic routines ----------- */

utilities_FortranMatrix*
utilities_FortranMatrixCreate(void);
void
utilities_FortranMatrixWrap( void*, long gh, long h, long w,
                 utilities_FortranMatrix* mtx );
void
utilities_FortranMatrixDestroy( utilities_FortranMatrix* mtx );
long
utilities_FortranMatrixGlobalHeight( utilities_FortranMatrix* mtx );
long
utilities_FortranMatrixHeight( utilities_FortranMatrix* mtx );
long
utilities_FortranMatrixWidth( utilities_FortranMatrix* mtx );
void*
utilities_FortranMatrixValues( utilities_FortranMatrix* mtx );
double
utilities_FortranMatrixFNorm( utilities_FortranMatrix* mtx );

/* ------------ double routines ------------- */

void
utilities_FortranMatrixAllocateData( long h, long w,
                     utilities_FortranMatrix* mtx );
void
utilities_FortranMatrixClear( utilities_FortranMatrix* mtx );
void
utilities_FortranMatrixClearL( utilities_FortranMatrix* mtx );
void
utilities_FortranMatrixSetToIdentity( utilities_FortranMatrix* mtx );
void
utilities_FortranMatrixTransposeSquare( utilities_FortranMatrix* mtx );
void
utilities_FortranMatrixSymmetrize( utilities_FortranMatrix* mtx );
void
utilities_FortranMatrixCopy( utilities_FortranMatrix* src, BlopexInt t,
                             utilities_FortranMatrix* dest );
void
utilities_FortranMatrixIndexCopy( BlopexInt* index,
                  utilities_FortranMatrix* src, BlopexInt t,
                  utilities_FortranMatrix* dest );
void
utilities_FortranMatrixSetDiagonal( utilities_FortranMatrix* mtx,
                    utilities_FortranMatrix* d );
void
utilities_FortranMatrixGetDiagonal( utilities_FortranMatrix* mtx,
                    utilities_FortranMatrix* d );
void
utilities_FortranMatrixAdd( double a,
                utilities_FortranMatrix* mtxA,
                utilities_FortranMatrix* mtxB,
                utilities_FortranMatrix* mtxC );
void
utilities_FortranMatrixDMultiply( utilities_FortranMatrix* d,
                  utilities_FortranMatrix* mtx );
void
utilities_FortranMatrixMultiplyD( utilities_FortranMatrix* mtx,
                  utilities_FortranMatrix* d );
void
utilities_FortranMatrixMultiply( utilities_FortranMatrix* mtxA, BlopexInt tA,
                 utilities_FortranMatrix* mtxB, BlopexInt tB,
                 utilities_FortranMatrix* mtxC );

double
utilities_FortranMatrixAbs( utilities_FortranMatrix* mtx,
                            long i, long j );
void*
utilities_FortranMatrixValuePtr( utilities_FortranMatrix* mtx,
                 long i, long j );
double
utilities_FortranMatrixMaxValue( utilities_FortranMatrix* mtx );
void
utilities_FortranMatrixSelectBlock( utilities_FortranMatrix* mtx,
                    long iFrom, long iTo,
                    long jFrom, long jTo,
                    utilities_FortranMatrix* block );
void
utilities_FortranMatrixUpperInv( utilities_FortranMatrix* u );
BlopexInt
utilities_FortranMatrixPrint( utilities_FortranMatrix* mtx, char fileName[] );

/* ------------ complex routines ------------- */

void
zutilities_FortranMatrixAllocateData( long h, long w,
                     utilities_FortranMatrix* mtx );
void
zutilities_FortranMatrixClear( utilities_FortranMatrix* mtx );
void
zutilities_FortranMatrixClearL( utilities_FortranMatrix* mtx );
void
zutilities_FortranMatrixSetToIdentity( utilities_FortranMatrix* mtx );
void
zutilities_FortranMatrixTransposeSquare( utilities_FortranMatrix* mtx );
void
zutilities_FortranMatrixSymmetrize( utilities_FortranMatrix* mtx );
void
zutilities_FortranMatrixCopy( utilities_FortranMatrix* src, BlopexInt t,
                             utilities_FortranMatrix* dest );
void
zutilities_FortranMatrixIndexCopy( BlopexInt* index,
                  utilities_FortranMatrix* src, BlopexInt t,
                  utilities_FortranMatrix* dest );
void
zutilities_FortranMatrixSetDiagonal( utilities_FortranMatrix* mtx,
                    utilities_FortranMatrix* d );
void
zutilities_FortranMatrixGetDiagonal( utilities_FortranMatrix* mtx,
                    utilities_FortranMatrix* d );
void
zutilities_FortranMatrixAdd( double a,
                utilities_FortranMatrix* mtxA,
                utilities_FortranMatrix* mtxB,
                utilities_FortranMatrix* mtxC );
void
zutilities_FortranMatrixDMultiply( utilities_FortranMatrix* d,
                  utilities_FortranMatrix* mtx );
void
zutilities_FortranMatrixMultiplyD( utilities_FortranMatrix* mtx,
                  utilities_FortranMatrix* d );
void
zutilities_FortranMatrixMultiply( utilities_FortranMatrix* mtxA, BlopexInt tA,
                 utilities_FortranMatrix* mtxB, BlopexInt tB,
                 utilities_FortranMatrix* mtxC );

double
zutilities_FortranMatrixAbs( utilities_FortranMatrix* mtx,
                            long i, long j );
void*
zutilities_FortranMatrixValuePtr( utilities_FortranMatrix* mtx,
                 long i, long j );
void
zutilities_FortranMatrixSelectBlock( utilities_FortranMatrix* mtx,
                    long iFrom, long iTo,
                    long jFrom, long jTo,
                    utilities_FortranMatrix* block );
void
zutilities_FortranMatrixUpperInv( utilities_FortranMatrix* u );
BlopexInt
zutilities_FortranMatrixPrint( utilities_FortranMatrix* mtx, char fileName[] );

#ifdef __cplusplus
}
#endif

#endif /* FORTRAN_STYLE_MATRIX */
