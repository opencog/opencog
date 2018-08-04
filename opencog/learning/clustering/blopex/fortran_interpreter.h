/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

#ifndef LOBPCG_FORTRAN_INTERPRETER_HEADER
#define LOBPCG_FORTRAN_INTERPRETER_HEADER

#include "fortran_matrix.h"

typedef struct
{
  utilities_FortranMatrix* (*FortranMatrixCreate)  ( void );
  void (*FortranMatrixAllocateData) ( long, long, utilities_FortranMatrix* );
  void (*FortranMatrixWrap) ( void* , long, long, long, utilities_FortranMatrix* );
  void (*FortranMatrixDestroy) ( utilities_FortranMatrix* );
  long (*FortranMatrixGlobalHeight) ( utilities_FortranMatrix* );
  long (*FortranMatrixHeight) ( utilities_FortranMatrix* );
  long (*FortranMatrixWidth) ( utilities_FortranMatrix* );
  void* (*FortranMatrixValues) ( utilities_FortranMatrix* );
  void (*FortranMatrixClear) ( utilities_FortranMatrix* );
  void (*FortranMatrixClearL) ( utilities_FortranMatrix* );
  void (*FortranMatrixSetToIdentity) ( utilities_FortranMatrix* );
  void (*FortranMatrixTransposeSquare) ( utilities_FortranMatrix* );
  void (*FortranMatrixSymmetrize) ( utilities_FortranMatrix* );
  void (*FortranMatrixCopy) ( utilities_FortranMatrix* src, BlopexInt,
                              utilities_FortranMatrix* dest);
  void (*FortranMatrixIndexCopy) ( BlopexInt* index,
                                   utilities_FortranMatrix* src, BlopexInt,
                                   utilities_FortranMatrix* dest);
  void (*FortranMatrixSetDiagonal) ( utilities_FortranMatrix* mtx ,
                                     utilities_FortranMatrix* d );
  void (*FortranMatrixGetDiagonal) ( utilities_FortranMatrix* mtx,
                                     utilities_FortranMatrix* d );
  void (*FortranMatrixAdd) ( double a, utilities_FortranMatrix* mtxA,
                                       utilities_FortranMatrix* mtxB,
                                       utilities_FortranMatrix* mtxC);
  void (*FortranMatrixDMultiply) ( utilities_FortranMatrix* d,
                                   utilities_FortranMatrix* mtx);
  void (*FortranMatrixMultiplyD) ( utilities_FortranMatrix* mtx,
                                   utilities_FortranMatrix* d);
  void (*FortranMatrixMultiply) ( utilities_FortranMatrix* mtxA, BlopexInt tA,
                                  utilities_FortranMatrix* mtxB, BlopexInt tB,
                                  utilities_FortranMatrix* mtxC );
  double (*FortranMatrixFNorm) ( utilities_FortranMatrix* mtx );
  double (*FortranMatrixAbs) ( utilities_FortranMatrix* mtx, long i, long j );

  void* (*FortranMatrixValuePtr) ( utilities_FortranMatrix* mtx, long i, long j );
  double (*FortranMatrixMaxValue) ( utilities_FortranMatrix* mtx );
  void (*FortranMatrixSelectBlock) ( utilities_FortranMatrix* mtx,
                                     long iFrom, long iTo, long jFrom, long jTo,
                                     utilities_FortranMatrix* block);
  void (*FortranMatrixUpperInv) ( utilities_FortranMatrix* u );
  BlopexInt (*FortranMatrixPrint) ( utilities_FortranMatrix* mtx, char fileName[] );

} utilities_FortranInterpreter;

#endif /* LOBPCG_FORTRAN_INTERPRETER_HEADER */
