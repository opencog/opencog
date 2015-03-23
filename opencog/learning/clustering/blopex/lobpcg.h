/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

#ifndef LOCALLY_OPTIMAL_BLOCK_PRECONDITIONED_CONJUGATE_GRADIENTS
#define LOCALLY_OPTIMAL_BLOCK_PRECONDITIONED_CONJUGATE_GRADIENTS

#include "multivector.h"
#include "fortran_interpreter.h"
#include "fortran_matrix.h"       /* we use "komplex" type defined there */

#define PROBLEM_SIZE_TOO_SMALL                  1
#define WRONG_BLOCK_SIZE                    2
#define WRONG_CONSTRAINTS                               3
#define REQUESTED_ACCURACY_NOT_ACHIEVED         -1

typedef struct {

  double    absolute;
  double    relative;

} lobpcg_Tolerance;

typedef struct {

/* these pointers should point to 2 functions providing standard lapack  functionality */

/* double precision */

   BlopexInt (*dpotrf) (char *uplo, BlopexInt *n,
                  double *a, BlopexInt * lda, BlopexInt *info);
   BlopexInt (*dsygv) (BlopexInt *itype, char *jobz, char *uplo, BlopexInt * n,
                 double *a, BlopexInt *lda, double *b, BlopexInt *ldb,
                 double *w, double *work, BlopexInt *lwork, BlopexInt *info);

/* komplex (double precision complex)*/

   BlopexInt (*zpotrf) (char *uplo, BlopexInt *n,
                  komplex *a, BlopexInt *lda, BlopexInt *info);
   BlopexInt (*zhegv) (BlopexInt *itype, char *jobz, char *uplo, BlopexInt * n,
                 komplex *a, BlopexInt *lda, komplex *b, BlopexInt *ldb,
                 double *w, komplex *work, BlopexInt *lwork,
                 double * rwork,BlopexInt *info);

} lobpcg_BLASLAPACKFunctions;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
BlopexInt
(*chol)( utilities_FortranMatrix* a,
         lobpcg_BLASLAPACKFunctions blap_fn,
         utilities_FortranInterpreter* util );

BlopexInt
(*solveGEVP)( utilities_FortranMatrix* mtxA,
              utilities_FortranMatrix* mtxB,
              utilities_FortranMatrix* eigVal,
              lobpcg_BLASLAPACKFunctions blap_fn,
              utilities_FortranInterpreter* util);

} lobpcg_Interpreter;

int
lobpcg_solve_double(
          mv_MultiVectorPtr blockVectorX,
          void* operatorAData,
          void (*operatorA)( void*, void*, void* ),
          void* operatorBData,
          void (*operatorB)( void*, void*, void* ),
          void* operatorTData,
          void (*operatorT)( void*, void*, void* ),
          mv_MultiVectorPtr blockVectorY,
          lobpcg_BLASLAPACKFunctions blap_fn,
          lobpcg_Tolerance tolerance,
          int maxIterations,
          int verbosityLevel,
          int* iterationNumber,

/* eigenvalues; "lambda_values" should point to array  containing <blocksize> doubles where
   <blocksize> is the width of multivector "blockVectorX" */

          double * lambda_values,

/* eigenvalues history; a pointer to the entries of the  <blocksize>-by-(<maxIterations>+1) matrix
   stored in  fortran-style. (i.e. column-wise) The matrix may be  a submatrix of a larger matrix,
   see next argument; If you don't need eigenvalues history, provide NULL in this entry */

          double * lambdaHistory_values,

/* global height of the matrix (stored in fotran-style)  specified by previous argument */

          BlopexInt lambdaHistory_gh,

/* residual norms; argument should point to array of <blocksize> doubles */

          double * residualNorms_values,

/* residual norms history; a pointer to the entries of the  <blocksize>-by-(<maxIterations>+1)
   matrix stored in  fortran-style. (i.e. column-wise) The matrix may be a submatrix of a
   larger matrix, see next argument If you don't need residual norms history, provide NULL in this entry */

          double * residualNormsHistory_values ,

/* global height of the matrix (stored in fotran-style)  specified by previous argument */

          BlopexInt residualNormsHistory_gh

);

int
lobpcg_solve_complex(
          mv_MultiVectorPtr blockVectorX,
          void* operatorAData,
          void (*operatorA)( void*, void*, void* ),
          void* operatorBData,
          void (*operatorB)( void*, void*, void* ),
          void* operatorTData,
          void (*operatorT)( void*, void*, void* ),
          mv_MultiVectorPtr blockVectorY,

          lobpcg_BLASLAPACKFunctions blap_fn,
          lobpcg_Tolerance tolerance,
          int maxIterations,
          int verbosityLevel,
          int* iterationNumber,

          komplex * lambda_values,
          komplex * lambdaHistory_values,
          BlopexInt lambdaHistory_gh,

          double * residualNorms_values,
          double * residualNormsHistory_values ,

          BlopexInt residualNormsHistory_gh
);
int
lobpcg_solve(
          mv_MultiVectorPtr blockVectorX,
          void* operatorAData,
          void (*operatorA)( void*, void*, void* ),
          void* operatorBData,
          void (*operatorB)( void*, void*, void* ),
          void* operatorTData,
          void (*operatorT)( void*, void*, void* ),
          mv_MultiVectorPtr blockVectorY,

          lobpcg_BLASLAPACKFunctions blap_fn,
          lobpcg_Tolerance tolerance,
          int maxIterations,
          int verbosityLevel,
          int* iterationNumber,

          void * lambda_values,
          void * lambdaHistory_values,
          BlopexInt lambdaHistory_gh,

          double * residualNorms_values,
          double * residualNormsHistory_values ,
          BlopexInt residualNormsHistory_gh,

          lobpcg_Interpreter *lobpcg,
          utilities_FortranInterpreter *util
);


#ifdef __cplusplus
}
#endif

#endif /* LOCALLY_OPTIMAL_BLOCK_PRECONDITIONED_CONJUGATE_GRADIENTS */
