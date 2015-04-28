/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "fortran_matrix.h"
#include "fortran_interpreter.h"
#include "lobpcg.h"

/* ------------------------------------------------------------
   lobpcg_chol       chol fact of a ie a=u'*u            double
                     replaces a with u
   ------------------------------------------------------------ */
static BlopexInt
lobpcg_chol( utilities_FortranMatrix* a,
             lobpcg_BLASLAPACKFunctions blap_fn,
             utilities_FortranInterpreter* util )
{

  BlopexInt lda, n;
  double* aval;
  char uplo;
  BlopexInt ierr;

  lda = (util->FortranMatrixGlobalHeight) ( a );
  n = (util->FortranMatrixHeight) ( a );
  aval = (double *) (util->FortranMatrixValues) ( a );
  uplo = 'U';

  blap_fn.dpotrf ( &uplo, &n, aval, &lda, &ierr );

  return ierr;
}
/* ------------------------------------------------------------
   zlobpcg_chol                                         complex
   ------------------------------------------------------------ */
static BlopexInt
zlobpcg_chol( utilities_FortranMatrix* a,
              lobpcg_BLASLAPACKFunctions blap_fn,
              utilities_FortranInterpreter* util )
{

  BlopexInt lda, n;
  komplex* aval;
  char uplo;
  BlopexInt ierr;

  lda = (util->FortranMatrixGlobalHeight) ( a );
  n = (util->FortranMatrixHeight) ( a );
  aval = (komplex *) (util->FortranMatrixValues) ( a );
  uplo = 'U';

  blap_fn.zpotrf( &uplo, &n, aval, &lda, &ierr );

  return ierr;
}
/* ------------------------------------------------------------
   lobpcg_solveGEVP                                      double
   ------------------------------------------------------------ */
static BlopexInt
lobpcg_solveGEVP( utilities_FortranMatrix* mtxA,
                  utilities_FortranMatrix* mtxB,
                  utilities_FortranMatrix* eigVal,
                  lobpcg_BLASLAPACKFunctions blap_fn,
                  utilities_FortranInterpreter* util
){

  BlopexInt n, lda, ldb, itype, lwork, info;
  char jobz, uplo;
  double* work;
  double* a;
  double* b;
  double* lmd;

  itype = 1;
  jobz = 'V';
  uplo = 'L';

  a = (double *) (util->FortranMatrixValues)( mtxA );
  b = (double *) (util->FortranMatrixValues)( mtxB );
  lmd = (double *) (util->FortranMatrixValues)( eigVal );

  n = (util->FortranMatrixHeight)( mtxA );
  lda = (util->FortranMatrixGlobalHeight)( mtxA );
  ldb = (util->FortranMatrixGlobalHeight)( mtxB );
  lwork = 10*n;

  work = (double*)calloc( lwork, sizeof(double) );

  blap_fn.dsygv( &itype, &jobz, &uplo, &n,
                  a, &lda, b, &ldb,
                  lmd, &work[0], &lwork, &info );

  free( work );
  return info;

}
/* ------------------------------------------------------------
   zlobpcg_solveGEVP                                    complex
                     find eig of A*x=(lambda)*B*x
                     mtxA is Hermitian (contains e-vec on ret)
                     mtxB is Hermitian, Pos Def
   ------------------------------------------------------------ */
static BlopexInt
zlobpcg_solveGEVP( utilities_FortranMatrix* mtxA,
                   utilities_FortranMatrix* mtxB,
                   utilities_FortranMatrix* eigVal,
                   lobpcg_BLASLAPACKFunctions blap_fn,
                   utilities_FortranInterpreter* util )
{

  BlopexInt i;
  BlopexInt n, lda, ldb, itype, lwork, info;
  char jobz, uplo;
  komplex* work;
  komplex* a;
  komplex* b;
  double* w;
  komplex * lmd;
  double * rwork;

  itype = 1;
  jobz = 'V';
  uplo = 'L';

  a = (komplex *) (util->FortranMatrixValues)( mtxA );
  b = (komplex *) (util->FortranMatrixValues)( mtxB );

  n = (util->FortranMatrixHeight)( mtxA );
  lda = (util->FortranMatrixGlobalHeight)( mtxA );
  ldb = (util->FortranMatrixGlobalHeight)( mtxB );

  lwork = 10*n;
  work = (komplex*)calloc( lwork, sizeof(komplex) );
  w = (double *)calloc(n,sizeof(double));
  rwork = (double *)calloc(3*n,sizeof(double));

  blap_fn.zhegv( &itype, &jobz, &uplo, &n,
                 a, &lda, b, &ldb,
                 w, &work[0], &lwork, rwork, &info );

  lmd = (komplex *) (util->FortranMatrixValues)( eigVal );
  for (i=0;i<n;i++) {
    lmd[i].real = w[i];
    lmd[i].imag = 0.0;
  }

  free( work );
  free( w );
  free( rwork );
  return info;

}
/* ------------------------------------------------------------
   lobpcg_MultiVectorByMultiVector    xy=x'*y           generic
   ------------------------------------------------------------ */
static void
lobpcg_MultiVectorByMultiVector(
              mv_MultiVectorPtr x,
              mv_MultiVectorPtr y,
              utilities_FortranMatrix* xy,
              utilities_FortranInterpreter* util )
{
  mv_MultiVectorByMultiVector( x, y,
                  (util->FortranMatrixGlobalHeight)( xy ),
                  (util->FortranMatrixHeight)( xy ),
                  (util->FortranMatrixWidth)( xy ),
                  (util->FortranMatrixValues)( xy ) );
}
/* ------------------------------------------------------------
   lobpcg_MultiVectorByMatrix   y=x*r                   generic
   ------------------------------------------------------------ */
static void
lobpcg_MultiVectorByMatrix(
             mv_MultiVectorPtr x,
             utilities_FortranMatrix* r,
             mv_MultiVectorPtr y,
             utilities_FortranInterpreter* util )
{
  mv_MultiVectorByMatrix( x,
                 (util->FortranMatrixGlobalHeight)( r ),
                 (util->FortranMatrixHeight)( r ),
                 (util->FortranMatrixWidth)( r ),
                 (util->FortranMatrixValues)( r ),
                 y );
}
/* ------------------------------------------------------------
   lobpcg_MultiVectorImplicitQR                         generic
   orthonormalize x
   ------------------------------------------------------------ */
static BlopexInt
lobpcg_MultiVectorImplicitQR(
       mv_MultiVectorPtr x, mv_MultiVectorPtr y,
       utilities_FortranMatrix* r,
       mv_MultiVectorPtr z,
       lobpcg_BLASLAPACKFunctions blap_fn,
       lobpcg_Interpreter* lobpcg,
       utilities_FortranInterpreter* util )
{

  /* B-orthonormalizes x using y = B x */

  BlopexInt ierr;

  /*--- r = x'*y                   */
  lobpcg_MultiVectorByMultiVector( x, y, r, util );

 /*--- r = u in chol fact r=u'*u  */
  ierr = (lobpcg->chol)( r,blap_fn, util );
  if ( ierr != 0 )
    return ierr;

  /*--- r = inverse(r)  */
  (util->FortranMatrixUpperInv)( r );

  /*--- lower(r)=0      */
  (util->FortranMatrixClearL)( r );

  /*--- z = x           */
  mv_MultiVectorCopy( x, z );

  /*--- x = z * r       */
  lobpcg_MultiVectorByMatrix( z, r, x, util );

  return 0;
}

/* ------------------------------------------------------------
   lobpcg_ComputeResidualNorms                          generic
   ------------------------------------------------------------ */
static void
lobpcg_ComputeResidualNorms( mv_MultiVectorPtr blockVectorR,
                             utilities_FortranMatrix* resNorms,
                             utilities_FortranMatrix* resDiag,
                             BlopexInt* mask, BlopexInt sizeX,
                             utilities_FortranInterpreter* util )
{
  BlopexInt i;
  double *rp;

  /* take inner product of blockVectorR with mask and
     return results in resDiag */

  mv_MultiVectorByMultiVectorDiag( blockVectorR, blockVectorR,
                                 mask, sizeX, resDiag->value);

  /* take sqrt of resDiag and place in resNorms */

  rp = (double *)resNorms->value;

  for ( i = 0; i < sizeX; i++ )
    if ( mask == NULL || mask[i] )
      rp[i] = sqrt((util->FortranMatrixAbs)(resDiag,i+1,1));
}

/* ------------------------------------------------------------
   lobpcg_checkResiduals                                generic
   ------------------------------------------------------------ */
static BlopexInt
lobpcg_checkResiduals( utilities_FortranMatrix* resNorms,
                       utilities_FortranMatrix* lambda,
                       lobpcg_Tolerance tol,
                       BlopexInt* activeMask,
                       utilities_FortranInterpreter* util )
{
  BlopexInt i, n;
  BlopexInt notConverged;
  double atol;
  double rtol;

  n = (util->FortranMatrixHeight)( resNorms );

  atol = tol.absolute;
  rtol = tol.relative;

  notConverged = 0;
  for ( i = 0; i < n; i++ ) {
    if ( utilities_FortranMatrixAbs( resNorms, i + 1, 1 ) >
     (util->FortranMatrixAbs)( lambda, i + 1, 1 )*rtol + atol
     + DBL_EPSILON ) {
      activeMask[i] = 1;
      notConverged++;
    }
    else
      activeMask[i] = 0;
  }
  return notConverged;
}
/* ------------------------------------------------------------
   lobpcg_errorMessage                                  generic
   ------------------------------------------------------------ */
static void
lobpcg_errorMessage( BlopexInt verbosityLevel, char* message )
{
  if ( verbosityLevel ) {
    fprintf( stderr, "Error in LOBPCG:\n" );
    fprintf( stderr, "%s", message );
  }
}
/* ------------------------------------------------------------
   lobpcg_dumpLambda                                    generic
   ------------------------------------------------------------ */
static void
lobpcg_dumpLambda( utilities_FortranMatrix* residualNorms,
                   utilities_FortranMatrix* lambda,
                   int* iterationNumber,
                   BlopexInt sizeX,
                   utilities_FortranInterpreter* util )
{
    BlopexInt i;
    double * eig;

    printf("\n");
    printf("Eigenvalue lambda       Residual              \n");
    for ( i = 1; i <= sizeX; i++ ) {
      eig = (double *)(util->FortranMatrixValuePtr)( lambda, i, 1);
      printf("%22.14e  %22.14e\n", *eig,
              utilities_FortranMatrixAbs( residualNorms, i, 1) );
      }
    printf("\n%d iterations\n", *iterationNumber );
}

/* ------------------------------------------------------------
   lobpcg_solve_double                                   double
   ------------------------------------------------------------ */
int
lobpcg_solve_double( mv_MultiVectorPtr blockVectorX,
          void * operatorAData,
          void (*operatorA)( void*, void*, void* ),
          void * operatorBData,
          void (*operatorB)( void*, void*, void* ),
          void * operatorTData,
          void (*operatorT)( void*, void*, void* ),
          mv_MultiVectorPtr blockVectorY,
          lobpcg_BLASLAPACKFunctions blap_fn,
          lobpcg_Tolerance tolerance,
          int      maxIterations,
          int      verbosityLevel,
          int    * iterationNumber,
          double * lambda_values,
          double * lambdaHistory_values,
          BlopexInt      lambdaHistory_gh,
          double * residualNorms_values,
          double * residualNormsHistory_values ,
          BlopexInt      residualNormsHistory_gh
){
  int exitflag;
  utilities_FortranInterpreter util;
  lobpcg_Interpreter lobpcg;

  util.FortranMatrixCreate = utilities_FortranMatrixCreate;
  util.FortranMatrixAllocateData = utilities_FortranMatrixAllocateData;
  util.FortranMatrixWrap = utilities_FortranMatrixWrap;
  util.FortranMatrixDestroy = utilities_FortranMatrixDestroy ;
  util.FortranMatrixGlobalHeight = utilities_FortranMatrixGlobalHeight;
  util.FortranMatrixHeight = utilities_FortranMatrixHeight;
  util.FortranMatrixWidth = utilities_FortranMatrixWidth;
  util.FortranMatrixValues = utilities_FortranMatrixValues;
  util.FortranMatrixClear = utilities_FortranMatrixClear;
  util.FortranMatrixClearL = utilities_FortranMatrixClearL;
  util.FortranMatrixSetToIdentity = utilities_FortranMatrixSetToIdentity;
  util.FortranMatrixTransposeSquare = utilities_FortranMatrixTransposeSquare;
  util.FortranMatrixSymmetrize = utilities_FortranMatrixSymmetrize;
  util.FortranMatrixCopy = utilities_FortranMatrixCopy;
  util.FortranMatrixIndexCopy = utilities_FortranMatrixIndexCopy;
  util.FortranMatrixSetDiagonal = utilities_FortranMatrixSetDiagonal;
  util.FortranMatrixGetDiagonal = utilities_FortranMatrixGetDiagonal;
  util.FortranMatrixAdd = utilities_FortranMatrixAdd;
  util.FortranMatrixDMultiply = utilities_FortranMatrixDMultiply;
  util.FortranMatrixMultiplyD = utilities_FortranMatrixMultiplyD;
  util.FortranMatrixMultiply = utilities_FortranMatrixMultiply;
  util.FortranMatrixFNorm = utilities_FortranMatrixFNorm;
  util.FortranMatrixAbs = utilities_FortranMatrixAbs;
  util.FortranMatrixValuePtr = utilities_FortranMatrixValuePtr;
  util.FortranMatrixMaxValue = utilities_FortranMatrixMaxValue;
  util.FortranMatrixSelectBlock = utilities_FortranMatrixSelectBlock;
  util.FortranMatrixUpperInv = utilities_FortranMatrixUpperInv;
  util.FortranMatrixPrint = utilities_FortranMatrixPrint;

  lobpcg.chol = lobpcg_chol;
  lobpcg.solveGEVP = lobpcg_solveGEVP;

  exitflag = lobpcg_solve(
          blockVectorX,
          operatorAData,
          operatorA,
          operatorBData,
          operatorB,
          operatorTData,
          operatorT,
          blockVectorY,
          blap_fn,
          tolerance,
          maxIterations,
          verbosityLevel,
          iterationNumber,
          (void *) lambda_values,
          (void *) lambdaHistory_values,
          lambdaHistory_gh,
          residualNorms_values,
          residualNormsHistory_values ,
          residualNormsHistory_gh,
          &lobpcg,
          &util );

   return(exitflag);
}
/* ------------------------------------------------------------
   lobpcg_solve_complex                                complex
   ------------------------------------------------------------ */
int
lobpcg_solve_complex( mv_MultiVectorPtr blockVectorX,
          void * operatorAData,
          void (*operatorA)( void*, void*, void* ),
          void * operatorBData,
          void (*operatorB)( void*, void*, void* ),
          void * operatorTData,
          void (*operatorT)( void*, void*, void* ),
          mv_MultiVectorPtr blockVectorY,
          lobpcg_BLASLAPACKFunctions blap_fn,
          lobpcg_Tolerance tolerance,
          int      maxIterations,
          int      verbosityLevel,
          int    * iterationNumber,
          komplex * lambda_values,
          komplex * lambdaHistory_values,
          BlopexInt      lambdaHistory_gh,
          double * residualNorms_values,
          double * residualNormsHistory_values ,
          BlopexInt      residualNormsHistory_gh
){
  int exitflag;
  utilities_FortranInterpreter util;
  lobpcg_Interpreter lobpcg;

  util.FortranMatrixCreate = utilities_FortranMatrixCreate;
  util.FortranMatrixAllocateData = zutilities_FortranMatrixAllocateData;
  util.FortranMatrixWrap = utilities_FortranMatrixWrap;
  util.FortranMatrixDestroy = utilities_FortranMatrixDestroy ;
  util.FortranMatrixGlobalHeight = utilities_FortranMatrixGlobalHeight;
  util.FortranMatrixHeight = utilities_FortranMatrixHeight;
  util.FortranMatrixWidth = utilities_FortranMatrixWidth;
  util.FortranMatrixValues = utilities_FortranMatrixValues;
  util.FortranMatrixClear = zutilities_FortranMatrixClear;
  util.FortranMatrixClearL = zutilities_FortranMatrixClearL;
  util.FortranMatrixSetToIdentity = zutilities_FortranMatrixSetToIdentity;
  util.FortranMatrixTransposeSquare = zutilities_FortranMatrixTransposeSquare;
  util.FortranMatrixSymmetrize = zutilities_FortranMatrixSymmetrize;
  util.FortranMatrixCopy = zutilities_FortranMatrixCopy;
  util.FortranMatrixIndexCopy = zutilities_FortranMatrixIndexCopy;
  util.FortranMatrixSetDiagonal = zutilities_FortranMatrixSetDiagonal;
  util.FortranMatrixGetDiagonal = zutilities_FortranMatrixGetDiagonal;
  util.FortranMatrixAdd = zutilities_FortranMatrixAdd;
  util.FortranMatrixDMultiply = zutilities_FortranMatrixDMultiply;
  util.FortranMatrixMultiplyD = zutilities_FortranMatrixMultiplyD;
  util.FortranMatrixMultiply = zutilities_FortranMatrixMultiply;
  util.FortranMatrixFNorm = utilities_FortranMatrixFNorm;
  util.FortranMatrixAbs = zutilities_FortranMatrixAbs;
  util.FortranMatrixValuePtr = zutilities_FortranMatrixValuePtr;
  util.FortranMatrixMaxValue = utilities_FortranMatrixMaxValue;
  util.FortranMatrixSelectBlock = zutilities_FortranMatrixSelectBlock;
  util.FortranMatrixUpperInv = zutilities_FortranMatrixUpperInv;
  util.FortranMatrixPrint = zutilities_FortranMatrixPrint;

  lobpcg.chol = zlobpcg_chol;
  lobpcg.solveGEVP = zlobpcg_solveGEVP;

  exitflag = lobpcg_solve(
          blockVectorX,
          operatorAData,
          operatorA,
          operatorBData,
          operatorB,
          operatorTData,
          operatorT,
          blockVectorY,
          blap_fn,
          tolerance,
          maxIterations,
          verbosityLevel,
          iterationNumber,
          (void *) lambda_values,
          (void *) lambdaHistory_values,
          lambdaHistory_gh,
          residualNorms_values,
          residualNormsHistory_values ,
          residualNormsHistory_gh,
          &lobpcg,
          &util );

   return(exitflag);
}
/* ------------------------------------------------------------
   lobpcg_solve                                         generic
   ------------------------------------------------------------ */
int
lobpcg_solve( mv_MultiVectorPtr blockVectorX,
              void* operatorAData,
              void (*operatorA)( void*, void*, void* ),
              void* operatorBData,
              void (*operatorB)( void*, void*, void* ),
              void* operatorTData,
              void (*operatorT)( void*, void*, void* ),
              mv_MultiVectorPtr blockVectorY,
              lobpcg_BLASLAPACKFunctions blap_fn,
              lobpcg_Tolerance tolerance,
              int  maxIterations,
              int  verbosityLevel,
              int* iterationNumber,

              void * lambda_values,
              void * lambdaHistory_values,
              BlopexInt lambdaHistory_gh,

              double * residualNorms_values,
              double * residualNormsHistory_values ,
              BlopexInt residualNormsHistory_gh,

              lobpcg_Interpreter *lobpcg,
              utilities_FortranInterpreter *util

){

  BlopexInt               sizeX; /* number of eigenvectors */
  BlopexInt               sizeY; /* number of constraints */
  BlopexInt               sizeR; /* number of residuals used */
  BlopexInt               sizeP; /* number of conj. directions used */
  BlopexInt               sizeA; /* size of the Gram matrix for A */
  BlopexInt               sizeX3; /* 3*sizeX */

  BlopexInt               firstR; /* first line of the Gram block
                               corresponding to residuals */
  BlopexInt               lastR; /* last line of this block */
  BlopexInt               firstP; /* same for conjugate directions */
  BlopexInt               lastP;

  BlopexInt               noTFlag; /* nonzero: no preconditioner */
  BlopexInt               noBFlag; /* nonzero: no operator B */
  BlopexInt               noYFlag; /* nonzero: no constaints */

  int                     exitFlag; /* 1: problem size is too small,
                                 2: block size < 1,
                                 3: linearly dependent constraints,
                                -1: requested accuracy not achieved */

  BlopexInt*              activeMask; /* soft locking mask */

  BlopexInt               i; /* short loop counter */

#if 0
  long              n; /* dimension 1 of X */
  /* had to remove because n is not available in some interfaces */
#endif

  mv_MultiVectorPtr     blockVectorR; /* residuals */
  mv_MultiVectorPtr     blockVectorP; /* conjugate directions */

  mv_MultiVectorPtr     blockVectorW; /* auxiliary block vector */

  mv_MultiVectorPtr     blockVectorAX; /* A*X */
  mv_MultiVectorPtr     blockVectorAR; /* A*R */
  mv_MultiVectorPtr     blockVectorAP; /* A*P */

  mv_MultiVectorPtr     blockVectorBX; /* B*X */
  mv_MultiVectorPtr     blockVectorBR; /* B*R */
  mv_MultiVectorPtr     blockVectorBP; /* B*P */

  mv_MultiVectorPtr     blockVectorBY; /* B*Y */

  utilities_FortranMatrix*  gramA; /* Gram matrix for A */
  utilities_FortranMatrix*  gramB; /* Gram matrix for B */
  utilities_FortranMatrix*  lambdaAB; /* eigenvalues of
                                         gramA u = lambda gram B u */
  utilities_FortranMatrix*  lambdaX; /* first sizeX eigenvalues in
                                        lambdaAB (ref) */

  utilities_FortranMatrix*  gramXAX; /* XX block of gramA (ref) */
  utilities_FortranMatrix*  gramRAX; /* XR block of gramA (ref) */
  utilities_FortranMatrix*  gramPAX; /* XP block of gramA (ref) */

  utilities_FortranMatrix*  gramRAR; /* RR block of gramA (ref) */
  utilities_FortranMatrix*  gramPAR; /* RP block of gramA (ref) */

  utilities_FortranMatrix*  gramPAP; /* PP block of gramA (ref) */

  utilities_FortranMatrix*  gramXBX; /* XX block of gramB (ref) */
  utilities_FortranMatrix*  gramRBX; /* XR block of gramB (ref) */
  utilities_FortranMatrix*  gramPBX; /* XP block of gramB (ref) */

  utilities_FortranMatrix*  gramRBR; /* RR block of gramB (ref) */
  utilities_FortranMatrix*  gramPBR; /* RP block of gramB (ref) */

  utilities_FortranMatrix*  gramPBP; /* PP block of gramB (ref) */

  utilities_FortranMatrix*  gramYBY; /* Matrices for constraints */
  utilities_FortranMatrix*  gramYBX;
  utilities_FortranMatrix*  tempYBX;
  utilities_FortranMatrix*  gramYBR; /* ref. */
  utilities_FortranMatrix*  tempYBR; /* ref. */

  utilities_FortranMatrix*  coordX; /* coordinates of the first sizeX
                                       Ritz vectors in the XRP basis */
  utilities_FortranMatrix*  coordXX; /* coordinates of the above in X */
  utilities_FortranMatrix*  coordRX; /* coordinates of the above in R */
  utilities_FortranMatrix*  coordPX; /* coordinates of the above in P */

  utilities_FortranMatrix*  upperR; /* R factor in QR-fact. (ref) */

  utilities_FortranMatrix* lambda;
  utilities_FortranMatrix* lambdaHistory;
  utilities_FortranMatrix* lambdaColumn; /* reference to a column
                                            in lambda history */

  utilities_FortranMatrix* residualNorms;
  utilities_FortranMatrix* residualNormsHistory;
  utilities_FortranMatrix* residualNormsColumn;  /* reference to a column
                                                    in norms history */

  utilities_FortranMatrix* residualDiag;

  /* ------------- initialization ------------- */

  exitFlag = 0;
  *iterationNumber = 0;
  noTFlag = operatorT == NULL;
  noBFlag = operatorB == NULL;

  sizeY = mv_MultiVectorWidth( blockVectorY );
  noYFlag = sizeY == 0;

  sizeX = mv_MultiVectorWidth( blockVectorX );

  lambda = (util->FortranMatrixCreate)();
  (util->FortranMatrixWrap)(lambda_values, sizeX, sizeX, 1, lambda);


/* prepare to process eigenvalues history, if user has provided non-NULL
   as "lambdaHistory_values" argument */

  if (lambdaHistory_values!=NULL) {
      lambdaHistory = (util->FortranMatrixCreate)();
      (util->FortranMatrixWrap)(lambdaHistory_values, lambdaHistory_gh, sizeX,
                                maxIterations+1, lambdaHistory);
  }
  else
      lambdaHistory = NULL;

  residualNorms = utilities_FortranMatrixCreate();
  utilities_FortranMatrixWrap(residualNorms_values, sizeX, sizeX, 1, residualNorms);

/* while residualNorms is always double residualDiag is same as blockVectors */
  residualDiag = (util->FortranMatrixCreate)();
  (util->FortranMatrixAllocateData)(sizeX,1,residualDiag);

/* prepare to process residuals history, if user has provided non-NULL as
"residualNormsHistory_values " argument */

  if (residualNormsHistory_values!=NULL) {
      residualNormsHistory = utilities_FortranMatrixCreate();
      utilities_FortranMatrixWrap(residualNormsHistory_values, residualNormsHistory_gh,
                                 sizeX, maxIterations+1,residualNormsHistory);
  }
  else
      residualNormsHistory = NULL;

#if 0
  /* had to remove because n is not available in some interfaces */
  n = mv_MultiVectorHeight( blockVectorX );

  if ( n < 5*sizeX ) {
    exitFlag = PROBLEM_SIZE_TOO_SMALL;
    lobpcg_errorMessage( verbosityLevel,
                        "Problem size too small compared to block size\n" );
    (util->FortranMatrixDestroy)( lambda );
    (util->FortranMatrixDestroy)( lambdaHistory );
    utilities_FortranMatrixDestroy( residualNorms );
    utilities_FortranMatrixDestroy( residualNormsHistory );
    (util->FortranMatrixDestroy)( residualDiag );
    return exitFlag;
  }
#endif

  if ( sizeX < 1 ) {
    exitFlag = WRONG_BLOCK_SIZE;
    lobpcg_errorMessage( verbosityLevel, "The bloc size is wrong.\n" );
    (util->FortranMatrixDestroy)( lambda );
    (util->FortranMatrixDestroy)( lambdaHistory );
    utilities_FortranMatrixDestroy( residualNorms );
    utilities_FortranMatrixDestroy( residualNormsHistory );
    (util->FortranMatrixDestroy)( residualDiag );
    return exitFlag;
  }

  gramYBY = (util->FortranMatrixCreate)();
  gramYBX = (util->FortranMatrixCreate)();
  tempYBX = (util->FortranMatrixCreate)();
  gramYBR = (util->FortranMatrixCreate)();
  tempYBR = (util->FortranMatrixCreate)();

  blockVectorW = mv_MultiVectorCreateCopy( blockVectorX, 0 );
  blockVectorBY = blockVectorY;

  if ( !noYFlag ) {
    (util->FortranMatrixAllocateData)( sizeY, sizeY, gramYBY );
    (util->FortranMatrixAllocateData)( sizeY, sizeX, gramYBX );
    (util->FortranMatrixAllocateData)( sizeY, sizeX, tempYBX );
    blockVectorBY = blockVectorY;
    if ( !noBFlag ) {
      blockVectorBY = mv_MultiVectorCreateCopy( blockVectorY, 0 );
      operatorB( operatorBData, mv_MultiVectorGetData(blockVectorY),
                 mv_MultiVectorGetData(blockVectorBY) );
    };

    lobpcg_MultiVectorByMultiVector( blockVectorBY, blockVectorY, gramYBY, util );
    exitFlag = (lobpcg->chol)( gramYBY, blap_fn, util );

    if ( exitFlag != 0 ) {
      if ( verbosityLevel )
        printf("Cannot handle linear dependent constraints\n");
      (util->FortranMatrixDestroy)( gramYBY );
      (util->FortranMatrixDestroy)( gramYBX );
      (util->FortranMatrixDestroy)( tempYBX );
      (util->FortranMatrixDestroy)( gramYBR );
      (util->FortranMatrixDestroy)( tempYBR );
      if ( !noBFlag )
        mv_MultiVectorDestroy( blockVectorBY );
      mv_MultiVectorDestroy( blockVectorW );
      (util->FortranMatrixDestroy)( lambda );
      (util->FortranMatrixDestroy)( lambdaHistory );
      utilities_FortranMatrixDestroy( residualNorms );
      utilities_FortranMatrixDestroy( residualNormsHistory );
      (util->FortranMatrixDestroy)( residualDiag );
      return WRONG_CONSTRAINTS;
    }

    (util->FortranMatrixUpperInv)( gramYBY );
    (util->FortranMatrixClearL)( gramYBY );

    /*--------- apply the constraints to the initial X -------- */

    lobpcg_MultiVectorByMultiVector( blockVectorBY, blockVectorX, gramYBX, util );
    (util->FortranMatrixMultiply)( gramYBY, 1, gramYBX, 0, tempYBX );
    (util->FortranMatrixMultiply)( gramYBY, 0, tempYBX, 0, gramYBX );
    lobpcg_MultiVectorByMatrix( blockVectorY, gramYBX, blockVectorW, util );
    mv_MultiVectorAxpy( -1.0, blockVectorW, blockVectorX );
  }

  if ( verbosityLevel ) {
    printf("\nSolving ");
    if ( noBFlag )
      printf("standard");
    else
      printf("generalized");
    printf(" eigenvalue problem with");
    if ( noTFlag )
      printf("out");
    printf(" preconditioning\n\n");
    printf("block size %d\n\n",(int) sizeX );
    if ( noYFlag )
      printf("No constraints\n\n");
    else {
      if ( sizeY > 1 )
        printf("%d constraints\n\n",(int) sizeY);
      else
        printf("%d constraint\n\n",(int) sizeY);
    }
  }

  /* -------------- creating fortran matrix shells --------- */

  gramA = (util->FortranMatrixCreate)();
  gramB = (util->FortranMatrixCreate)();
  lambdaAB = (util->FortranMatrixCreate)();
  lambdaX = (util->FortranMatrixCreate)();

  gramXAX = (util->FortranMatrixCreate)();
  gramRAX = (util->FortranMatrixCreate)();
  gramPAX = (util->FortranMatrixCreate)();

  gramRAR = (util->FortranMatrixCreate)();
  gramPAR = (util->FortranMatrixCreate)();

  gramPAP = (util->FortranMatrixCreate)();

  gramXBX = (util->FortranMatrixCreate)();
  gramRBX = (util->FortranMatrixCreate)();
  gramPBX = (util->FortranMatrixCreate)();

  gramRBR = (util->FortranMatrixCreate)();
  gramPBR = (util->FortranMatrixCreate)();

  gramPBP = (util->FortranMatrixCreate)();

  coordX  = (util->FortranMatrixCreate)();
  coordXX = (util->FortranMatrixCreate)();
  coordRX = (util->FortranMatrixCreate)();
  coordPX = (util->FortranMatrixCreate)();

  upperR = (util->FortranMatrixCreate)();

  lambdaColumn = utilities_FortranMatrixCreate();
  residualNormsColumn = utilities_FortranMatrixCreate();

  /* --------- initializing soft locking mask ------------ */
  activeMask = (BlopexInt*)calloc( sizeX, sizeof(BlopexInt) );
  BlopexAssert( activeMask != NULL );
  for ( i = 0; i < sizeX; i++ )
    activeMask[i] = 1;

  /*------- allocate memory for Gram matrices and the Ritz values ----- */
  sizeX3 = 3*sizeX;
  (util->FortranMatrixAllocateData)( sizeX3, sizeX3, gramA );
  (util->FortranMatrixAllocateData)( sizeX3, sizeX3, gramB );
  (util->FortranMatrixAllocateData)( sizeX3, 1, lambdaAB );

  /*------ creating block vectors R, P, AX, AR, AP, BX, BR, BP and W ----- */
  blockVectorR = mv_MultiVectorCreateCopy( blockVectorX, 0 );
  blockVectorP = mv_MultiVectorCreateCopy( blockVectorX, 0 );
  blockVectorAX = mv_MultiVectorCreateCopy( blockVectorX, 0 );
  blockVectorAR = mv_MultiVectorCreateCopy( blockVectorX, 0 );
  blockVectorAP = mv_MultiVectorCreateCopy( blockVectorX, 0 );

  if ( !noBFlag ) {
    blockVectorBX = mv_MultiVectorCreateCopy( blockVectorX, 0 );
    blockVectorBR = mv_MultiVectorCreateCopy( blockVectorX, 0 );
    blockVectorBP = mv_MultiVectorCreateCopy( blockVectorX, 0 );
  }
  else {
    blockVectorBX = blockVectorX;
    blockVectorBR = blockVectorR;
    blockVectorBP = blockVectorP;
  }

  mv_MultiVectorSetMask( blockVectorR, activeMask );
  mv_MultiVectorSetMask( blockVectorP, activeMask );
  mv_MultiVectorSetMask( blockVectorAR, activeMask );
  mv_MultiVectorSetMask( blockVectorAP, activeMask );
  if ( !noBFlag ) {
    mv_MultiVectorSetMask( blockVectorBR, activeMask );
    mv_MultiVectorSetMask( blockVectorBP, activeMask );
  }
  mv_MultiVectorSetMask( blockVectorW, activeMask );

  /*--- B-orthonormaliization of blockVectorX */
  /* selecting a block in gramB for R factor upperR */

  (util->FortranMatrixSelectBlock)( gramB, 1, sizeX, 1, sizeX, upperR );
  if ( !noBFlag ) {
    operatorB( operatorBData, mv_MultiVectorGetData(blockVectorX),
                              mv_MultiVectorGetData(blockVectorBX) );
  }
  exitFlag = lobpcg_MultiVectorImplicitQR( blockVectorX, blockVectorBX,
                       upperR, blockVectorW,blap_fn,lobpcg, util );
  if ( exitFlag ) {
    lobpcg_errorMessage( verbosityLevel, "Bad initial vectors: orthonormalization failed\n" );
    if ( verbosityLevel )
      printf("DPOTRF INFO = %d\n", exitFlag);
  }
  else {
    if ( !noBFlag ) { /* update BX */
      lobpcg_MultiVectorByMatrix( blockVectorBX, upperR, blockVectorW, util );
      mv_MultiVectorCopy( blockVectorW, blockVectorBX );
    }

    /*--- blockVectorAX = operatorAData*blockVectorX  */
    /*--- or, if no operatorAData */
    /*--- blockVectorAX = ???(blockVectorX)  */
    operatorA( operatorAData, mv_MultiVectorGetData(blockVectorX),
               mv_MultiVectorGetData(blockVectorAX) );
 /*
    mv_MultiVectorPrint( blockVectorX,"blockVectorX",0 );
    mv_MultiVectorPrint( blockVectorAX,"blockVectorAX",0 );
 */
    /*--- gramXAX = gramA(1:sizeX,1:sizeX) */
    /*--- gramXAX = blockVectorX'*blockVectorAX */
    /*--- gramXAX = (gramXAX'+gramXAX)/2 */
    (util->FortranMatrixSelectBlock)( gramA, 1, sizeX, 1, sizeX, gramXAX );
    lobpcg_MultiVectorByMultiVector( blockVectorX, blockVectorAX, gramXAX, util );
    (util->FortranMatrixSymmetrize)( gramXAX );

    /*--- gramXBX = gramB(1:sizeX,1:sizeX) */
    /*--- gramXBX = blockVectorX'*blockVectorBX */
    /*--- gramXBX = (gramXBX'+gramXBX)/2 */
    (util->FortranMatrixSelectBlock)( gramB, 1, sizeX, 1, sizeX, gramXBX );
    lobpcg_MultiVectorByMultiVector( blockVectorX, blockVectorBX, gramXBX, util );
    (util->FortranMatrixSymmetrize)( gramXBX );

    /*  (util->FortranMatrixSetToIdentity}( gramXBX );*/ /* X may be bad! */

    /*--- find eig of gramXAX*x=(lambda)*gramXBX*x */
    /*--- gramXAX = eig-vectors */
    /*--- lambda  = eig-values  */
    if ( (exitFlag = (lobpcg->solveGEVP)( gramXAX, gramXBX, lambda,blap_fn, util)) != 0 ) {
      lobpcg_errorMessage( verbosityLevel,
                          "Bad problem: Rayleigh-Ritz in the initial subspace failed\n" );
      if ( verbosityLevel )
        printf("DSYGV INFO = %d\n", exitFlag);
    }
    else {
      /*--- coordX = gramXAX(1:sizeZ,1:sizeX) */
      (util->FortranMatrixSelectBlock)( gramXAX, 1, sizeX, 1, sizeX, coordX );

      /*--- blockVectorW = blockVectorX * coordX */
      /*--- blockVectorX = blockVectorW */
      lobpcg_MultiVectorByMatrix( blockVectorX, coordX, blockVectorW, util );
      mv_MultiVectorCopy( blockVectorW, blockVectorX );

      /*--- blockVectorW = blockVectorAX * coordX */
      /*--- blockVectorAx = blockVectorW */
      lobpcg_MultiVectorByMatrix( blockVectorAX, coordX, blockVectorW, util );
      mv_MultiVectorCopy( blockVectorW, blockVectorAX );

      if ( !noBFlag ) {
        lobpcg_MultiVectorByMatrix( blockVectorBX, coordX, blockVectorW, util );
        mv_MultiVectorCopy( blockVectorW, blockVectorBX );
      }

      /*
      lobpcg_MultiVectorByMultiVector( blockVectorBX, blockVectorX, upperR, util );
      (util->FortranMatrixPrint)( upperR, "xbx.dat" );
      (util->FortranMatrixPrint)( lambda, "lmd.dat" );
      */

      /*--- blockVectorR = blockVectorBX*lambda */
      mv_MultiVectorByDiagonal( blockVectorBX,
                                NULL, sizeX,
                               (util->FortranMatrixValues)( lambda ),
                                blockVectorR );

      /*--- blockVectorR = BlockVectorR - blockVectorAx */
      mv_MultiVectorAxpy( -1.0, blockVectorAX, blockVectorR );

      /*--- residualDiag = diag(blockVectorR'*blockVectorR) */
      /*--- residualNorms = sqrt(abs(residualDiag))  */
      lobpcg_ComputeResidualNorms( blockVectorR, residualNorms,
                                   residualDiag, NULL, sizeX, util );

      if ( lambdaHistory != NULL ) {
        (util->FortranMatrixSelectBlock)( lambdaHistory, 1, sizeX, 1, 1,
                                          lambdaColumn );
        (util->FortranMatrixCopy)( lambda, 0, lambdaColumn );
      }

      if ( residualNormsHistory != NULL ) {
        utilities_FortranMatrixSelectBlock( residualNormsHistory, 1, sizeX, 1, 1,
                                            residualNormsColumn );
        utilities_FortranMatrixCopy( residualNorms, 0, residualNormsColumn );
      }

      if ( verbosityLevel == 2 )
        lobpcg_dumpLambda( residualNorms, lambda,
                           iterationNumber, sizeX, util);
      else if ( verbosityLevel == 1 )
        printf("\nInitial Max. Residual %22.14e\n",
               utilities_FortranMatrixMaxValue( residualNorms ) );
    }
  }

  /*================== start of main loop ====================*/
  for ( *iterationNumber = 1; exitFlag == 0 && *iterationNumber <= maxIterations;
      (*iterationNumber)++ ) {

    /*--- sets activeMask, eliminating lambda that have converged */
    sizeR = lobpcg_checkResiduals( residualNorms, lambda, tolerance,
                                   activeMask, util );
    if ( sizeR < 1 )
      break;

/* following code added by Ilya Lashuk on March 22, 2005; with current
   multivector implementation mask needs to be reset after it has changed on each vector
   mask applies to */

    mv_MultiVectorSetMask( blockVectorR,  activeMask );
    mv_MultiVectorSetMask( blockVectorP,  activeMask );
    mv_MultiVectorSetMask( blockVectorAR, activeMask );
    mv_MultiVectorSetMask( blockVectorAP, activeMask );
    if ( !noBFlag ) {
      mv_MultiVectorSetMask( blockVectorBR, activeMask );
      mv_MultiVectorSetMask( blockVectorBP, activeMask );
    }
    mv_MultiVectorSetMask( blockVectorW, activeMask );

/* ***** end of added code ***** */

    if ( !noTFlag ) {
      operatorT( operatorTData, mv_MultiVectorGetData(blockVectorR),
                 mv_MultiVectorGetData(blockVectorW) );
      mv_MultiVectorCopy( blockVectorW, blockVectorR );
    }

    if ( !noYFlag ) { /* apply the constraints to R  */
      /*--- gramYBR = gramYBX(1:sizeY,1:sizeR) */
      /*--- tempYBR = tempYBX(1:sizeY,1:sizeR) */
      (util->FortranMatrixSelectBlock)( gramYBX, 1, sizeY, 1, sizeR, gramYBR );
      (util->FortranMatrixSelectBlock)( tempYBX, 1, sizeY, 1, sizeR, tempYBR );

      /*--- gramYBR = blockVectorBY'*blockVectorR      */
      /*--- tempYBR = gramYBY'*gramYBR                 */
      /*--- gramYBR = gramYBY*tempYBR                  */
      /*--- blockVectorW = blockVectorY*gramYBR        */
      /*--- blockVectorR = blockVectorR - blockVectorW */
      lobpcg_MultiVectorByMultiVector( blockVectorBY, blockVectorR, gramYBR, util );
      (util->FortranMatrixMultiply)( gramYBY, 1, gramYBR, 0, tempYBR );
      (util->FortranMatrixMultiply)( gramYBY, 0, tempYBR, 0, gramYBR );
      lobpcg_MultiVectorByMatrix( blockVectorY, gramYBR, blockVectorW, util );
      mv_MultiVectorAxpy( -1.0, blockVectorW, blockVectorR );
    }

    firstR = sizeX + 1;
    lastR = sizeX + sizeR;
    firstP = lastR + 1;

    /*--- upperR = gramB(firstR:lastR,firstR:lastR)  */
    (util->FortranMatrixSelectBlock)( gramB, firstR, lastR, firstR, lastR, upperR );

    if ( !noBFlag ) {
      operatorB( operatorBData, mv_MultiVectorGetData(blockVectorR),
                 mv_MultiVectorGetData(blockVectorBR) );
    }

    /*--- orthonormilize blockVectorR */
    exitFlag = lobpcg_MultiVectorImplicitQR( blockVectorR, blockVectorBR,
                                             upperR, blockVectorW,blap_fn,lobpcg, util );
    if ( exitFlag ) {
      lobpcg_errorMessage( verbosityLevel, "Orthonormalization of residuals failed\n" );
      if ( verbosityLevel )
        printf("DPOTRF INFO = %d\n", exitFlag);
      break;
    }

    if ( !noBFlag ) { /* update BR */
      lobpcg_MultiVectorByMatrix( blockVectorBR, upperR, blockVectorW, util );
      mv_MultiVectorCopy( blockVectorW, blockVectorBR );
    }

    /*--- blockVectorAR = operatorAdata*blockVectorR */
    operatorA( operatorAData, mv_MultiVectorGetData(blockVectorR),
               mv_MultiVectorGetData(blockVectorAR) );

    if ( *iterationNumber > 1 ) {

      sizeP = sizeR;
      lastP = lastR + sizeP;

      /*--- upperR = gramB(firstP:lastP,firstP:lastP)  */
      (util->FortranMatrixSelectBlock)( gramB, firstP, lastP, firstP, lastP, upperR );

      /*--- orthonormilize blockVectorP */
      exitFlag = lobpcg_MultiVectorImplicitQR( blockVectorP, blockVectorBP,
                                               upperR, blockVectorW,blap_fn,lobpcg, util );
      if ( exitFlag ) {
    /*
        lobpcg_errorMessage( verbosityLevel, "Orthonormalization of P failed\n" );
        if ( verbosityLevel )
          printf("DPOTRF INFO = %d\n", exitFlag);
    */
        sizeP = 0;
      }
      else {

        if ( !noBFlag ) { /* update BP */
          lobpcg_MultiVectorByMatrix( blockVectorBP, upperR, blockVectorW, util );
        mv_MultiVectorCopy( blockVectorW, blockVectorBP );
      }

      /*--- update AP */
      /*--- blockVectorW = blockVectorAP * upperR  */
      /*--- blockVectorAP = blockVectorW           */
      lobpcg_MultiVectorByMatrix( blockVectorAP, upperR, blockVectorW, util );
      mv_MultiVectorCopy( blockVectorW, blockVectorAP );
      }
    }
    else {

      sizeP = 0;
      lastP = lastR;
    }

    sizeA = lastR + sizeP;

    /*--- gramXAX = gramA(1:sizeX,1:sizeX)            */
    /*--- gramRAX = gramA(firstR:lastR,1:sizeX)       */
    /*--- gramRAR = gramA(firstR:lastR,firstR:lastR)  */
    /*--- gramXBX = gramB(1:sizeX,1:sizeX)            */
    /*--- gramRBX = gramB(firstR:lastR,1:sizeX)       */
    /*--- gramRBR = gramB(firstR:lastR,firstR:lastR)  */
    (util->FortranMatrixSelectBlock)( gramA, 1, sizeX, 1, sizeX, gramXAX );
    (util->FortranMatrixSelectBlock)( gramA, firstR, lastR, 1, sizeX,
                                      gramRAX );
    (util->FortranMatrixSelectBlock)( gramA, firstR, lastR, firstR, lastR,
                                      gramRAR );
    (util->FortranMatrixSelectBlock)( gramB, 1, sizeX, 1, sizeX, gramXBX );
    (util->FortranMatrixSelectBlock)( gramB, firstR, lastR, 1, sizeX,
                                      gramRBX );
    (util->FortranMatrixSelectBlock)( gramB, firstR, lastR, firstR, lastR,
                                      gramRBR );

    /*--- gramXAX = 0.0;        */
    /*--- diag(gramXAX)=lambda  */
    (util->FortranMatrixClear)( gramXAX );
    (util->FortranMatrixSetDiagonal)( gramXAX, lambda );

    /*--- gramRAX = blockVectorR'*blockVectorAX  */
    lobpcg_MultiVectorByMultiVector( blockVectorR, blockVectorAX, gramRAX, util );

    /*--- gramRAR = blockVectorR'*blockVectorAR  */
    /*--- gramRAR = (gramRAR'+gramRAR)/2         */
    lobpcg_MultiVectorByMultiVector( blockVectorR, blockVectorAR, gramRAR, util );
    (util->FortranMatrixSymmetrize)( gramRAR );

    /*--- gramXBX = eye  */
    (util->FortranMatrixSetToIdentity)( gramXBX );

    /*--- gramRBX = blockVectorR'*blockVectorBX  */
    lobpcg_MultiVectorByMultiVector( blockVectorR, blockVectorBX, gramRBX, util );

    /*--- gramRBR = eye  */
    (util->FortranMatrixSetToIdentity)( gramRBR );

    if ( *iterationNumber > 1 ) {

      /*--- gramPAX = gramA(firstP:lastP,1:sizeX)       */
      /*--- gramPAR = gramA(firstP:lastP,firstR:lastR)  */
      /*--- gramPAP = gramA(firstP:lastP,firstP:lastP)  */
      (util->FortranMatrixSelectBlock)( gramA, firstP, lastP, 1, sizeX, gramPAX );
      (util->FortranMatrixSelectBlock)( gramA, firstP, lastP, firstR, lastR, gramPAR );
      (util->FortranMatrixSelectBlock)( gramA, firstP, lastP, firstP, lastP, gramPAP );

      /*--- gramPBX = gramB(firstP:lastP,1:sizeX)        */
      /*--- gramPBR = gramB(firstP:lastP,firstR:lastR)   */
      /*--- gramPBP = gramB(firstP:lastP,firstP:lastP)   */
      (util->FortranMatrixSelectBlock)( gramB, firstP, lastP, 1, sizeX, gramPBX );
      (util->FortranMatrixSelectBlock)( gramB, firstP, lastP, firstR, lastR, gramPBR );
      (util->FortranMatrixSelectBlock)( gramB, firstP, lastP, firstP, lastP, gramPBP );

      /*--- gramPAX = blockVectorP'*blockVectorAX    */
      lobpcg_MultiVectorByMultiVector( blockVectorP, blockVectorAX, gramPAX, util );

      /*--- gramPAR = blockVectorP'*blockVectorAR    */
      lobpcg_MultiVectorByMultiVector( blockVectorP, blockVectorAR, gramPAR, util );

      /*--- gramPAP = blockVectorP'*blockVectorAP    */
      /*--- gramPAP = (gramPAP'+gramPAP)/2           */
      lobpcg_MultiVectorByMultiVector( blockVectorP, blockVectorAP, gramPAP, util );
      (util->FortranMatrixSymmetrize)( gramPAP );

      /*--- gramPBX = blockVectorP'*blockVectorBX    */
      lobpcg_MultiVectorByMultiVector( blockVectorP, blockVectorBX, gramPBX, util );

      /*--- gramPBR = blockVectorP'*blockVectorBR    */
      lobpcg_MultiVectorByMultiVector( blockVectorP, blockVectorBR, gramPBR, util );

      /*--- gramPBP = eye   */
      (util->FortranMatrixSetToIdentity)( gramPBP );
    }

    /*--- gramXAX = gramA(1:sizeA,1:sizeA)   */
    /*--- gramXBX = gramB(1:sizeA,1:sizeA)   */
    (util->FortranMatrixSelectBlock)( gramA, 1, sizeA, 1, sizeA, gramXAX );
    (util->FortranMatrixSelectBlock)( gramB, 1, sizeA, 1, sizeA, gramXBX );

    /*--- find eig of gramXAX*x=(lambdaAB)*gramXBX*x  */
    /*--- gramXAX  = eig-vectors */
    /*--- lambdaAB = eig-values  */
    if ( (exitFlag = (lobpcg->solveGEVP)( gramXAX, gramXBX, lambdaAB, blap_fn, util )) != 0 ) {
      lobpcg_errorMessage( verbosityLevel, "GEVP solver failure\n" );
      fprintf(stderr, "Return Code %d\n", exitFlag);
      printf("INFO = %d\n", exitFlag);
      (*iterationNumber)--;
      break;
    }

    /*--- lambdaX = lambdaAB(1:sizeX,1:1)    */
    /*--- lambda  = lambdaX                  */
    (util->FortranMatrixSelectBlock)( lambdaAB, 1, sizeX, 1, 1, lambdaX );
    (util->FortranMatrixCopy)( lambdaX, 0, lambda );

    /*--- coordX = gramA(1:sizeA,1:sizeX)    */
    (util->FortranMatrixSelectBlock)( gramA, 1, sizeA, 1, sizeX, coordX );

    /*--- coordXX = coordX(1:sizeX,1:sizeX)        */
    /*--- coordRX = coordX(firstR:lastR,1:sizeX)   */
    (util->FortranMatrixSelectBlock)( coordX, 1, sizeX, 1, sizeX, coordXX );
    (util->FortranMatrixSelectBlock)( coordX, firstR, lastR, 1, sizeX, coordRX );

    if ( *iterationNumber > 1 ) {

      /*--- coordPX = coordX(firstP:lastP,1:sizeX)   */
      (util->FortranMatrixSelectBlock)( coordX, firstP, lastP, 1, sizeX, coordPX );

      /*--- make all vectors in blockVectorW active  */
      /*--- blockVectorW = blockVectorP*coordPX      */
      /*--- make all vectors in blockVectorP active  */
      /*--- blockVectorP = blockVectorW              */
      mv_MultiVectorSetMask( blockVectorW, NULL );
      lobpcg_MultiVectorByMatrix( blockVectorP, coordPX, blockVectorW, util );
      mv_MultiVectorSetMask( blockVectorP, NULL );
      mv_MultiVectorCopy( blockVectorW, blockVectorP );

      /*--- blockVectorW = blockVectorAP * coordPX   */
      /*--- make all vectors of blockVectorAP active */
      /*--- blockVectorAP = blockVectorW             */
      lobpcg_MultiVectorByMatrix( blockVectorAP, coordPX, blockVectorW, util );
      mv_MultiVectorSetMask( blockVectorAP, NULL );
      mv_MultiVectorCopy( blockVectorW, blockVectorAP );

      if ( !noBFlag ) {
        lobpcg_MultiVectorByMatrix( blockVectorBP, coordPX, blockVectorW, util );
        mv_MultiVectorSetMask( blockVectorBP, NULL );
        mv_MultiVectorCopy( blockVectorW, blockVectorBP );
      }

      /*--- blockVectorW = blockVectorR * coordRX         */
      /*--- blockVectorP = blockVectorP + blockVector W   */
      lobpcg_MultiVectorByMatrix( blockVectorR, coordRX, blockVectorW, util );
      mv_MultiVectorAxpy( 1.0, blockVectorW, blockVectorP );

      /*--- blockVectorW = blockVectorAR * coordRX        */
      /*--- blockVectorAP = blockVectorAP + glockVectorW  */
      lobpcg_MultiVectorByMatrix( blockVectorAR, coordRX, blockVectorW, util );
      mv_MultiVectorAxpy( 1.0, blockVectorW, blockVectorAP );

      if ( !noBFlag ) {
        lobpcg_MultiVectorByMatrix( blockVectorBR, coordRX, blockVectorW, util );
        mv_MultiVectorAxpy( 1.0, blockVectorW, blockVectorBP );
      }

    }
    else {

      /*--- make all vectors of blockVectorP active */
      /*--- blockVectorP = blockVectorR * coordRX   */
      mv_MultiVectorSetMask( blockVectorP, NULL );
      lobpcg_MultiVectorByMatrix( blockVectorR, coordRX, blockVectorP, util );

      /*--- make all vectors of blockVectorAP active */
      /*--- blockVectorAP = blockVectorAR * coordRX  */
      mv_MultiVectorSetMask( blockVectorAP, NULL );
      lobpcg_MultiVectorByMatrix( blockVectorAR, coordRX, blockVectorAP, util );

      if ( !noBFlag ) {
        mv_MultiVectorSetMask( blockVectorBP, NULL );
        lobpcg_MultiVectorByMatrix( blockVectorBR, coordRX, blockVectorBP, util );
      }

    }

    /*--- blockVectorW = blockVector X              */
    /*--- blockVectorX = blockVector W * coordXX    */
    /*--- blockVectorX = blockVectorX + blockVectorP  */
    mv_MultiVectorSetMask( blockVectorW, NULL );
    mv_MultiVectorCopy( blockVectorX, blockVectorW );
    lobpcg_MultiVectorByMatrix( blockVectorW, coordXX, blockVectorX, util );
    mv_MultiVectorAxpy( 1.0, blockVectorP, blockVectorX );

    /*--- blockVectorW  = blockVectorAX  */
    /*--- blockVectorAX = blockVector W * coordXX    */
    /*--- blockVectorAX = blockVectorAX + blockVectorAP  */
    mv_MultiVectorCopy( blockVectorAX, blockVectorW );
    lobpcg_MultiVectorByMatrix( blockVectorW, coordXX, blockVectorAX, util );
    mv_MultiVectorAxpy( 1.0, blockVectorAP, blockVectorAX );

    if ( !noBFlag ) {
      mv_MultiVectorCopy( blockVectorBX, blockVectorW );
      lobpcg_MultiVectorByMatrix( blockVectorW, coordXX, blockVectorBX, util );
      mv_MultiVectorAxpy( 1.0, blockVectorBP, blockVectorBX );
    }

    mv_MultiVectorSetMask( blockVectorAX, activeMask );
    mv_MultiVectorSetMask( blockVectorBX, activeMask );

    /*--- blockVectorR = blockVectorBX(activeMask)*lambda  */
    mv_MultiVectorByDiagonal( blockVectorBX, activeMask, sizeX,
                             (util->FortranMatrixValues)( lambda ),
                              blockVectorR );

    /*--- blockVectorR = blockVectorR - blockVectorAX  */
    mv_MultiVectorAxpy( -1.0, blockVectorAX, blockVectorR );

    lobpcg_ComputeResidualNorms( blockVectorR, residualNorms,
                                 residualDiag, activeMask, sizeX, util );

    i = *iterationNumber + 1;
    if ( lambdaHistory != NULL ) {
      (util->FortranMatrixSelectBlock)( lambdaHistory, 1, sizeX, i, i,
                                        lambdaColumn );
      (util->FortranMatrixCopy)( lambda, 0, lambdaColumn );
    }

    if ( residualNormsHistory != NULL ) {
      utilities_FortranMatrixSelectBlock( residualNormsHistory, 1, sizeX, i, i,
                                          residualNormsColumn );
      utilities_FortranMatrixCopy( residualNorms, 0, residualNormsColumn );
    }

    if ( verbosityLevel == 2 ) {
      printf( "Iteration %d \tbsize %d\n", *iterationNumber,(int) sizeR );
      lobpcg_dumpLambda( residualNorms, lambda,
                       iterationNumber, sizeX, util);
      }
    else if ( verbosityLevel == 1 )
      printf("Iteration %d \tbsize %d \tmaxres %22.14e\n",
             *iterationNumber,(int) sizeR,
             utilities_FortranMatrixMaxValue( residualNorms ) );

    mv_MultiVectorSetMask( blockVectorAX, NULL );
    mv_MultiVectorSetMask( blockVectorBX, NULL );
    mv_MultiVectorSetMask( blockVectorAP, activeMask );
    mv_MultiVectorSetMask( blockVectorBP, activeMask );
    mv_MultiVectorSetMask( blockVectorP, activeMask );
    mv_MultiVectorSetMask( blockVectorW, activeMask );

  }
  /*================== end of main loop ====================*/

  if ( exitFlag != 0 || *iterationNumber > maxIterations )
    exitFlag = REQUESTED_ACCURACY_NOT_ACHIEVED;

  (*iterationNumber)--;

  if ( verbosityLevel == 1 )
    lobpcg_dumpLambda( residualNorms, lambda,
                       iterationNumber, sizeX, util);

  mv_MultiVectorDestroy( blockVectorR );
  mv_MultiVectorDestroy( blockVectorP );
  mv_MultiVectorDestroy( blockVectorAX );
  mv_MultiVectorDestroy( blockVectorAR );
  mv_MultiVectorDestroy( blockVectorAP );
  if ( !noBFlag ) {
    mv_MultiVectorDestroy( blockVectorBX );
    mv_MultiVectorDestroy( blockVectorBR );
    mv_MultiVectorDestroy( blockVectorBP );
    if ( !noYFlag )
      mv_MultiVectorDestroy( blockVectorBY );
  }
  mv_MultiVectorDestroy( blockVectorW );

  (util->FortranMatrixDestroy)( gramA );
  (util->FortranMatrixDestroy)( gramB );
  (util->FortranMatrixDestroy)( lambdaAB );
  (util->FortranMatrixDestroy)( lambdaX );

  (util->FortranMatrixDestroy)( gramXAX );
  (util->FortranMatrixDestroy)( gramRAX );
  (util->FortranMatrixDestroy)( gramPAX );
  (util->FortranMatrixDestroy)( gramRAR );
  (util->FortranMatrixDestroy)( gramPAR );
  (util->FortranMatrixDestroy)( gramPAP );

  (util->FortranMatrixDestroy)( gramXBX );
  (util->FortranMatrixDestroy)( gramRBX );
  (util->FortranMatrixDestroy)( gramPBX );
  (util->FortranMatrixDestroy)( gramRBR );
  (util->FortranMatrixDestroy)( gramPBR );
  (util->FortranMatrixDestroy)( gramPBP );

  (util->FortranMatrixDestroy)( gramYBY );
  (util->FortranMatrixDestroy)( gramYBX );
  (util->FortranMatrixDestroy)( tempYBX );
  (util->FortranMatrixDestroy)( gramYBR );
  (util->FortranMatrixDestroy)( tempYBR );

  (util->FortranMatrixDestroy)( coordX );
  (util->FortranMatrixDestroy)( coordXX );
  (util->FortranMatrixDestroy)( coordRX );
  (util->FortranMatrixDestroy)( coordPX );

  (util->FortranMatrixDestroy)( upperR );


  (util->FortranMatrixDestroy)( lambda );
  (util->FortranMatrixDestroy)( lambdaHistory );
  (util->FortranMatrixDestroy)( lambdaColumn );
  (util->FortranMatrixDestroy)( residualNormsColumn );
  (util->FortranMatrixDestroy)( residualDiag );
  utilities_FortranMatrixDestroy( residualNorms );
  utilities_FortranMatrixDestroy( residualNormsHistory );

  free( activeMask );

  return exitFlag;
}
