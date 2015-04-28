/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* This code was developed by Merico Argentati, Andrew Knyazev, Ilya Lashuk and Evgueni Ovtchinnikov */
/* test driver for double serial implementation of lobpcg */

#define MAXIT 100
#define TOL 1e-6

#include "../blopex/fortran_matrix.h"
#include "../blopex/fortran_interpreter.h"
#include "../blopex/lobpcg.h"
#include "../blopex/multivector.h"
#include "multi_vector.h"
#include "pcg_multi.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

BlopexInt dsygv_ (BlopexInt *itype, char *jobz, char *uplo, BlopexInt *
                    n, double *a, BlopexInt *lda, double *b, BlopexInt *ldb,
                    double *w, double *work, BlopexInt *lwork, BlopexInt *info);

BlopexInt dpotrf_ (char *uplo, BlopexInt *n, double *a, BlopexInt *
                    lda, BlopexInt *info);

int main(void)
{
   serial_Multi_Vector * xsoln;  // solution is here.
   serial_Multi_Vector * operatorAdata;  // raw data is here
   mv_MultiVectorPtr xx;
   double * eigs;
   double * resid;
   int iterations;
   lobpcg_Tolerance lobpcg_tol;
   mv_InterfaceInterpreter ii;
   lobpcg_BLASLAPACKFunctions blap_fn;

   int MV_HEIGHT, MV_WIDTH;

   /* create operatorAdata as a 40x40 matrix */
   operatorAdata = serial_Multi_VectorCreate(40,40);
   /* reserve space for the matrix */
   serial_Multi_VectorInitialize(operatorAdata);
   /* initialize the matrix to zeros */
   serial_Multi_VectorSetConstantValues( operatorAdata, 0.0);
   /* set diagonals of matrix to (1 2 3 ... 40) */
   double * pc = (double *)operatorAdata->data;
   int di;
   for (di=0; di<40; di++) {
     *pc = di+1;
/*
     *pc = 2;
     if (di !=0) *(pc-1) = -1;
     if (di != 39) *(pc+1) = -1;
*/
     pc = pc + 41;
   }
   /* print first part of operatorA */
   serial_Multi_VectorPrint(operatorAdata,"operatorA", 4);

/* set eigenvector length and number */
/* this is how many eigenvalues to solve for */
   MV_HEIGHT = operatorAdata->size;
   MV_WIDTH =  5;

/* create multivector */
   xsoln = serial_Multi_VectorCreate(MV_HEIGHT, MV_WIDTH);
   serial_Multi_VectorInitialize(xsoln);

/* fill it with random numbers */
   serial_Multi_VectorSetRandomValues(xsoln, 1);

/* get memory for eigenvalues, eigenvalue history, residual norms, residual norms history */

  /* request memory for eig-vals */
   eigs = (double *)malloc(sizeof(double)*MV_WIDTH);

   /* request memory for resid. norms */
   resid = (double *)malloc(sizeof(double)*MV_WIDTH);

/* set tolerances */
   lobpcg_tol.absolute = TOL;
   lobpcg_tol.relative = 1e-50;

/* setup interface interpreter and wrap around "xsoln" another structure */
   SerialSetupInterpreter( &ii );
   xx = mv_MultiVectorWrap( &ii, xsoln, 0);

/* set pointers to lapack functions */
   blap_fn.dpotrf = dpotrf_;
   blap_fn.dsygv = dsygv_;

/* execute lobpcg to solve for Ax=(lambda)Bx
   with initial guess of e-vectors xx
   and preconditioner T 

   -for this example there is no B or T

   -number of vectors in xx determines number of e-values eig to solve for
   -solving for the smallest e-values under constraints Y
   -execution stops when e-values with tolerances or after max iterations 
   
   -serial multivectors are defined by struct serial_Multi_vector in multi_vector.h
   -a MultiVectorPtr points to a mv_MultiVector struct which contains pointers to 
    the serial interface routines, which perform matrix operations on serial multivectors 
    and fortran type matries
   -these multivectors are needed because lobpcg is designed to interface with software
    packages where matrices are stored in different formats.  Internally locpcg creates and
    uses matrices fortran format.  When it must do operations involving the data in external 
    formats it calls the routines specified via the MultiVectorPtr   
   -the serial interface uses matrices in fortran format and the interface routines supplied
    are designed for this */

   printf("Call lobpcg solve double\n");

    lobpcg_solve_double(
               xx,           /*input-initial guess of e-vectors  */
                            /*     -plus pointers to interface routines */
                            /*     -a MultiVectorPtr  */
   (void *) operatorAdata,  /*input-operatorAdata (a matrix)  */
                            /*     -a pointer to a serial multivector */
                            /*     -use this parameter if data is in  */
                            /*     -the form of a matrix rather than an */
                            /*     -operator otherwise set to NULL  */
          MatMultiVec,      /*input-operatorA                  */
                            /*     -a pointer to the MatMultiVec function */
                            /*     -if operatorAdata is null this implements */
                            /*     -a call to a operator such as 1d laplacian  */
                            /*     -using X determines AX                      */
                            /*     -if operatorAdata is not null this does AX=A*X */
                            /*     -void operatorA(void * A, void * X, void * AX) */
                            /*     -parameters are all serial multivectors  */
          NULL,             /*input-operatorBdata (a matrix) */
                            /*     -a pointer to a serial multivector */
                            /*     -use this parameter if solving for generalized  */
                            /*     -eigenvalue problem and an operator for B is not used */
                            /*     -otherwise leave as NULL */
          NULL,             /*input-operatorB */
                            /*     -pointer to a function that either evaluates BY=B*Y */
                            /*     -when OperatorBdata is not null or does an operator */
                            /*     -evaluation of BY using Y                           */
                            /*     -leave as NULL if not doing generalized eigenvalue problem */
                            /*     -void operatorB(void * B, void * Y, void * BY)    */
                            /*     -parameters are are serial multivectors          */
                            /*     -must support Y being a matrix not just a column vector */
          NULL,             /*input-operatorTdata (a matrix) to use as a preconditioner */
                            /*     -pointer to a serial multivector */
                            /*     -leave as NULL if no preconditioner or if predonditioner */
                            /*     -is an operator                                          */
                            /*     -void operatorT(void * B, void * Y, void * BY)    */
                            /*     -parameters are are serial multivectors          */
                            /*     -must support Y being a matrix not just a column vector */
          NULL,             /*input-operator T */
                            /*     -pointer to a function that either evaluates R=T*W */
                            /*     -when OperatorTdata is not null or does an operator */
                            /*     -evaluation of R using W                           */
                            /*     -leave as NULL if not using preconditioner  */
                            /*     -void operatorT(void * T, void * R, void * W)    */
                            /*     -parameters are are serial multivectors          */
                            /*     -must support W being a matrix not just a column vector */
          NULL,             /*input-matrix Y */
              blap_fn,      /*input-lapack functions */
              lobpcg_tol,   /*input-tolerances */
              MAXIT,        /*input-max iterations */
              2,            /*input-verbosity level */

              &iterations,  /*output-actual iterations */
              eigs,         /*output-eigenvalues */
          NULL,             /*output-eigenvalues history */
              0,            /*output-history global height */
              resid,        /*output-residual norms */
           NULL,            /*output-residual norms history */
              0             /*output-history global height  */
    );

/* print eigenvectors */
/* serial_Multi_VectorPrint(xsoln, "eigenvectors", 0);  */

/* destroy multivector and other objects */

   serial_Multi_VectorDestroy(xsoln);
   mv_MultiVectorDestroy(xx);
   free(eigs);
   free(resid);

   return 0;
}
