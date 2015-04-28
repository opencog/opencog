/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@ BLOPEX (version 1.1) LGPL Version 2.1 or above.See www.gnu.org. */
/* @@@ Copyright 2010 BLOPEX team http://code.google.com/p/blopex/     */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* This code was developed by Merico Argentati, Andrew Knyazev, Ilya Lashuk and Evgueni Ovtchinnikov */

#ifndef BlopexInt
#define BlopexInt int
#endif

#ifndef serial_MULTI_VECTOR_HEADER
#define serial_MULTI_VECTOR_HEADER

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------------------------------------------
 * serial_Multi_Vector
 *--------------------------------------------------------------------------*/

typedef struct
{
   double  *data;
   BlopexInt      size;
   BlopexInt      owns_data;
   BlopexInt      num_vectors;  /* the above "size" is size of one vector */

   BlopexInt      num_active_vectors;
   BlopexInt     *active_indices;   /* indices of active vectors; 0-based notation */

} serial_Multi_Vector ;

/*--------------------------------------------------------------------------
 * Accessor functions for the Multi_Vector structure
 *--------------------------------------------------------------------------*/

#define serial_Multi_VectorData(vector)      ((vector) -> data)
#define serial_Multi_VectorSize(vector)      ((vector) -> size)
#define serial_Multi_VectorOwnsData(vector)  ((vector) -> owns_data)
#define serial_Multi_VectorNumVectors(vector) ((vector) -> num_vectors)

serial_Multi_Vector * serial_Multi_VectorCreate( BlopexInt size, BlopexInt num_vectors  );
serial_Multi_Vector *serial_Multi_VectorRead( char *file_name );

BlopexInt serial_Multi_VectorDestroy( serial_Multi_Vector *vector );
BlopexInt serial_Multi_VectorInitialize( serial_Multi_Vector *vector );
BlopexInt serial_Multi_VectorSetDataOwner(serial_Multi_Vector *vector , BlopexInt owns_data);
/*
BlopexInt serial_Multi_VectorPrint( serial_Multi_Vector *vector, const char *file_name );
*/
BlopexInt serial_Multi_VectorPrint( serial_Multi_Vector *vector, char * tag, BlopexInt limit);

BlopexInt serial_Multi_VectorSetConstantValues(serial_Multi_Vector *v,double value);
BlopexInt serial_Multi_VectorSetRandomValues(serial_Multi_Vector *v , BlopexInt seed);
BlopexInt serial_Multi_VectorCopy( serial_Multi_Vector *x , serial_Multi_Vector *y);
BlopexInt serial_Multi_VectorScale( double alpha , serial_Multi_Vector *y, BlopexInt *mask  );
BlopexInt serial_Multi_VectorAxpy( double alpha , serial_Multi_Vector *x , serial_Multi_Vector *y);
BlopexInt serial_Multi_VectorInnerProd( serial_Multi_Vector *x,
                                  serial_Multi_Vector *y,
                                  BlopexInt gh, BlopexInt h, BlopexInt w, double* v);
BlopexInt serial_Multi_VectorMultiScale( double *alpha, serial_Multi_Vector *v, BlopexInt *mask );

BlopexInt serial_Multi_VectorByDiag( serial_Multi_Vector *x,
                                 BlopexInt                *mask,
                                 BlopexInt                n,
                                 double             *alpha,
                                 serial_Multi_Vector *y);

BlopexInt serial_Multi_VectorInnerProdDiag( serial_Multi_Vector *x,
                                      serial_Multi_Vector *y,
                      BlopexInt* mask, BlopexInt n, double* diag);

BlopexInt
 serial_Multi_VectorSetMask(serial_Multi_Vector *mvector, BlopexInt * mask);
BlopexInt
 serial_Multi_VectorCopyWithoutMask(serial_Multi_Vector *x , serial_Multi_Vector *y);
BlopexInt
 serial_Multi_VectorByMatrix(serial_Multi_Vector *x, BlopexInt rGHeight, BlopexInt rHeight,
                                 BlopexInt rWidth, double* rVal, serial_Multi_Vector *y);
BlopexInt
 serial_Multi_VectorByMulti_Vector(serial_Multi_Vector *x,
                                   serial_Multi_Vector *y,
                                   serial_Multi_Vector *z);
#ifdef __cplusplus
}
#endif

#endif /* serial_MULTI_VECTOR_HEADER */
