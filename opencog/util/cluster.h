/******************************************************************************/
/* The C Clustering Library.
 * Copyright (C) 2002 Michiel Jan Laurens de Hoon.
 *
 * This library was written at the Laboratory of DNA Information Analysis,
 * Human Genome Center, Institute of Medical Science, University of Tokyo,
 * 4-6-1 Shirokanedai, Minato-ku, Tokyo 108-8639, Japan.
 * Contact: mdehoon 'AT' gsc.riken.jp
 * 
 * Permission to use, copy, modify, and distribute this software and its
 * documentation with or without modifications and for any purpose and
 * without fee is hereby granted, provided that any copyright notices
 * appear in all copies and that both those copyright notices and this
 * permission notice appear in supporting documentation, and that the
 * names of the contributors or copyright holders not be used in
 * advertising or publicity pertaining to distribution of the software
 * without specific prior permission.
 * 
 * THE CONTRIBUTORS AND COPYRIGHT HOLDERS OF THIS SOFTWARE DISCLAIM ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT
 * OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE
 * OR PERFORMANCE OF THIS SOFTWARE.
 * 
 */

#ifndef	_C_CLUSTERING_LIBRARY_H
#define	_C_CLUSTERING_LIBRARY_H

/** \addtogroup grp_cogutil
 *  @{
 */

/** @name C Clustering Library
 * This library was written at the Laboratory of DNA Information Analysis,
 * Human Genome Center, Institute of Medical Science, University of Tokyo,
 * by Michiel Jan Laurens de Hoon
 */
///@{

#ifdef WINDOWS
#  include <windows.h>
#endif

#define CLUSTERVERSION "1.49"

/* Chapter 2 */

/**
The clusterdistance routine calculates the distance between two clusters
containing genes or microarrays using the measured gene expression vectors. The
distance between clusters, given the genes/microarrays in each cluster, can be
defined in several ways. Several distance measures can be used.

The routine returns the distance in double precision.
If the parameter transpose is set to a nonzero value, the clusters are
interpreted as clusters of microarrays, otherwise as clusters of gene.

\param nrows     (input) int
The number of rows (i.e., the number of genes) in the gene expression data
matrix.

\param ncolumns      (input) int
The number of columns (i.e., the number of microarrays) in the gene expression
data matrix.

\param data       (input) double[nrows][ncolumns]
The array containing the data of the vectors.

\param mask       (input) int[nrows][ncolumns]
This array shows which data values are missing. If mask[i][j]==0, then
data[i][j] is missing.

\param weight     (input) double[ncolumns] if transpose==0;
                   double[nrows]    if transpose==1
The weights that are used to calculate the distance.

\param n1         (input) int
The number of elements in the first cluster.

\param n2         (input) int
The number of elements in the second cluster.

\param index1     (input) int[n1]
Identifies which genes/microarrays belong to the first cluster.

\param index2     (input) int[n2]
Identifies which genes/microarrays belong to the second cluster.

\param dist       (input) char
Defines which distance measure is used, as given by the table:
dist=='e': Euclidean distance
dist=='b': City-block distance
dist=='c': correlation
dist=='a': absolute value of the correlation
dist=='u': uncentered correlation
dist=='x': absolute uncentered correlation
dist=='s': Spearman's rank correlation
dist=='k': Kendall's tau
For other values of dist, the default (Euclidean distance) is used.

\param method     (input) char
Defines how the distance between two clusters is defined, given which genes
belong to which cluster:
method=='a': the distance between the arithmetic means of the two clusters
method=='m': the distance between the medians of the two clusters
method=='s': the smallest pairwise distance between members of the two clusters
method=='x': the largest pairwise distance between members of the two clusters
method=='v': average of the pairwise distances between members of the clusters

\param transpose  (input) int
If transpose is equal to zero, the distances between the rows is
calculated. Otherwise, the distances between the columns is calculated.
The former is needed when genes are being clustered; the latter is used
when microarrays are being clustered.
*/
double clusterdistance (int nrows, int ncolumns, double** data, int** mask,
  double weight[], int n1, int n2, int index1[], int index2[], char dist,
  char method, int transpose);

/**
The distancematrix routine calculates the distance matrix between genes or
microarrays using their measured gene expression data. Several distance measures
can be used. The routine returns a pointer to a ragged array containing the
distances between the genes. As the distance matrix is symmetric, with zeros on
the diagonal, only the lower triangular half of the distance matrix is saved.
The distancematrix routine allocates space for the distance matrix. If the
parameter transpose is set to a nonzero value, the distances between the columns
(microarrays) are calculated, otherwise distances between the rows (genes) are
calculated.
If sufficient space in memory cannot be allocated to store the distance matrix,
the routine returns a NULL pointer, and all memory allocated so far for the
distance matrix is freed.


\param nrows      (input) int
The number of rows in the gene expression data matrix (i.e., the number of
genes)

\param ncolumns   (input) int
The number of columns in the gene expression data matrix (i.e., the number of
microarrays)

\param data       (input) double[nrows][ncolumns]
The array containing the gene expression data.

\param mask       (input) int[nrows][ncolumns]
This array shows which data values are missing. If mask[i][j]==0, then
data[i][j] is missing.

\param weight (input) double[n]
The weights that are used to calculate the distance. The length of this vector
is equal to the number of columns if the distances between genes are calculated,
or the number of rows if the distances between microarrays are calculated.

\param dist       (input) char
Defines which distance measure is used, as given by the table:
dist=='e': Euclidean distance
dist=='b': City-block distance
dist=='c': correlation
dist=='a': absolute value of the correlation
dist=='u': uncentered correlation
dist=='x': absolute uncentered correlation
dist=='s': Spearman's rank correlation
dist=='k': Kendall's tau
For other values of dist, the default (Euclidean distance) is used.

\param transpose  (input) int
If transpose is equal to zero, the distances between the rows is
calculated. Otherwise, the distances between the columns is calculated.
The former is needed when genes are being clustered; the latter is used
when microarrays are being clustered.
*/
double** distancematrix (int ngenes, int ndata, double** data,
  int** mask, double* weight, char dist, int transpose);

/* Chapter 3 */
/**
The getclustercentroids routine calculates the cluster centroids, given to
which cluster each element belongs. Depending on the argument method, the
centroid is defined as either the mean or the median for each dimension over
all elements belonging to a cluster.

\param nclusters  (input) int
The number of clusters.

\param nrows     (input) int
The number of rows in the gene expression data matrix, equal to the number of
genes.

\param ncolumns  (input) int
The number of columns in the gene expression data matrix, equal to the number of
microarrays.

\param data       (input) double[nrows][ncolumns]
The array containing the gene expression data.

\param mask       (input) int[nrows][ncolumns]
This array shows which data values are missing. If mask[i][j]==0, then
data[i][j] is missing.

\param clusterid  (output) int[nrows] if transpose==0
                    int[ncolumns] if transpose==1
The cluster number to which each element belongs. If transpose==0, then the
dimension of clusterid is equal to nrows (the number of genes). Otherwise, it
is equal to ncolumns (the number of microarrays).

\param cdata      (output) double[nclusters][ncolumns] if transpose==0
                    double[nrows][nclusters] if transpose==1
On exit of getclustercentroids, this array contains the cluster centroids.

\param cmask      (output) int[nclusters][ncolumns] if transpose==0
                    int[nrows][nclusters] if transpose==1
This array shows which data values of are missing for each centroid. If
cmask[i][j]==0, then cdata[i][j] is missing. A data value is missing for
a centroid if all corresponding data values of the cluster members are missing.

\param transpose  (input) int
If transpose==0, clusters of rows (genes) are specified. Otherwise, clusters of
columns (microarrays) are specified.

\param method     (input) char
For method=='a', the centroid is defined as the mean over all elements
belonging to a cluster for each dimension.
For method=='m', the centroid is defined as the median over all elements
belonging to a cluster for each dimension.

Return value
============

The function returns an integer to indicate success or failure. If a
memory error occurs, or if method is not 'm' or 'a', getclustercentroids
returns 0. If successful, getclustercentroids returns 1.
*/
int getclustercentroids(int nclusters, int nrows, int ncolumns,
  double** data, int** mask, int clusterid[], double** cdata, int** cmask,
  int transpose, char method);
  
/**
The getclustermedoids routine calculates the cluster centroids, given to which
cluster each element belongs. The centroid is defined as the element with the
smallest sum of distances to the other elements.

\param nclusters  (input) int
The number of clusters.

\param nelements  (input) int
The total number of elements.

\param distmatrix (input) double array, ragged
  (number of rows is nelements, number of columns is equal to the row number)
The distance matrix. To save space, the distance matrix is given in the
form of a ragged array. The distance matrix is symmetric and has zeros
on the diagonal. See distancematrix for a description of the content.

\param clusterid  (output) int[nelements]
The cluster number to which each element belongs.

\param centroid   (output) int[nclusters]
The index of the element that functions as the centroid for each cluster.

\param errors     (output) double[nclusters]
The within-cluster sum of distances between the items and the cluster
centroid.
*/
void getclustermedoids(int nclusters, int nelements, double** distance,
  int clusterid[], int centroids[], double errors[]);

/**
The kcluster routine performs k-means or k-median clustering on a given set of
elements, using the specified distance measure. The number of clusters is given
by the user. Multiple passes are being made to find the optimal clustering
solution, each time starting from a different initial clustering.

nclusters  (input) int
\param The number of clusters to be found.

\param data       (input) double[nrows][ncolumns]
The array containing the data of the elements to be clustered (i.e., the gene
expression data).

\param mask       (input) int[nrows][ncolumns]
This array shows which data values are missing. If
mask[i][j] == 0, then data[i][j] is missing.

\param nrows     (input) int
The number of rows in the data matrix, equal to the number of genes.

\param ncolumns  (input) int
The number of columns in the data matrix, equal to the number of microarrays.

\param weight (input) double[n]
The weights that are used to calculate the distance.

\param transpose  (input) int
If transpose==0, the rows of the matrix are clustered. Otherwise, columns
of the matrix are clustered.

\param npass      (input) int
The number of times clustering is performed. Clustering is performed npass
times, each time starting from a different (random) initial assignment of 
genes to clusters. The clustering solution with the lowest within-cluster sum
of distances is chosen.
If npass==0, then the clustering algorithm will be run once, where the initial
assignment of elements to clusters is taken from the clusterid array.

\param method     (input) char
Defines whether the arithmetic mean (method=='a') or the median
(method=='m') is used to calculate the cluster center.

\param dist       (input) char
Defines which distance measure is used, as given by the table:
dist=='e': Euclidean distance
dist=='b': City-block distance
dist=='c': correlation
dist=='a': absolute value of the correlation
dist=='u': uncentered correlation
dist=='x': absolute uncentered correlation
dist=='s': Spearman's rank correlation
dist=='k': Kendall's tau
For other values of dist, the default (Euclidean distance) is used.

\param clusterid  (output; input) int[nrows] if transpose==0
                           int[ncolumns] if transpose==1
The cluster number to which a gene or microarray was assigned. If npass==0,
then on input clusterid contains the initial clustering assignment from which
the clustering algorithm starts. On output, it contains the clustering solution
that was found.

\param error      (output) double*
The sum of distances to the cluster center of each item in the optimal k-means
clustering solution that was found.

\param ifound     (output) int*
The number of times the optimal clustering solution was
found. The value of ifound is at least 1; its maximum value is npass. If the
number of clusters is larger than the number of elements being clustered,
*ifound is set to 0 as an error code. If a memory allocation error occurs,
*ifound is set to -1.
*/
void kcluster (int nclusters, int ngenes, int ndata, double** data,
  int** mask, double weight[], int transpose, int npass, char method, char dist,
  int clusterid[], double* error, int* ifound);

/**
The kmedoids routine performs k-medoids clustering on a given set of elements,
using the distance matrix and the number of clusters passed by the user.
Multiple passes are being made to find the optimal clustering solution, each
time starting from a different initial clustering.

\param nclusters  (input) int
The number of clusters to be found.

\param nelements  (input) int
The number of elements to be clustered.

\param distmatrix (input) double array, ragged
  (number of rows is nelements, number of columns is equal to the row number)
The distance matrix. To save space, the distance matrix is given in the
form of a ragged array. The distance matrix is symmetric and has zeros
on the diagonal. See distancematrix for a description of the content.

\param npass      (input) int
The number of times clustering is performed. Clustering is performed npass
times, each time starting from a different (random) initial assignment of genes
to clusters. The clustering solution with the lowest within-cluster sum of
distances is chosen.
If npass==0, then the clustering algorithm will be run once, where the initial
assignment of elements to clusters is taken from the clusterid array.

\param clusterid  (output; input) int[nelements]
On input, if npass==0, then clusterid contains the initial clustering assignment
from which the clustering algorithm starts; all numbers in clusterid should be
between zero and nelements-1 inclusive. If npass!=0, clusterid is ignored on
input.
On output, clusterid contains the clustering solution that was found: clusterid
contains the number of the cluster to which each item was assigned. On output,
the number of a cluster is defined as the item number of the centroid of the
cluster.

\param error      (output) double
The sum of distances to the cluster center of each item in the optimal k-medoids
clustering solution that was found.

\param ifound     (output) int
If kmedoids is successful: the number of times the optimal clustering solution
was found. The value of ifound is at least 1; its maximum value is npass.
If the user requested more clusters than elements available, ifound is set
to 0. If kmedoids fails due to a memory allocation error, ifound is set to -1.
*/
void kmedoids (int nclusters, int nelements, double** distance,
  int npass, int clusterid[], double* error, int* ifound);

/* Chapter 4 */
/**
 * A Node struct describes a single node in a tree created by hierarchical
 * clustering. The tree can be represented by an array of n Node structs,
 * where n is the number of elements minus one. The integers left and right
 * in each Node struct refer to the two elements or subnodes that are joined
 * in this node. The original elements are numbered 0..nelements-1, and the
 * nodes -1..-(nelements-1). For each node, distance contains the distance
 * between the two subnodes that were joined.
 */
typedef struct {int left; int right; double distance;} Node;

/**
The treecluster routine performs hierarchical clustering using pairwise
single-, maximum-, centroid-, or average-linkage, as defined by method, on a
given set of gene expression data, using the distance metric given by dist.
If successful, the function returns a pointer to a newly allocated Tree struct
containing the hierarchical clustering solution, and NULL if a memory error
occurs. The pointer should be freed by the calling routine to prevent memory
leaks.

\param nrows     (input) int
The number of rows in the data matrix, equal to the number of genes.

\param ncolumns  (input) int
The number of columns in the data matrix, equal to the number of microarrays.

\param data       (input) double[nrows][ncolumns]
The array containing the data of the vectors to be clustered.

\param mask       (input) int[nrows][ncolumns]
This array shows which data values are missing. If mask[i][j]==0, then
data[i][j] is missing.

\param weight (input) double array[n]
The weights that are used to calculate the distance.

\param transpose  (input) int
If transpose==0, the rows of the matrix are clustered. Otherwise, columns
of the matrix are clustered.

\param dist       (input) char
Defines which distance measure is used, as given by the table:
dist=='e': Euclidean distance
dist=='b': City-block distance
dist=='c': correlation
dist=='a': absolute value of the correlation
dist=='u': uncentered correlation
dist=='x': absolute uncentered correlation
dist=='s': Spearman's rank correlation
dist=='k': Kendall's tau
For other values of dist, the default (Euclidean distance) is used.

\param method     (input) char
Defines which hierarchical clustering method is used:
method=='s': pairwise single-linkage clustering
method=='m': pairwise maximum- (or complete-) linkage clustering
method=='a': pairwise average-linkage clustering
method=='c': pairwise centroid-linkage clustering
For the first three, either the distance matrix or the gene expression data is
sufficient to perform the clustering algorithm. For pairwise centroid-linkage
clustering, however, the gene expression data are always needed, even if the
distance matrix itself is available.

\param distmatrix (input) double**
The distance matrix. If the distance matrix is zero initially, the distance
matrix will be allocated and calculated from the data by treecluster, and
deallocated before treecluster returns. If the distance matrix is passed by the
calling routine, treecluster will modify the contents of the distance matrix as
part of the clustering algorithm, but will not deallocate it. The calling
routine should deallocate the distance matrix after the return from treecluster.

Return value
============

A pointer to a newly allocated array of Node structs, describing the
hierarchical clustering solution consisting of nelements-1 nodes. Depending on
whether genes (rows) or microarrays (columns) were clustered, nelements is
equal to nrows or ncolumns. See src/cluster.h for a description of the Node
structure.
If a memory error occurs, treecluster returns NULL.
*/
Node* treecluster (int nrows, int ncolumns, double** data, int** mask,
  double weight[], int transpose, char dist, char method, double** distmatrix);


/**
The cuttree routine takes the output of a hierarchical clustering routine, and
divides the elements in the tree structure into clusters based on the
hierarchical clustering result. The number of clusters is specified by the user.

\param nelements      (input) int
The number of elements that were clustered.

\param tree           (input) Node[nelements-1]
The clustering solution. Each node in the array describes one linking event,
with tree[i].left and tree[i].right representig the elements that were joined.
The original elements are numbered 0..nelements-1, nodes are numbered
-1..-(nelements-1).

\param nclusters      (input) int
The number of clusters to be formed.

\param clusterid      (output) int[nelements]
The number of the cluster to which each element was assigned. Space for this
array should be allocated before calling the cuttree routine. If a memory
error occured, all elements in clusterid are set to -1.
*/
void cuttree (int nelements, Node* tree, int nclusters, int clusterid[]);

/* Chapter 5 */
/**
The somcluster routine implements a self-organizing map (Kohonen) on a
rectangular grid, using a given set of vectors. The distance measure to be
used to find the similarity between genes and nodes is given by dist.

\param nrows     (input) int
The number of rows in the data matrix, equal to the number of genes.

\param ncolumns  (input) int
The number of columns in the data matrix, equal to the number of microarrays.

\param data       (input) double[nrows][ncolumns]
The array containing the gene expression data.

\param mask       (input) int[nrows][ncolumns]
This array shows which data values are missing. If
mask[i][j] == 0, then data[i][j] is missing.

\param weights    (input) double[ncolumns] if transpose==0;
                   double[nrows]    if transpose==1
The weights that are used to calculate the distance. The length of this vector
is ncolumns if genes are being clustered, or nrows if microarrays are being
clustered.

\param transpose  (input) int
If transpose==0, the rows (genes) of the matrix are clustered. Otherwise,
columns (microarrays) of the matrix are clustered.

\param nxgrid    (input) int
The number of grid cells horizontally in the rectangular topology of clusters.

\param nygrid    (input) int
The number of grid cells horizontally in the rectangular topology of clusters.

\param inittau    (input) double
The initial value of tau, representing the neighborhood function.

\param niter      (input) int
The number of iterations to be performed.

\param dist       (input) char
Defines which distance measure is used, as given by the table:
dist=='e': Euclidean distance
dist=='b': City-block distance
dist=='c': correlation
dist=='a': absolute value of the correlation
dist=='u': uncentered correlation
dist=='x': absolute uncentered correlation
dist=='s': Spearman's rank correlation
dist=='k': Kendall's tau
For other values of dist, the default (Euclidean distance) is used.

\param celldata (output) double[nxgrid][nygrid][ncolumns] if transpose==0;
                  double[nxgrid][nygrid][nrows]    if tranpose==1
The gene expression data for each node (cell) in the 2D grid. This can be
interpreted as the centroid for the cluster corresponding to that cell. If
celldata is NULL, then the centroids are not returned. If celldata is not
NULL, enough space should be allocated to store the centroid data before callingsomcluster.

\param clusterid (output), int[nrows][2]    if transpose==0;
                    int[ncolumns][2] if transpose==1
For each item (gene or microarray) that is clustered, the coordinates of the
cell in the 2D grid to which the item was assigned. If clusterid is NULL, the
cluster assignments are not returned. If clusterid is not NULL, enough memory
should be allocated to store the clustering information before calling
somcluster.
*/
void somcluster (int nrows, int ncolumns, double** data, int** mask,
  const double weight[], int transpose, int nxnodes, int nynodes,
  double inittau, int niter, char dist, double*** celldata,
  int clusterid[][2]);

/* Chapter 6 */
/**
This subroutine uses the singular value decomposition to perform principal
components analysis of a real nrows by ncolumns rectangular matrix.

\param nrows     (input) int
The number of rows in the matrix u.

\param ncolumns  (input) int
The number of columns in the matrix v.

\param u          (input) double[nrows][ncolumns]
On input, the array containing the data to which the principal component
analysis should be applied. The function assumes that the mean has already been
subtracted of each column, and hence that the mean of each column is zero.
On output, see below.

\param v          (input) double[n][n], where n = min(nrows, ncolumns)
Not used on input.

\param w          (input) double[n], where n = min(nrows, ncolumns)
Not used on input.


Return value
============

On output:

If nrows >= ncolumns, then

u contains the coordinates with respect to the principal components;
v contains the principal component vectors.

The dot product u . v reproduces the data that were passed in u.


If nrows < ncolumns, then

u contains the principal component vectors;
v contains the coordinates with respect to the principal components.

The dot product v . u reproduces the data that were passed in u.

The eigenvalues of the covariance matrix are returned in w.

The arrays u, v, and w are sorted according to eigenvalue, with the largest
eigenvalues appearing first.

The function returns 0 if successful, -1 if memory allocation fails, and a
positive integer if the singular value decomposition fails to converge.
*/
int pca(int m, int n, double** u, double** v, double* w);

/**	Sets up an index table given the data, such that data[index[]] is in
 *	increasing order. Sorting is done on the indices; the array data
 *	is unchanged.
 */
void sort(int n, const double data[], int index[]);

//! compute the mean of the numbers in list
double mean(int n, double x[]);

/**
Find the median of X(1), ... , X(N), using as much of the quicksort
algorithm as is needed to isolate it.
N.B. On exit, the array X is partially ordered.
Based on Alan J. Miller's median.f90 routine.
*/
double median (int n, double x[]);

/**
This function calculates the weights using the weighting scheme proposed by
Michael Eisen:
w[i] = 1.0 / sum_{j where d[i][j]<cutoff} (1 - d[i][j]/cutoff)^exponent
where the cutoff and the exponent are specified by the user.

\param nrows      (input) int
The number of rows in the gene expression data matrix, equal to the number of
genes.

\param ncolumns   (input) int
The number of columns in the gene expression data matrix, equal to the number of
microarrays.

\param data       (input) double[nrows][ncolumns]
The array containing the gene expression data.

\param mask       (input) int[nrows][ncolumns]
This array shows which data values are missing. If mask[i][j]==0, then
data[i][j] is missing.

\param weight     (input) int[ncolumns] if transpose==0,
                   int[nrows]    if transpose==1
The weights that are used to calculate the distance. The length of this vector
is ncolumns if gene weights are being clustered, and nrows if microarrays
weights are being clustered.

\param transpose (input) int
If transpose==0, the weights of the rows of the data matrix are calculated.
Otherwise, the weights of the columns of the data matrix are calculated.

\param dist      (input) char
Defines which distance measure is used, as given by the table:
dist=='e': Euclidean distance
dist=='b': City-block distance
dist=='c': correlation
dist=='a': absolute value of the correlation
dist=='u': uncentered correlation
dist=='x': absolute uncentered correlation
dist=='s': Spearman's rank correlation
dist=='k': Kendall's tau
For other values of dist, the default (Euclidean distance) is used.

\param cutoff    (input) double
The cutoff to be used to calculate the weights.

\param exponent  (input) double
The exponent to be used to calculate the weights.


Return value
============

The function returns a pointer to a newly allocated array containing the
calculated weights for the rows (if transpose==0) or columns (if
transpose==1). If not enough memory could be allocated to store the
weights array, the function returns NULL.
*/
double* calculate_weights(int nrows, int ncolumns, double** data, int** mask,
  double weights[], int transpose, char dist, double cutoff, double exponent);


///@}
/** @}*/

#endif /* _C_CLUSTERING_LIBRARY_H */
