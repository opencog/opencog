#ifndef __NODE_H
#define __NODE_H

/* Node Struct Definition */
struct Node {
    /* HOST VARIABLES BEGIN */

    // node parameters
    uint     nb;            // number of beliefs
    uint     ni;            // number of inputs
    uint     ns;            // number of states
    uint     np;            // number of beliefs for parent
    float   starvCoeff;     // starvation coefficient
    float   alpha;          // mu update weight (centroid location)
    float   beta;           // sigma update weight (centroid variance)

    uint     winner;        // winning centroid index

    //holds the memory for mu, sigma, starv, beliefEuc, beliefMal, pBelief
    float * memory_area;

    // node statistics
    float * mu;             // centroid locations
    float * sigma;          // centroid variances
    float * starv;          // centroid starvation coefficients
    
    // node input
    float * input;          // input pointer (null for input layer nodes)
    uint  * inputOffsets;   // offsets for each pixel taken from framePtr for this node
                            // (null for non-input layer nodes)
    float * observation;    // contains the node's input, previous 
                            // belief, and parent's previous belief
    
    // node beliefs
    float * beliefEuc;      // belief (euclidean distance)
    float * beliefMal;      // belief (malhanobis distance)
    float * pBelief;        // previous belief (euclidean)
    float * parent_pBelief; // parent previous belief

    /* HOST VARIABLES END */

    /* DEVICE VARIABLES BEGIN */
    struct CudaNode *node_dev;
    /* DEVICE VARIABLES END */
};

struct CudaNode {
    // node identifier
    uint   * inputOffsets;

    // node parameters
    uint     nb;
    uint     ni;
    uint     ns;
    uint     np;
    float   starvCoeff;
    float   alpha;
    float   beta;

    uint     winner;

    // node statistics
    float * mu;
    float * sigma;
    float * starv;
    
    // node input
    float * observation;
    float * input;
    
    // node beliefs
    float * beliefEuc;
    float * beliefMal;
    float * pBelief;
    float * parent_pBelief;
};
/* Node Struct Definition End */


/* Node Functions Begin */
void   PrintNode( Node * );             // print the node!!!!

void   InitNode(                        // initialize a node.
                 uint,                  // node index
                 uint,                  // belief dimensionality (# centroids)
                 uint,                  // input dimensionality (# input values)
                 uint,                  // parent belief dimensionality
                 float,                 // starvation coefficient
                 float,                 // alpha (mu step size)
                 float,                 // beta (sigma step size)
                 Node *,                // pointer node on host
                 CudaNode *,            // pointer to node on device
                 uint *,                // input offsets from input image (NULL for any non-input node)
                 uint *,                // chunk of preallocated cuda memory to put the input offsets
                 float *,               // pointer to input on device
                 float *,               // pointer to input on host
                 float *,               // pointer to belief on device
                 float *,               // pointer to belief on host
                 float *                // pointer to stats memory area on device
                );

void DestroyNode(
                 Node *
                );

void CopyNodeToDevice(                  // copy node in host mem to device mem
                 Node *                 // node pointer
                );

void CopyNodeFromDevice(                // copy node from device mem to host mem
                 Node *                 // node pointer
                );

uint NodeStatsSize(                     // calculate the size of a node's stats memory area
                int ni,                 // node input dimensionality
                int nb,                 // number of node beliefs
                int np                  // number of node's parent's beliefs
               );
/* Node Node Functions End */

/* CUDA device function definitions */
//

//make SWIG ignore these when generating the bindings.
#ifdef __global__

__global__ void GetObservation(
                    CudaNode *,         // pointer to the list of nodes
                    float *             // pointer to input frame
                );

__global__ void CalculateDistances(
                    CudaNode *          // pointer to the list of nodes
                );

__global__ void NormalizeBelief(
                    CudaNode *          // pointer to the list of nodes
                );

__global__ void NormalizeBeliefGetWinner(
                    CudaNode *          // pointer to the list of nodes
                );

__global__ void UpdateWinner(
                    CudaNode *n         // pointer to the list of nodes
                );
#endif
void cudaPrintMemory();

/* CPU implementation of CUDA kernels */
void __CPU_GetObservation(
                    Node *,             // pointer to list of nodes
                    float *,            // pointer to input frame
                    uint                // node index
                );

void __CPU_CalculateDistances(
                    Node *,             // pointer to list of nodes
                    uint                // node index
                );

void __CPU_NormalizeBelief(
                    Node *,             // pointer to list of nodes
                    uint                // node index
                );

void __CPU_NormalizeBeliefGetWinner(
                    Node *,             // pointer to list of nodes
                    uint                // node index
                );

void __CPU_UpdateWinner(
                    Node *,             // pointer to list of nodes
                    uint                // node index
                );

#endif
