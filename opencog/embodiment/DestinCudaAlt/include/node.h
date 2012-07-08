#ifndef __NODE_H
#define __NODE_H

/* Node Struct Definition */
struct Node {
    /* HOST VARIABLES BEGIN */
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
    float   clustErr;

    uint     winner;


    //holds the memory for mu, sigma, starv, beliefEuc, beliefMal, pBelief
    float * memory_area;

    // node statistics
    float * mu;
    float * sigma;
    float * starv;
    
    // node beliefs
    float * input;
    float * beliefEuc;
    float * beliefMal;
    float * pBelief;

    float * parent_pBelief;

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
    float   clustErr;

    uint     winner;

    // node statistics
    float * mu;
    float * sigma;
    float * starv;
    float * dist;
    
    // node beliefs
    float * input;
    float * beliefEuc;
    float * beliefMal;
    float * pBelief;
    float * parent_pBelief;
};
/* Node Struct Definition End */


/* Node Functions Begin */
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
                 float *,               // pointer to input on device
                 float *,               // pointer to belief on device
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
__global__ void CalculateDistances(
                    CudaNode *,         // pointer to the list of nodes
                    float *             // pointer to the frame
                );

__global__ void NormalizeBelief(
                    CudaNode *          // pointer to the list of nodes
                );

__global__ void NormalizeBeliefGetWinner(
                    CudaNode *          // pointer to the list of nodes
                );

__global__ void UpdateWinner(
                    CudaNode *n,        // pointer to the list of nodes
                    float *framePtr     // pointer to the frame
                );
#endif
void cudaPrintMemory();

#endif
