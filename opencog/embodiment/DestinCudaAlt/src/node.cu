#include <cuda.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "node.h"
#include "macros.h"

#include "device_functions.h"

#define ALPHA       0.01
#define BETA        0.01
#define LAMBDA      0.5
#define GAMMA       1
#define STARVCOEFF  0.05


uint NodeStatsSize(int ni, int nb, int np)
{
    uint ns = ni + nb + np;
    return nb * ns * 2 + nb * 4;
}

// Initialize a node
void InitNode
    (
    uint         nodeIdx,
    uint         ni,
    uint         nb,
    uint         np,
    float       starvCoeff,
    float       alpha,
    float       beta,
    Node        *node_host,
    CudaNode    *cudaNode_dev,
    uint        *inputOffsets,
    uint        *inputOffsetMemory_dev,
    float       *input_dev,
    float       *belief_dev,
    float       *statsMemory_dev

    )
{

    CudaNode cudaNode_host;
    
    // calculate the state dimensionality (number of inputs + number of beliefs)
    uint ns = ni+nb+np;

    if( ns > THREADS_MAX )
    {
        fprintf(stderr, "State dimensionality is too high.\n");
        fprintf(stderr, "Node index: %d.  ni: %d. nb: %d. np: %d. ns: %d\n", nodeIdx, ni, nb, np, ns);
        exit(1);
    }

    // link host with device pointer
    node_host->node_dev = cudaNode_dev;

    // Initialize node parameters
    node_host->nb            = nb;
    node_host->ni            = ni;
    node_host->ns            = ns;
    node_host->np            = np;
    node_host->starvCoeff    = starvCoeff;
    node_host->alpha         = alpha;
    node_host->beta          = beta;
    node_host->winner        = 0;


    // allocate space on host
    MALLOC( node_host->memory_area , float , (nb * ns * 2 + nb * 4) );
    //use pointer arithmetic to divide the memory
    node_host->mu =         node_host->memory_area;
    node_host->sigma =      node_host->mu         + nb*ns;
    node_host->starv =      node_host->sigma      + nb*ns;
    node_host->pBelief =    node_host->starv      + nb;
    node_host->beliefEuc =  node_host->pBelief    + nb;
    node_host->beliefMal =  node_host->beliefEuc  + nb;

    if( inputOffsets == NULL )
    {
        MALLOC(node_host->input, float, ni);
    }
    else
    {
        node_host->input = NULL;
    }

    // Initialize node parameters
    cudaNode_host.nb            = nb;
    cudaNode_host.ni            = ni;
    cudaNode_host.np            = np;
    cudaNode_host.ns            = ns;
    cudaNode_host.starvCoeff    = starvCoeff;
    cudaNode_host.alpha         = alpha;
    cudaNode_host.beta          = beta;

    // allocate node statistics on device using pointer arithmetic to divide up the memory
    cudaNode_host.mu =          statsMemory_dev;
    cudaNode_host.sigma =       cudaNode_host.mu        + nb*ns;
    cudaNode_host.starv =       cudaNode_host.sigma     + nb*ns;
    cudaNode_host.beliefEuc =   cudaNode_host.starv     + nb;
    cudaNode_host.beliefMal =   cudaNode_host.beliefEuc + nb;
    cudaNode_host.dist =        cudaNode_host.beliefMal + nb;

    // point to the space allocated for the input (should be NULL for input nodes)
    cudaNode_host.input = input_dev;

    // copy the input offset for the inputs (should be NULL for non-input nodes)
    if( inputOffsets != NULL )
    {
        MALLOC(node_host->inputOffsets, uint, ni);
        memcpy(node_host->inputOffsets, inputOffsets, sizeof(uint) * ni);
        cudaNode_host.inputOffsets = inputOffsetMemory_dev;
        CUDAMEMCPY( cudaNode_host.inputOffsets, node_host->inputOffsets, sizeof(uint) * ni, cudaMemcpyHostToDevice);
    }
    else
    {
        node_host->inputOffsets = NULL;
        cudaNode_host.inputOffsets = NULL;
    }


    // set prior belief to block-allocated value
    cudaNode_host.pBelief = belief_dev;

    uint i,j;

    for(i=0; i < nb; i++)
    {
        // init belief (node output)
        node_host->pBelief[i] = 0;

        // init starv trace to one
        node_host->starv[i] = 1.0f;

        // init mu and sigma
        for(j=0; j < ns; j++)
        {
            node_host->mu[i*ns+j] = (float) rand() / (float) RAND_MAX;
            node_host->sigma[i*ns+j] = 0.00001;
        }
    }

    // copy initialized statistics to device
    CUDAMEMCPY( cudaNode_host.mu,       node_host->mu,      sizeof(float) * nb * ns,    cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.sigma,    node_host->sigma,   sizeof(float) * nb * ns,    cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.starv,    node_host->starv,   sizeof(float) * nb,         cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.pBelief,  node_host->pBelief, sizeof(float) * nb,         cudaMemcpyHostToDevice );

    // copy the node struct to the device
    CUDAMEMCPY( cudaNode_dev,           &cudaNode_host,     sizeof(CudaNode),           cudaMemcpyHostToDevice );
}

// deallocate the node.
void DestroyNode( Node *n )
{
    // free host data
    // free host memory for mu, sigma, starv, pBelief, beliefEuc, beliefMal
    FREE(n->memory_area);

    // if it is a zero-layer node, free the input offset array on the host
    if( n->inputOffsets != NULL)
    {
        FREE(n->inputOffsets);
    }
    else
    {
        FREE(n->input);
    }

    // free device data
    CudaNode cudaNode_host;
    CUDAMEMCPY( &cudaNode_host, n->node_dev, sizeof(CudaNode), cudaMemcpyDeviceToHost );

}

// copy the node statistics from the host to the device.
void CopyNodeToDevice(Node *host)
{
    CudaNode cudaNode_host;

    // copy struct from device
    CUDAMEMCPY( &cudaNode_host, host->node_dev, sizeof(CudaNode), cudaMemcpyDeviceToHost );

    cudaNode_host.winner = host->winner;

    // copy to pointers given from the struct
    CUDAMEMCPY( cudaNode_host.mu,        host->mu,           sizeof(float)*host->nb*host->ns,    cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.sigma,     host->sigma,        sizeof(float)*host->nb*host->ns,    cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.starv,     host->starv,        sizeof(float)*host->nb,             cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.beliefEuc, host->beliefEuc,    sizeof(float)*host->nb,             cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.beliefMal, host->beliefMal,    sizeof(float)*host->nb,             cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.pBelief,   host->pBelief,      sizeof(float)*host->nb,             cudaMemcpyHostToDevice );

    CUDAMEMCPY( host->node_dev,          &cudaNode_host,     sizeof(CudaNode),                   cudaMemcpyHostToDevice );
}

// copy the node statistics from the device to the host.
void CopyNodeFromDevice(Node *host)
{
    CudaNode cudaNode_host;

    // copy struct from device
    CUDAMEMCPY( &cudaNode_host, host->node_dev, sizeof(CudaNode), cudaMemcpyDeviceToHost );

    host->winner = cudaNode_host.winner;
    host->clustErr = cudaNode_host.clustErr;

    if( cudaNode_host.inputOffsets == NULL )
    {
        CUDAMEMCPY( host->input, cudaNode_host.input, sizeof(float)*host->ni, cudaMemcpyDeviceToHost );
    }

    // copy from pointers given from the struct
    CUDAMEMCPY( host->mu,        cudaNode_host.mu,           sizeof(float)*host->nb*host->ns,    cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->sigma,     cudaNode_host.sigma,        sizeof(float)*host->nb*host->ns,    cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->starv,     cudaNode_host.starv,        sizeof(float)*host->nb,             cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->pBelief,   cudaNode_host.pBelief,      sizeof(float)*host->nb,             cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->beliefEuc, cudaNode_host.beliefEuc,    sizeof(float)*host->nb,             cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->beliefMal, cudaNode_host.beliefMal,    sizeof(float)*host->nb,             cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->pBelief,   cudaNode_host.pBelief,      sizeof(float)*host->nb,             cudaMemcpyDeviceToHost );
}

//  CalculateDistances:
//      CUDA kernel that takes in a node and an observation
//        and writes the unnormalized belief to the particular
//        node.
//
//
//              block = (number of nodes) x (max belief dimensionality)
//             thread = (max state dimensionality) x 1
//         shared mem = euclidean sum  (length: ns)
//                      malhanobis sum (length: ns)
__global__ void CalculateDistances( CudaNode *n, float *framePtr )
{
    // shared array -- includes the euclidean and malhanobis arrays for reduction
    extern __shared__ float shared[];
    
    // value for how far the observation (or collection of beliefs from a set of previous
    // nodes) deviates from each centroid
    float delta;

    // pointers in shared memory to the euclidean and malhanobis sums
    float *sumEuc, *sumMal;

    // grab pointer to the node we want to get distances for
    n = &n[blockIdx.x];

    uint i;          // iterator for the reduction
    uint mIdx;       // entry in the mu/sigma matrix to calculate delta
    uint ns;         // node state size
    uint ni;         // node input size
    uint np;         // node parent belief size
    uint nb;         // node belief size

    ns = n->ns;
    ni = n->ni;
    nb = n->nb;
    np = n->np;

    // maxNS is likely greater than n->ns.  don't execute if we're not in range.
    // we do this because the state dimensionalities differ between nodes, but
    // we can't modify the size of the kernel that is called.  if the state
    // dimensionality is out of range, effectively execute a no-op.
    if( threadIdx.x < ns )
    {
        // point euclidean and malhanobis arrays to shared mem
        sumEuc = (float *) &shared[ns*0];
        sumMal = (float *) &shared[ns*1];

        // get entry in the mu/sigma matrices to calculate for this thread
        mIdx = blockIdx.y * ns + threadIdx.x;

        // get difference for each dimension between the input state (input + prev belief) and every
        // centroid location
        if( threadIdx.x < ni )
        {
            if( n->inputOffsets != NULL )
            {
                delta = n->mu[mIdx] - framePtr[n->inputOffsets[threadIdx.x]];
            }
            else
            {
                delta = n->mu[mIdx] - n->input[threadIdx.x];
            }
        }
        else if( threadIdx.x < ni + nb )
        {
            delta = (n->mu[mIdx] - n->pBelief[threadIdx.x-ni]) * LAMBDA;
        }
        else
        {
            if( np > 0 )
            {
                delta = (n->mu[mIdx] - n->parent_pBelief[threadIdx.x-ni-nb]) * GAMMA;
            }
        }

        delta *= n->starv[blockIdx.y];
        delta *= delta;

        syncthreads();

        sumEuc[threadIdx.x] = delta;

        if( n->sigma[mIdx] == 0 )
        {
            sumMal[threadIdx.x] = 0;
        }
        else
        {
            sumMal[threadIdx.x] = delta / n->sigma[mIdx];
        }

        // sync threads before summing up the columns
        syncthreads();

        // reduce euc and mal partial sums.  total sum will be in
        // sumEuc[0] and sumMal[0].

        for( i=1; i < ns; i <<= 1 )
        {
            if( threadIdx.x % (i*2) == 0 && threadIdx.x + i < ns )
            {
                
                sumEuc[threadIdx.x] += sumEuc[threadIdx.x+i];
                sumMal[threadIdx.x] += sumMal[threadIdx.x+i];
            }
            syncthreads();
        }
        

        // get inverse of distance (provides "confidence" or a value of
        // closeness from the centroid to the observation)
        if( threadIdx.x == 0 )
        {
            sumEuc[0] = sqrt(sumEuc[0]);
            sumMal[0] = sqrt(sumMal[0]);

            // save the euclidean distance for cluster err calculation
            n->dist[blockIdx.y] = sumEuc[0];

            if( sumEuc[0] == 0 )
            {
                n->beliefEuc[blockIdx.y] = 1;
            }
            else
            {
                n->beliefEuc[blockIdx.y] = 1 / sumEuc[0];
            }

            if( sumMal[0] == 0 )
            {
                n->beliefMal[blockIdx.y] = 1;
            }
            else
            {
                n->beliefMal[blockIdx.y] = 1 / sumMal[0];
            }
        }
    }
}

// NormalizeBelief:
//      CUDA kernel that normalizes the belief of a node such that all
//      the individual components sum to 1.  This gives each belief a
//      probability that the observation is "close" to a centroid.
//
//                block = individual node
//               thread = belief dimension for a node
//           shared mem = euclidean sum
//                        malhanobis sum

__global__ void NormalizeBelief(CudaNode *n)
{
    extern __shared__ float shared[];
    uint i;

    // grab the node we want
    n = &n[blockIdx.x];

    uint nb;         // number of centroids

    nb = n->nb;

    if( threadIdx.x < nb )
    {
        float *normEuc = (float *) &shared[nb*0];
        float *normMal = (float *) &shared[nb*1];

        // copy normEuc and normMal from global memory
        normEuc[threadIdx.x] = n->beliefEuc[threadIdx.x];
        normMal[threadIdx.x] = n->beliefMal[threadIdx.x];

        // make sure normEuc and normMal are completely populated
        syncthreads();

        // calculate the normalization constant for the belief
        for( i=1; i < nb; i <<= 1 )
        {
            if( threadIdx.x % (i*2) == 0 && threadIdx.x + i < nb )
            {
                normEuc[threadIdx.x] += normEuc[threadIdx.x + i];
                normMal[threadIdx.x] += normMal[threadIdx.x + i];
            }

            syncthreads();
        }

        // normalize the output
        if( normMal[0] == 0 )
        {
            // if the normalization const is 0, the node should
            // have no particular belief.
            n->beliefMal[threadIdx.x] = 1 / nb;
        }
        else
        {
            // otherwise, normalize the sum to 1
            n->beliefMal[threadIdx.x] /= normMal[0];
        }

        // same behavior as malhanobis normalization, see above
        if( normEuc[0] == 0 )
        {
            n->beliefEuc[threadIdx.x] = 1 / nb;
        }
        else
        {
            n->beliefEuc[threadIdx.x] /= normEuc[0];
        }

        // update belief
        n->pBelief[threadIdx.x] = n->beliefMal[threadIdx.x];
    }
}


// NormalizeBeliefGetWinner:
//      CUDA kernel that normalizes the belief (as above) and determines the winning
//        centroid
//
//                block = individual node
//               thread = belief dimension for a node
//           shared mem = euclidean sum
//                        max euclidean belief (to determine the winning centroid)
//                        malhanobis sum
//                        max euclidean belief index (to modify the winning centroid)

__global__ void NormalizeBeliefGetWinner( CudaNode *n )
{
    extern __shared__ float shared[];
    
    uint     i;

    float  *normEuc, *maxEuc, *normMal;
    uint    *maxIdx;

    // pick our particular node
    n = &n[blockIdx.x];

    uint nb;         // number of centroids

    nb = n->nb;

    if( threadIdx.x < nb )
    {
        // set up shared pointers
        normEuc = (float *) &shared[nb*0];
        normMal = (float *) &shared[nb*1];
        maxEuc = (float *) &shared[nb*2];
        maxIdx = (uint *) &shared[nb*3];

        // populate shared memory for reductions
        normEuc[threadIdx.x] = n->beliefEuc[threadIdx.x];
        normMal[threadIdx.x] = n->beliefMal[threadIdx.x];
        maxEuc[threadIdx.x] = n->beliefEuc[threadIdx.x];
        maxIdx[threadIdx.x] = threadIdx.x;

        syncthreads();

        for( i=1; i < nb; i <<= 1 )
        {
            if( threadIdx.x % (i*2) == 0 && threadIdx.x + i < nb )
            {
                // reduce euclidean and malhanobis sums
                normEuc[threadIdx.x] += normEuc[threadIdx.x + i];
                normMal[threadIdx.x] += normMal[threadIdx.x + i];

                // reduce winning centroid
                if( maxEuc[threadIdx.x] < maxEuc[threadIdx.x + i] )
                {
                    maxEuc[threadIdx.x] = maxEuc[threadIdx.x + i];
                    maxIdx[threadIdx.x] = maxIdx[threadIdx.x + i];
                } 
            }

            syncthreads();
        }
        
        // normalize the output
        if( normMal[0] == 0 )
        {
            // if the normalization const is 0, the node should
            // have no particular belief.
            n->beliefMal[threadIdx.x] = 1 / nb;
        }
        else
        {
            // otherwise, normalize the sum to 1
            n->beliefMal[threadIdx.x] /= normMal[0];
        }

        // same behavior as malhanobis normalization, see above
        if( normEuc[0] == 0 )
        {
            n->beliefEuc[threadIdx.x] = 1 / nb;
        }
        else
        {
            n->beliefEuc[threadIdx.x] /= normEuc[0];
        }
    }

    syncthreads();

    if( threadIdx.x == 0 )
    {
        n->clustErr = n->dist[maxIdx[0]];
        n->winner = maxIdx[0];
    }
}

//  UpdateWinner:
//    CUDA Kernel that updates the winning centroid
//
//              block = (number of nodes) x 1
//             thread = (max state dimensionality) x 1
__global__ void UpdateWinner( CudaNode *n, float *framePtr )
{
    // value for how far the observation (or collection of beliefs from a set of previous
    // nodes) deviates from each centroid
    float delta;

    // grab pointer to the node we want to get distances for
    n = &n[blockIdx.x];

    uint mIdx;       // entry in the mu/sigma matrix to calculate delta

    uint nb;         // number of centroids
    uint ns;         // state dimensionality
    uint ni;         // input dimensionality
    uint np;         // parent belief dimensionality
    uint winner;     // winner idx

    nb = n->nb;
    ns = n->ns;
    np = n->np;
    ni = n->ni;
    winner = n->winner;

    // maxNS is likely greater than ns.  don't execute if we're not in range.
    // we do this because the state dimensionalities differ between nodes, but
    // we can't modify the size of the kernel that is called.  if the state
    // dimensionality is out of range, effectively execute a no-op.
    if( threadIdx.x < ns )
    {
        // get entry in the mu/sigma matrices to calculate for this thread
        mIdx = winner * ns + threadIdx.x;

        // get difference for each dimension between the input state (input + prev belief) and every
        // centroid location
        if( threadIdx.x < ni )
        {
            if( n->inputOffsets != NULL )
            {
                delta = n->mu[mIdx] - framePtr[n->inputOffsets[threadIdx.x]];
            }
            else
            {
                delta = n->mu[mIdx] - n->input[threadIdx.x];
            }
        }
        else if( threadIdx.x < ni + nb )
        {
            delta = (n->mu[mIdx] - n->pBelief[threadIdx.x-ni])*LAMBDA;
        }
        else
        {
            if( np > 0 )
            {
                delta = (n->mu[mIdx] - n->parent_pBelief[threadIdx.x-ni-nb]) * GAMMA;
            }
        }
        
        // update mu and sigma
        n->mu[mIdx] -= ALPHA*delta;
        n->sigma[mIdx] -= BETA*(n->sigma[mIdx] - delta*delta);

        // update starv
        if( threadIdx.x < nb )
        {
            n->starv[threadIdx.x] = (1 - STARVCOEFF) * n->starv[threadIdx.x];

            if( threadIdx.x == n->winner )
            {
                n->starv[threadIdx.x] += STARVCOEFF;
            }
        }
    }

    // update belief
    if( threadIdx.x < nb )
    {
        n->pBelief[threadIdx.x] = n->beliefMal[threadIdx.x];
    }
}

// a quick function to print remaining memory on the card (helps to debug any
// memory leak issues)
void cudaPrintMemory()
{
    size_t mfree, mtotal;

    cudaMemGetInfo(&mfree, &mtotal);
    printf("Usage: %012zu/%012zu\n", mfree, mtotal);
}
