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
#define EPSILON     1e-25
#define LARGE_FLOAT 10

void PrintNode( Node *n )
{
    CudaNode cn;

    printf("node statistics\n");
    printf("nb: 0x%x    ni: 0x%x    np: 0x%x    ns: 0x%x    nb*ns: 0x%x\n", n->nb, n->ni, n->np, n->ns, n->nb*n->ns);

/*
    printf("Host pointers:\n");
    printf("mu: %p\n", n->mu);
    printf("sigma: %p\n", n->sigma);
    printf("starv: %p\n", n->starv);
    printf("input: %p\n", n->input);
    printf("inputOffsets: %p\n", n->inputOffsets);
    printf("observation: %p\n", n->mu);
    printf("beliefEuc: %p\n", n->beliefEuc);
    printf("beliefMal: %p\n", n->beliefMal);
    printf("pBelief: %p\n", n->pBelief);
    printf("parent_pBelief: %p\n\n", n->parent_pBelief);
*/

    uint nb = n->nb;
    uint ni = n->ni;
    uint np = n->np;
    uint ns = n->ns;

    uint muSize = n->nb * n->ns;

    CUDAMEMCPY( &cn, n->node_dev, sizeof(CudaNode), cudaMemcpyDeviceToHost );

    printf("Device pointers:\n");
    printf("mu: %p to %p\n", cn.mu, cn.mu + muSize);
    printf("sigma: %p to %p\n", cn.sigma, &cn.sigma[muSize]);
    printf("starv: %p to %p\n", cn.starv, &cn.starv[nb] );
    printf("beliefEuc: %p to %p\n", cn.beliefEuc, &cn.beliefEuc[nb]);
    printf("beliefMal: %p to %p\n", cn.beliefMal, &cn.beliefMal[nb]);
    printf("observation: %p to %p\n", cn.observation, &cn.observation[ns] );

    printf("input: %p to %p\n", cn.input, &cn.input[ni] );
    printf("inputOffsets: %p to %p\n", cn.inputOffsets, &cn.inputOffsets[ni] );

    printf("pBelief: %p to %p\n", cn.pBelief, &cn.pBelief[nb] );
    printf("parent_pBelief: %p to %p\n", cn.parent_pBelief, &cn.parent_pBelief[np]);
}

uint NodeStatsSize(int ni, int nb, int np)
{
    uint ns = ni + nb + np;
    return nb * ns * 2 + nb * 3 + ns;
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
    float       *input_host,
    float       *belief_dev,
    float       *belief_host,
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
    node_host->np            = np;
    node_host->ns            = ns;
    node_host->starvCoeff    = starvCoeff;
    node_host->alpha         = alpha;
    node_host->beta          = beta;
    node_host->winner        = 0;

    // allocate space on host
    MALLOC( node_host->memory_area , float , (nb * ns * 2 + nb * 3 + ns) );

    //use pointer arithmetic to divide the memory
    node_host->mu =          node_host->memory_area;
    node_host->sigma =       node_host->mu         + nb*ns;
    node_host->starv =       node_host->sigma      + nb*ns;
    node_host->beliefEuc =   node_host->starv      + nb;
    node_host->beliefMal =   node_host->beliefEuc  + nb;
    node_host->observation = node_host->beliefMal  + nb;

    // point to the block-allocated space
    node_host->input = input_host;
    node_host->pBelief = belief_host;

    // Initialize node parameters
    cudaNode_host.nb            = nb;
    cudaNode_host.ni            = ni;
    cudaNode_host.np            = np;
    cudaNode_host.ns            = ns;
    cudaNode_host.starvCoeff    = starvCoeff;
    cudaNode_host.alpha         = alpha;
    cudaNode_host.beta          = beta;
    cudaNode_host.winner        = 0;

    // allocate node statistics on device using pointer arithmetic to divide up the memory
    cudaNode_host.mu =          statsMemory_dev;
    cudaNode_host.sigma =       cudaNode_host.mu        + nb*ns;
    cudaNode_host.starv =       cudaNode_host.sigma     + nb*ns;
    cudaNode_host.beliefEuc =   cudaNode_host.starv     + nb;
    cudaNode_host.beliefMal =   cudaNode_host.beliefEuc + nb;
    cudaNode_host.observation = cudaNode_host.beliefMal + nb;

    // point to the block-allocated space 
    cudaNode_host.input = input_dev;
    cudaNode_host.pBelief = belief_dev;

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

    uint i,j;
    for( i=0; i < nb; i++ )
    {
        // init belief (node output)
        node_host->pBelief[i] = 1 / nb;
        node_host->beliefEuc[i] = 1 / nb;
        node_host->beliefMal[i] = 1 / nb;

        // init starv trace to one
        node_host->starv[i] = 1.0f;

        // init mu and sigma
        for(j=0; j < ns; j++)
        {
            node_host->mu[i*ns+j] = (float) rand() / (float) RAND_MAX;
            node_host->sigma[i*ns+j] = 0.00001;
        }
    }

    for( i=0; i < ns; i++ )
    {
        node_host->observation[i] = 0;
    }

    // copy initialized statistics to device
    CUDAMEMCPY( cudaNode_host.mu,           node_host->mu,          sizeof(float) * nb * ns,    cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.sigma,        node_host->sigma,       sizeof(float) * nb * ns,    cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.starv,        node_host->starv,       sizeof(float) * nb,         cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.pBelief,      node_host->pBelief,     sizeof(float) * nb,         cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.observation,  node_host->observation, sizeof(float) * ns,         cudaMemcpyHostToDevice );

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
    CUDAMEMCPY( cudaNode_host.mu,           host->mu,           sizeof(float)*host->nb*host->ns,    cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.sigma,        host->sigma,        sizeof(float)*host->nb*host->ns,    cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.starv,        host->starv,        sizeof(float)*host->nb,             cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.pBelief,      host->pBelief,      sizeof(float)*host->nb,             cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.beliefEuc,    host->beliefEuc,    sizeof(float)*host->nb,             cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.beliefMal,    host->beliefMal,    sizeof(float)*host->nb,             cudaMemcpyHostToDevice );
    CUDAMEMCPY( cudaNode_host.observation,  host->observation,  sizeof(float)*host->ns,             cudaMemcpyHostToDevice );

    CUDAMEMCPY( host->node_dev,          &cudaNode_host,     sizeof(CudaNode),                   cudaMemcpyHostToDevice );
}

// copy the node statistics from the device to the host.
void CopyNodeFromDevice(Node *host)
{
    CudaNode cudaNode_host;

    // copy struct from device
    CUDAMEMCPY( &cudaNode_host, host->node_dev, sizeof(CudaNode), cudaMemcpyDeviceToHost );

    host->winner = cudaNode_host.winner;

    if( cudaNode_host.inputOffsets == NULL )
    {
        CUDAMEMCPY( host->input, cudaNode_host.input, sizeof(float)*host->ni, cudaMemcpyDeviceToHost );
    }

    // copy from pointers given from the struct
    CUDAMEMCPY( host->mu,           cudaNode_host.mu,           sizeof(float)*host->nb*host->ns,    cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->sigma,        cudaNode_host.sigma,        sizeof(float)*host->nb*host->ns,    cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->starv,        cudaNode_host.starv,        sizeof(float)*host->nb,             cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->pBelief,      cudaNode_host.pBelief,      sizeof(float)*host->nb,             cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->beliefEuc,    cudaNode_host.beliefEuc,    sizeof(float)*host->nb,             cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->beliefMal,    cudaNode_host.beliefMal,    sizeof(float)*host->nb,             cudaMemcpyDeviceToHost );
    CUDAMEMCPY( host->observation,  cudaNode_host.observation,  sizeof(float)*host->ns,             cudaMemcpyDeviceToHost );
}

// GetObservation:
//      CUDA kernel that creates the observation for a node
//      given its input (input image or a previous node's belief),
//      previous belief, and parent's belief.
__global__ void GetObservation( CudaNode *n, float *framePtr )
{
    n = &n[blockIdx.x];

    uint ns = n->ns;
    uint ni = n->ni;
    uint nb = n->nb;
    uint np = n->np;

    if( threadIdx.x < ns )
    {
        if( threadIdx.x < ni )
        {
            if( n->inputOffsets == NULL )
            {
                n->observation[threadIdx.x] = n->input[threadIdx.x];
            }
            else
            {
                n->observation[threadIdx.x] = framePtr[n->inputOffsets[threadIdx.x]];
            }
        }
        else if( threadIdx.x < ni + nb )
        {
            n->observation[threadIdx.x] = n->pBelief[threadIdx.x-ni];
        }
        else
        {
            if( np > 0 )
            {
                n->observation[threadIdx.x] = n->parent_pBelief[threadIdx.x-ni-nb];
            }
        }
    }
}

// CPU implementation of GetObservation kernel
void __CPU_GetObservation( Node *n, float *framePtr, uint nIdx )
{
    n = &n[nIdx];

    uint i;

    for( i=0; i < n->ni; i++ )
    {
        n->observation[i] = (n->inputOffsets == NULL) ? n->input[i] : framePtr[n->inputOffsets[i]];
    }

    for( i=n->ni; i < n->ni+n->nb; i++ )
    {
        n->observation[i] = n->pBelief[i-n->ni];
    }

    for( i=n->ni+n->nb; i < n->ns; i++ )
    {
        n->observation[i] = n->parent_pBelief[i-n->ni-n->nb];
    }
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
__global__ void CalculateDistances( CudaNode *n )
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

    ns = n->ns;

    // maxNS is likely greater than n->ns.  don't execute if we're not in range.
    // we do this because the state dimensionalities differ between nodes, but
    // we can't modify the size of the kernel that is called.  if the state
    // dimensionality is out of range, effectively execute a no-op.
    if( blockIdx.y < n->nb && threadIdx.x < ns )
    {
        // point euclidean and malhanobis arrays to shared mem
        sumEuc = (float *) &shared[ns*0];
        sumMal = (float *) &shared[ns*1];

        // get entry in the mu/sigma matrices to calculate for this thread
        mIdx = blockIdx.y * ns + threadIdx.x;

        // get difference for each dimension between the input state (input + prev belief) and every
        // centroid location
        delta = n->mu[mIdx] - n->observation[threadIdx.x];

        delta *= n->starv[blockIdx.y];
        delta *= delta;

        sumEuc[threadIdx.x] = delta;
        sumMal[threadIdx.x] = (n->sigma[mIdx] < EPSILON) ? LARGE_FLOAT  : (delta / n->sigma[mIdx]);

        // sync threads before summing up the columns
        __syncthreads();

        // reduce euc and mal partial sums.  total sum will be in
        // sumEuc[0] and sumMal[0].
        // ** THIS BLOCK NEEDS WORK **
        // It is awful for two reasons -- it is highly divergent,
        // and it causes a huge number of bank conflicts.
        for( i=1; i < ns; i <<= 1 )
        {
            if( threadIdx.x % (i*2) == 0 && threadIdx.x + i < ns )
            {
        /*
        for( i = ns / 2; i > 0; i >>= 1 )
        {
            if( threadIdx.x < i && threadIdx.x+i < ns )
            {
        */
                sumEuc[threadIdx.x] += sumEuc[threadIdx.x+i];
                sumMal[threadIdx.x] += sumMal[threadIdx.x+i];
            }
            __syncthreads();
        }
        
        // get inverse of distance (provides "confidence" or a value of
        // closeness from the centroid to the observation)
        if( threadIdx.x == 0 )
        {
            sumEuc[0] = sqrt(sumEuc[0]);
            sumMal[0] = sqrt(sumMal[0]);

            n->beliefEuc[blockIdx.y] = (sumEuc[0] < EPSILON) ? 1 : (1 / sumEuc[0]);
            n->beliefMal[blockIdx.y] = (sumMal[0] < EPSILON) ? 1 : (1 / sumMal[0]);
        }
    }
}

// CPU implementation of CalculateDistances kernel
void __CPU_CalculateDistances( Node *n, uint nIdx )
{
    float delta;
    float sumEuc, sumMal;

    n = &n[nIdx];

    uint i, j;

    // iterate over each belief
    for( i=0; i < n->nb; i++ )
    {
        sumEuc = 0;
        sumMal = 0;

        // iterate over each state for belief
        for( j=0; j < n->ns; j++ )
        {
            delta = n->mu[i*n->ns+j] - n->observation[j];

            sumEuc += delta * delta * n->starv[i];
            sumMal += delta * delta * n->starv[i] / n->sigma[i*n->ns+j];
        }

        sumEuc = sqrt(sumEuc);
        sumMal = sqrt(sumMal);

        n->beliefEuc[i] = ( sumEuc < EPSILON ) ? 1 : (1 / sumEuc);
        n->beliefMal[i] = ( sumMal < EPSILON ) ? 1 : (1 / sumMal);
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
        __syncthreads();

        // calculate the normalization constant for the belief
        // this reduction is slow and awful, see above
        for( i=1; i < nb; i <<= 1 )
        {
            if( threadIdx.x % (i*2) == 0 && threadIdx.x + i < nb )
            {
        /*
        for( i = nb / 2; i > 0; i >>= 1 )
        {
            if( threadIdx.x < i && threadIdx.x+i < nb )
            {
        */
                normEuc[threadIdx.x] += normEuc[threadIdx.x + i];
                normMal[threadIdx.x] += normMal[threadIdx.x + i];
            }

            __syncthreads();
        }

        n->beliefEuc[threadIdx.x] = ( normEuc[0] < EPSILON ) ? (1 / (float) n->nb) : (n->beliefEuc[threadIdx.x] / normEuc[0]);
        n->beliefMal[threadIdx.x] = ( normMal[0] < EPSILON ) ? (1 / (float) n->nb) : (n->beliefMal[threadIdx.x] / normMal[0]);

        // update belief
        n->pBelief[threadIdx.x] = n->beliefMal[threadIdx.x];
    }
}

// CPU implementation of NormalizeBelief kernel
void __CPU_NormalizeBelief( Node *n, uint nIdx )
{
    n = &n[nIdx];
    
    float normEuc = 0;
    float normMal = 0;

    uint i;

    for( i=0; i < n->nb; i++ )
    {
        normEuc += n->beliefEuc[i];
        normMal += n->beliefMal[i];
    }

    for( i=0; i < n->nb; i++ )
    {
        n->beliefEuc[i] = ( normEuc < EPSILON ) ? (1 / (float) n->nb) : (n->beliefEuc[i] / normEuc);
        n->beliefMal[i] = ( normMal < EPSILON ) ? (1 / (float) n->nb) : (n->beliefMal[i] / normMal);
        n->pBelief[i] = n->beliefMal[i];
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

        __syncthreads();

        // this reduction is slow and awful, see above
        for( i=1; i < nb; i <<= 1 )
        {
            if( threadIdx.x % (i*2) == 0 && threadIdx.x + i < nb )
            {
        /*
        for( i = nb / 2; i > 0; i >>= 1 )
        {
            if( threadIdx.x < i && threadIdx.x+i < nb )
            {
        */
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

            __syncthreads();
        }
        
        n->beliefEuc[threadIdx.x] = ( normEuc[0] < EPSILON ) ? (1 / n->nb) : (n->beliefEuc[threadIdx.x] / normEuc[0]);
        n->beliefMal[threadIdx.x] = ( normMal[0] < EPSILON ) ? (1 / n->nb) : (n->beliefMal[threadIdx.x] / normMal[0]);

        if( threadIdx.x == 0 )
        {
            n->winner = maxIdx[0];
        }
    }
}


// CPU implementation of NormalizeBelief kernel
void __CPU_NormalizeBeliefGetWinner( Node *n, uint nIdx )
{
    n = &n[nIdx];
    
    float normEuc = 0;
    float normMal = 0;

    float maxEucVal = n->beliefEuc[0];
    uint maxEucIdx = 0;

    uint i;

    for( i=0; i < n->nb; i++ )
    {
        normEuc += n->beliefEuc[i];
        normMal += n->beliefMal[i];

        if( n->beliefEuc[i] > maxEucVal )
        {
            maxEucVal = n->beliefEuc[i];
            maxEucIdx = i;
        }
    }

    for( i=0; i < n->nb; i++ )
    {
        n->beliefEuc[i] = ( normEuc < EPSILON ) ? (1 / n->nb) : (n->beliefEuc[i] / normEuc);
        n->beliefMal[i] = ( normMal < EPSILON ) ? (1 / n->nb) : (n->beliefMal[i] / normMal);
    }

    n->winner = maxEucIdx;
}

//  UpdateWinner:
//    CUDA Kernel that updates the winning centroid
//
//              block = (number of nodes) x 1
//             thread = (max state dimensionality) x 1
__global__ void UpdateWinner( CudaNode *n )
{
    // value for how far the observation (or collection of beliefs from a set of previous
    // nodes) deviates from each centroid
    float delta;

    // grab pointer to the node we want to get distances for
    n = &n[blockIdx.x];

    uint mIdx;       // entry in the mu/sigma matrix to calculate delta

    uint nb;         // number of centroids
    uint ns;         // state dimensionality
    uint winner;     // winner idx

    nb = n->nb;
    ns = n->ns;
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
        delta = n->mu[mIdx] - n->observation[threadIdx.x];

        // update mu and sigma
        n->mu[mIdx] -= ALPHA*delta;
        n->sigma[mIdx] -= BETA*(n->sigma[mIdx] - delta*delta);
    }

    // update starvation and belief
    if( threadIdx.x < nb )
    {
        n->starv[threadIdx.x] *= 1 - STARVCOEFF;

        if( threadIdx.x == n->winner )
        {
            n->starv[threadIdx.x] += STARVCOEFF;
        }

        n->pBelief[threadIdx.x] = n->beliefMal[threadIdx.x];
    }
}

// CPU implementation for UpdateWinner kernel
void __CPU_UpdateWinner( Node *n, uint nIdx )
{
    n = &n[nIdx];

    uint i;
    uint winnerOffset = n->winner*n->ns;
    float delta;

    for( i=0; i < n->ns; i++ )
    {
        delta = n->mu[winnerOffset+i] - n->observation[i];

        n->mu[winnerOffset+i] -= ALPHA * delta;
        n->sigma[winnerOffset+i] -= BETA * (n->sigma[winnerOffset+i] - delta*delta);
    }

    for( i=0; i < n->nb; i++ )
    {
        n->starv[i] *= 1 - STARVCOEFF;
        n->pBelief[i] = n->beliefMal[i];
    }

    n->starv[n->winner] += STARVCOEFF;
}

// a quick function to print remaining memory on the card (helps to debug any
// memory leak issues)
void cudaPrintMemory()
{
    size_t mfree, mtotal;

    cudaMemGetInfo(&mfree, &mtotal);
    printf("Usage: %012zu/%012zu\n", mfree, mtotal);
}
