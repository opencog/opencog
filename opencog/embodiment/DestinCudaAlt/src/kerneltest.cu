#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#include "macros.h"
#include "node.h"
#include "destin.h"

void KernelTest()
{
    Node *n;

    uint nb, ni, ns;
    uint *inputOffsets = NULL;
    float *input_dev, *belief_dev;

    float *input_host;

    // allocate node
    MALLOC( n, Node, 1 );
    CUDAMALLOC( (void **) &n->node_dev, sizeof(CudaNode) );

    // give a random belief and input dimensionality
    nb = rand() % 190 + 10;
    ni = rand() % 190 + 10;
    ns = nb + ni;

    // allocate input and belief for node
    CUDAMALLOC( (void **) &input_dev, sizeof(float) * ni );
    CUDAMALLOC( (void **) &belief_dev, sizeof(float) * nb );

    float * stat_mem;
    CUDAMALLOC( (void **)&stat_mem, sizeof(float) * NodeStatsSize(ni, nb, 0) );

    InitNode( 0, ni, nb, 0, 0.1, 0.01, 0.1, n, n->node_dev, inputOffsets, input_dev, belief_dev, stat_mem);

    // allocate input frame
    MALLOC( input_host, float, ni );

    uint i,j;
    for( i=0; i < ni; i++ )
    {
        input_host[i] = (float) rand() / RAND_MAX;
    }

    CUDAMEMCPY( input_dev, input_host, sizeof(float) * ni, cudaMemcpyHostToDevice );

    dim3 blocksize( 1, nb );

    CalculateDistances<<< blocksize, ns, sizeof(float)*2*ns >>>( n->node_dev, NULL );
    //NormalizeBelief<<< 1, nb, sizeof(float)*2*nb >>>( n->node_dev );
    NormalizeBeliefGetWinner<<< 1, nb, sizeof(float)*4*nb >>>( n->node_dev );
    CopyNodeFromDevice( n );

    float delta;
    float *dist;

    dist = (float *) malloc(sizeof(float) * nb);

    float errSum = 0;
    float distSum = 0;
    float err;

    for( i=0; i < nb; i++ )
    {
        dist[i] = 0;
        for( j=0; j < ns; j++ )
        {
            if( j < ni )
            {
                delta = n->mu[i*ns+j] - input_host[j];
            }
            else
            {
                delta = (n->mu[i*ns+j] - n->pBelief[j - ni]) * 0.5;
            }

            dist[i] += delta * delta;
        }

        dist[i] = 1 / dist[i];

        distSum += dist[i];

        err = (dist[i] - n->beliefEuc[i]) * (dist[i] - n->beliefEuc[i]);
        errSum += err;

        printf("  %0.3f\n", err);
    }

    float beliefEucSum = 0;

    for( i=0; i < nb; i++ ) 
    {
        dist[i] /= distSum;
        beliefEucSum += n->beliefEuc[i];
    }

    printf("MSE: %0.2f\n", errSum / nb);

    free(dist);
 
    CUDAFREE( input_dev );
    CUDAFREE( belief_dev );
    CUDAFREE( stat_mem );
    DestroyNode( n );
}

int main()
{
    srand(time(NULL));
    uint i;
    for( i=0; i < 10000; i++ )
    {
        KernelTest();
    }
}
