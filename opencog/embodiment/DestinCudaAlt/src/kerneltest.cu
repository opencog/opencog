#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "macros.h"
#include "node.h"
#include "destin.h"

#define NLAYERS 3

void GetMSE( Destin *d1, Destin *d2 )
{
    if( d1->nNodes != d2->nNodes )
    {
        fprintf(stderr, "networks differ in # nodes!\n");
        exit(1);
    }

    float muMSE, sigmaMSE, starvMSE, pBeliefMSE, diff;
    float muMSE_Sum, sigmaMSE_Sum, starvMSE_Sum, pBeliefMSE_Sum;

    uint n, i, j;


    Node *n1, *n2;
    muMSE_Sum = sigmaMSE_Sum = starvMSE_Sum = pBeliefMSE_Sum = 0;

    for( n=0; n < d1->nNodes; n++ )
    {
        n1 = &(d1->nodes_host[n]);
        n2 = &(d2->nodes_host[n]);

        muMSE = sigmaMSE = starvMSE = pBeliefMSE = 0;

        if( n1->nb != n2->nb || n1->ns != n2->ns )
        {
            fprintf(stderr, "node %d differ in dimensionality!\n", n);
            exit(1);
        }

        for( i=0; i < n1->nb; i++ )
        {
            diff = n1->pBelief[i] - n2->pBelief[i];
            //diff = n1->pBelief[i];

            pBeliefMSE += diff * diff;
        }

        for( i=0; i < n1->nb; i++ )
        {
            for( j=0; j < n1->ns; j++ )
            {
                diff = n1->mu[n1->nb*i+j] - n2->mu[n1->nb*i+j];
                //diff = n1->mu[n1->nb*i+j];
                muMSE += diff*diff;

                diff = n1->sigma[n1->nb*i+j] - n2->sigma[n1->nb*i+j];
                //diff = n1->sigma[n1->nb*i+j];
                sigmaMSE += diff*diff;
            }

            diff = n1->starv[i] - n2->starv[i];
            //diff = n1->starv[i];
            starvMSE += diff*diff;
        }

/*
        printf("pBelief mse: %0.20f\n", pBeliefMSE / n1->nb);
        printf("mu mse: %0.20f\n", muMSE / (n1->ns*n1->nb));
        printf("sigma mse: %0.20f\n", sigmaMSE / (n1->ns*n1->nb));
        printf("starv mse: %0.20f\n", starvMSE / (n1->nb));
*/

        pBeliefMSE_Sum += pBeliefMSE / n1->nb;
        muMSE_Sum += muMSE / (n1->ns * n1->nb);
        sigmaMSE_Sum += sigmaMSE / (n1->ns * n1->nb);
        starvMSE_Sum += starvMSE / n1->nb;
    }

    printf("pBelief mse: %0.20f\n", pBeliefMSE_Sum / d1->nNodes);
    printf("mu mse: %0.20f\n", muMSE_Sum / d1->nNodes);
    printf("sigma mse: %0.20f\n", sigmaMSE_Sum / d1->nNodes);
    printf("starv mse: %0.20f\n", starvMSE_Sum / d1->nNodes);

}

void GenerateFrame( float * frame_host, float * frame_dev, uint frameSize )
{
    uint i;
    for( i=0; i < frameSize; i++ )
    {
        frame_host[i] = (float) rand() / RAND_MAX;
    }

    CUDAMEMCPY( frame_dev, frame_host, sizeof(float) * frameSize, cudaMemcpyHostToDevice );

    return;
}

int main()
{
    Destin *d_cuda, *d_cpu;
    uint i, nLayers, nIt;
    uint *dims = NULL;
    float *frame_host = NULL, *frame_dev = NULL;

    nLayers = 7;
    nIt = 1000;
    
    srand(0);
    MALLOC( dims, uint, nLayers );
    for( i=0; i < nLayers; i++ )
    {
        dims[i] = rand() % 20 + 20;
    }
    
    uint frameSize = 1;
    for( i=0; i < nLayers - 1; i++ )
    {
        frameSize *= 4;
    }

    frameSize *= 16;

    MALLOC( frame_host, float, frameSize );
    CUDAMALLOC( (void **) &frame_dev, sizeof(float) * frameSize );

    srand(0);
    d_cuda = InitDestin(16, nLayers, dims, 0);

    srand(0);
    d_cpu = InitDestin(16, nLayers, dims, 0);

    ClearBeliefs( d_cuda );
    ClearBeliefs( d_cpu );


    GenerateFrame( frame_host, frame_dev, frameSize );
    float cardStart = (float) clock() / CLOCKS_PER_SEC;
    // do nIt training iterations on card
    for( i=0; i < nIt; i++ )
    {
        FormulateBelief( d_cuda, true, frame_dev);
    }

    // do nIt feedforwards on card
    for( i=0; i < nIt; i++ )
    {
        FormulateBelief( d_cuda, false, frame_dev);
    }
    float cardStop = (float) clock() / CLOCKS_PER_SEC;
    
    float tCard = cardStop - cardStart;
    
    printf("Card: %0.3f\n", tCard);

    // do nIt training iterations on host
    float hostStart = (float) clock() / CLOCKS_PER_SEC;
    for( i=0; i < nIt; i++ )
    {
        __CPU_FormulateBelief( d_cpu, true, frame_host);
    }

    // do nIt feedforwards on host
    for( i=0; i < nIt; i++ )
    {
        __CPU_FormulateBelief( d_cpu, false, frame_host);
    }
    float hostStop = (float) clock() / (CLOCKS_PER_SEC);

    float tHost = hostStop - hostStart;
    printf("Host: %0.3f\n", tHost);

    printf("Speedup: %0.2fx\n", tHost / tCard);

    CopyDestinFromDevice( d_cuda );

    GetMSE( d_cpu, d_cuda );

/*
    for( i=0; i < nIt; i++ )
    {
        GenerateFrame( frame_host, frame_dev, frameSize );

        FormulateBelief( d_cuda, true, frame_dev);
        __CPU_FormulateBelief( d_cpu, true, frame_host);

        CopyDestinFromDevice( d_cuda );

        GetMSE( d_cpu, d_cuda );
    }
*/

    DestroyDestin( d_cpu );
    DestroyDestin( d_cuda );
    
    FREE( dims );
    FREE( frame_host );
    CUDAFREE( frame_dev );

    return 0;
}
