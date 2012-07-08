#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#include "macros.h"
#include "node.h"
#include "destin.h"

Destin * InitDestin( uint ni, uint nl, uint *nb, uint nMovements )
{
    uint nNodes, nInputPipeline;
    uint i, l, nBeliefs, maxNs, maxNb;
    size_t bOffset, iOffset;

    float alpha, beta, starvCoeff;

    Destin *d;

    // initialize a new Destin object
    MALLOC(d, Destin, 1);

    MALLOC(d->nBeliefsPerNode, int, nl);
    memcpy(d->nBeliefsPerNode, nb, nl*sizeof(int));

    d->nNodes = 0;
    d->nLayers = nl;

    d->nMovements = nMovements;

    // get number of nodes to allocate
    // starting from the top layer with one node,
    // each subsequent layer has 4x the nodes.
    //
    // eg., 2-layer: 05 nodes
    //      3-layer: 21 nodes
    //      4-layer: 85 nodes
    //
    // also keep track of the number of beliefs
    // to allocate

    MALLOC(d->layerSize, uint, nl);

    nNodes = 0;
    nBeliefs = 0;
    for( i=0, l=nl ; l != 0; l--, i++ )
    {
        d->layerSize[i] = 1 << 2*(l-1);
//        printf("layer[%d] has %d nodes\n", i, d->layerSize[i]);
        nNodes += d->layerSize[i];
        nBeliefs += d->layerSize[i] * nb[i];
    }

    d->nNodes = nNodes;

    // input pipeline -- all beliefs are copied from the output of each
    // node to the input of the next node on each timestep. we want
    // the belief of each node except for the top node (its output goes
    // to no input to another node) to be easily copied to the input
    // of the next node, so we allocate a static buffer for it.
    nInputPipeline = nBeliefs - nb[nl-1];

    d->nInputPipeline = nInputPipeline;

    // allocate node pointers on host and device
//    printf("allocating %d nodes\n", nNodes);
    MALLOC(d->nodes_host, Node, nNodes);
    CUDAMALLOC( (void **) &d->nodes_dev, sizeof(CudaNode)*nNodes);

    // allocate space for inputs to nodes
//    printf("allocating %d inputs\n", nInputPipeline);
    MALLOC(d->inputPipeline, float, sizeof(float)*nInputPipeline);
    CUDAMALLOC( (void **) &d->inputPipeline_dev, sizeof(float)*nInputPipeline);

    // allocate space for beliefs for nodes on host and device
//    printf("allocating %d beliefs\n", nBeliefs);
    MALLOC(d->belief, float, sizeof(float)*nBeliefs);
    CUDAMALLOC( (void **) &d->belief_dev, sizeof(float)*nBeliefs);

    d->nBeliefs = nBeliefs;

    alpha = 0.001;
    beta = 0.01;
    starvCoeff = 0.05;

    // init belief and input offsets (pointers to big belief and input chunks we
    // allocated above)
    bOffset = 0;
    iOffset = 0;

    // keep track of the max num of beliefs and states.  we need this information
    // to correctly call kernels later
    maxNb = 0;
    maxNs = 0;

    // allocate the input layer offsets.  each node gets an offset from
    // the frame it is presented with.  now computing it indirectly with
    // an array, but there's gotta be a closed-form way of getting the
    // input offset..
    //
    // **note** this is hard-coding a 4-to-1 reduction assuming visual
    // input (2d). we may want 2-to-1 reduction for audio input for
    // future research

    uint n, m;

    uint **inputOffsets;
    uint nInputNodes = pow(4,nl-1);

    MALLOC(inputOffsets, uint *, sizeof(uint **) * d->layerSize[0]);
    for( i=0; i < nInputNodes; i++ )
    {
        MALLOC(inputOffsets[i], uint, sizeof(uint) * nInputNodes);
    }

    // get integer sq root of layersize[0]
    uint layerSizeSqRoot = (uint) sqrt( d->layerSize[0] );

    // get integer sq root of ni for lowest layer.  asssumes input is a square.
    uint inputSizeSqRoot = (uint) sqrt( ni );


    // get column size of input image (assuming it is square)
    uint nc = (uint) sqrt( d->layerSize[0] * ni );

    // calculate offsets.
    uint a, b, innerIdx, bias;
    
    // iterate through rows... (nodes)
    for( i=0, m=0; m < layerSizeSqRoot; m+=2 )
    {
        // iterate through columns... (nodes)
        for( n=0; n < layerSizeSqRoot; n+=2, i+=4 )
        {
            // iterate through rows... (inputs)
            for( innerIdx = 0, a=0; a < inputSizeSqRoot; a++ )
            {
                // iterate through columns... (inputs)
                for( b=0; b < inputSizeSqRoot; b++, innerIdx++ )
                {
                    bias = m*nc*inputSizeSqRoot + n*inputSizeSqRoot;
                    inputOffsets[i+0][innerIdx] = bias + a*nc+b;
                    inputOffsets[i+1][innerIdx] = bias + a*nc+b+inputSizeSqRoot;
                    inputOffsets[i+2][innerIdx] = bias + (a+inputSizeSqRoot)*nc+b;
                    inputOffsets[i+3][innerIdx] = bias + (a+inputSizeSqRoot)*nc+b+inputSizeSqRoot;
                }
            }
        }
    }


    int layerSize;
    MALLOC( d->nodeRef, int *, sizeof(int **) * d->nLayers );

    // set up layer/row/col references for nodes
    for( i=0, l=0; l < d->nLayers - 1; l++ )
    {
        layerSize = d->layerSize[l];
        layerSizeSqRoot = (uint) sqrt( layerSize );

        MALLOC( d->nodeRef[l], int, sizeof(int *)*layerSize );

        for( m=0; m < layerSizeSqRoot; m+=2 )
        {
            for( n=0; n < layerSizeSqRoot; n+=2, i+=4 )
            {
                d->nodeRef[l][  m   * layerSizeSqRoot + n  ] = i;
                d->nodeRef[l][  m   * layerSizeSqRoot + n+1] = i+1;
                d->nodeRef[l][(m+1) * layerSizeSqRoot + n  ] = i+2;
                d->nodeRef[l][(m+1) * layerSizeSqRoot + n+1] = i+3;
            }
        }
    }

    // set up layer/row/col reference for top node
    MALLOC( d->nodeRef[d->nLayers - 1], int, sizeof(int *) * 1 );
    d->nodeRef[l][0] = d->nNodes - 1;

    //printf("\n");


    // create array of pointers, one pointer per layer. Each layer
    // gets a memory block to hold all the nodes statistics for all nodes
    // in the layer. pre allocate cuda memory for node statistics all at once
    // ahead of time because cudaMalloc is very slow on some cards
    MALLOC( d->stats_dev, float *, nl );

    // allocate stats memory block for the input layer
    uint statsSizePerNode;
    statsSizePerNode = NodeStatsSize(ni, nb[0], nb[1]);
    CUDAMALLOC((void **)&d->stats_dev[0], sizeof(float) * statsSizePerNode * d->layerSize[0]);

    // allocate input offsets memory block for input layer
    CUDAMALLOC((void**)&d->inputOffsets_dev, sizeof(uint) * ni * d->layerSize[0]);

    // initialize zero-layer nodes
    float * statsOffset;
    uint * io_offset;
    for( n=0, i=0, statsOffset = d->stats_dev[0], io_offset = d->inputOffsets_dev ;
            i < d->layerSize[0];
            i++, n++, statsOffset += statsSizePerNode, io_offset += ni )
    {
        InitNode( n, ni, nb[0], nb[1],
                    starvCoeff, alpha, beta, 
                    &d->nodes_host[n], &d->nodes_dev[n], inputOffsets[n],
                    io_offset, NULL, &d->belief_dev[bOffset], statsOffset );

        // increment belief offset
        bOffset += nb[0];

    }

    // update max belief 
    if( nb[0] > maxNb )
    {
        maxNb = nb[0];
    }

    // update max state
    if( nb[0] + ni > maxNs )
    {
        maxNs = nb[1] + nb[0] + ni;
    }

    // initialize the rest of the network
    for( l=1; l < nl; l++ )
    {
        // update max belief
        if( nb[l] > maxNb )
        {
            maxNb = nb[l];
        }

        uint np = l == nl - 1 ? 0 : nb[l + 1];
        // update max state
        if( nb[l] + 4 * nb[l-1] + np > maxNs )
        {
            maxNs = nb[l] + 4 * nb[l-1] + np;
        }

        statsSizePerNode = NodeStatsSize(nb[l-1]*4, nb[l], np);
        CUDAMALLOC((void **)&d->stats_dev[l], sizeof(float) * statsSizePerNode * d->layerSize[l]);
        for( i=0, statsOffset = d->stats_dev[l] ; i < d->layerSize[l]; i++, n++, statsOffset+=statsSizePerNode )
        {
            InitNode( n, nb[l-1]*4, nb[l], np,
                    starvCoeff, alpha, beta,
                    &d->nodes_host[n], &d->nodes_dev[n],
                    NULL, NULL, &d->inputPipeline_dev[iOffset],
                    &d->belief_dev[bOffset], statsOffset );

            // increment previous belief offset (input to next node)
            iOffset += 4*nb[l-1];

            // increment belief offset (so beliefs are mapped contiguously in memory)
            bOffset += nb[l];
        }
    }
    
    LinkParentBeliefToChildren( d );

    // set up maximum state and belief sizes for kernel calling
    d->maxNs = maxNs;
    d->maxNb = maxNb;

    //printf("maxNb: %d. maxNs: %d\n", maxNb, maxNs);
    
    for( i=0; i < nInputNodes; i++ )
    {
        free(inputOffsets[i]);
    }

    free(inputOffsets);

    return d;
}

void LinkParentBeliefToChildren( Destin *d )
{
    CudaNode cudaNode_host, cudaNodeParent_host;

    Node *node, *parent;
    uint i, n, l;

    uint parentBias = d->layerSize[0];
    for( n=0, l=0; l < d->nLayers - 1; l++ )
    {
        for( i=0; i < d->layerSize[l]; i++, n++)
        {
            // get structs from device
            node = &d->nodes_host[n];
            parent = &d->nodes_host[parentBias + i / 4];

            // update values
            node->parent_pBelief = parent->pBelief;

            // get structs from card
            CUDAMEMCPY( &cudaNode_host, node->node_dev, sizeof(CudaNode), cudaMemcpyDeviceToHost );
            CUDAMEMCPY( &cudaNodeParent_host, parent->node_dev, sizeof(CudaNode), cudaMemcpyDeviceToHost );

            // update values
            cudaNode_host.parent_pBelief = cudaNodeParent_host.pBelief;

            // copy structs back to card
            CUDAMEMCPY( node->node_dev, &cudaNode_host, sizeof(CudaNode), cudaMemcpyHostToDevice );
            CUDAMEMCPY( parent->node_dev, &cudaNodeParent_host, sizeof(CudaNode), cudaMemcpyHostToDevice );
        }

        parentBias += d->layerSize[l+1];
    }

}

void PrintNetwork( Destin *d, float *inImg )
{
    Node *nPtr;

    CopyDestinFromDevice( d );

    uint i, j, l, n;

/*
    float avgClustErr;

    printf("avg clustering err:\n");
    for( n=0, l=0; l < d->nLayers; l++ )
    {
        avgClustErr = 0;

        for( i=0; i < d->layerSize[l]; i++, n++ )
        {
            avgClustErr += d->nodes_host[i].clustErr;
        }

        printf("    layer[%d]: %0.10f\n", l, avgClustErr / d->layerSize[l]);
    }
    printf("\n");
*/

    for( n=0, l=0; l < d->nLayers; l++ )
    {
        printf("layer %d:\n", l);

        for( i=0; i < d->layerSize[l]; i++, n++ )
        {
            nPtr = &d->nodes_host[n];

            printf("  node %d:\n", n, nPtr->ni);
            printf("INPUTS\n");
            if( nPtr->inputOffsets != NULL )
            {
                for( j=0; j < nPtr->ni; j++ )
                {
                    printf("%02.0f ", 100*inImg[ nPtr->inputOffsets[j] ] );
                }
            }
            else
            {
                for( j=0 ; j < nPtr->ni; j++ )
                {
                    printf("%02.0f ", 100*nPtr->input[j]);
                }
            }

            printf(" | ");

            for( j=0; j < nPtr->nb; j++ )
            {
                printf("%02.0f ", 100*nPtr->pBelief[j]);
            }
            
            printf(" | ");

            for( j=0; j < nPtr->np; j++ )
            {
                printf("%02.0f ", 100*nPtr->parent_pBelief[j]);
            }
            printf("\n\n");
            
            /*
            printf("STATISTICS\n");
            nPtr = &d->nodes_host[n];

            printf("  mu:\n");
            for( b=0; b < nPtr->nb; b++ )
            {
                printf("  ");
                for( s=0; s < nPtr->ns; s++ )
                {
                    printf("%0.2f ", nPtr->mu[s+b*nPtr->ns]);
                }
                printf("\n");
            }

            printf("\n\n");
            printf("  sigma:\n");
            for( b=0; b < nPtr->nb; b++ )
            {
                printf("  ");
                for( s=0; s < nPtr->ns; s++ )
                {
                    printf("%0.2f ", nPtr->sigma[s+b*nPtr->ns]);
                }
                printf("\n");
            }
            
            printf("\n\n");
            printf("  starvation:\n");
            printf("  ");
            for( b=0; b < nPtr->nb; b++ )
            {
                printf("%0.2f ", nPtr->starv[b]);
            }
            printf("\n");
            */
        }
    }
}

float * RunDestin( Destin *d, char *dataFileName, bool isTrain )
{
    // belief output.  allocate if isTrain is false (only output beliefs if the network
    // is trained)
    float *beliefOut = NULL;

    // check if destin passed is initialized
    if( d == NULL )
    {
        fprintf(stderr, "Destin 0x%p not initialized!\n", d);
        exit(1);
    }

    FILE *dataFile;
    
    // filesize in bytes
    size_t nFloats;
    
    dataFile = fopen(dataFileName, "r");
    if( !dataFile ) {
        fprintf(stderr, "Cannot open data file %s\n", dataFileName);
        exit(1);
    }

    // get filesize
    fseek(dataFile, 0L, SEEK_END);
    nFloats = ftell(dataFile) / 4; //TODO: should this divide by sizeof(float) instead?
    fseek(dataFile, 0L, SEEK_SET);

    // get remaining memory on card
    size_t deviceFree, deviceTotal;
    cudaMemGetInfo(&deviceFree, &deviceTotal);

    size_t chunkSize;
    size_t inputFrameSize = d->layerSize[0] * 16; //TODO: replace magic number 16 with a constant variable

    uint nPresentations;

    if( !isTrain )
    {
        nPresentations = nFloats / inputFrameSize / d->nMovements;

        MALLOC( beliefOut, float, d->nBeliefs * nPresentations * 3 );
    }

    // if the dataset fits in 80% of remaining memory, allocate the whole she-bang
    if( nFloats < (size_t) ((float) (deviceFree/4) * 0.8) )
    {
        chunkSize = nFloats;
    }
    else
    {
        // otherwise, use ~80% of remaining memory for data chunks (rounding up to the next digit presentation size)
        chunkSize = (size_t) ((float) (deviceFree/4) * 0.8 );
        chunkSize += inputFrameSize*d->nMovements - (chunkSize % (inputFrameSize*d->nMovements));
    }

    // allocate space for data set
    MALLOC(d->dataSet, float, chunkSize);
    CUDAMALLOC( (void **) &d->dataSet_dev, sizeof(float)*chunkSize );

    size_t nFloatsRead = 0;

    uint i, iMod, wIt;

    wIt = 0;

    // while the whole file hasn't been read...
    while( nFloatsRead < nFloats )
    {
        size_t nFloats_it = 0;

        // read in a chunk
        i = 0;
        while( i < chunkSize )
        {
            if( feof( dataFile ) )
            {
                break;
            }
            nFloats_it += fread( &d->dataSet[i], sizeof(float), inputFrameSize, dataFile );

            i += inputFrameSize;
        }

        nFloatsRead += nFloats_it;

        // copy chunk of data set to device
        CUDAMEMCPY( d->dataSet_dev, d->dataSet, sizeof(float) * chunkSize, cudaMemcpyHostToDevice );

        // get number of iterations to run
        uint nIt = nFloats_it / inputFrameSize;

        printf("presenting %d movements...\n", nIt);

        for( i=0; i < nIt; i++ )
        {
            // a new digit is picked up every d->nMovements movements.
            if( i % d->nMovements == 0 )
            {
                ClearBeliefs( d );
            }

            // formulate belief/update on the presentation
            FormulateBelief( d, isTrain, d->dataSet_dev + i*inputFrameSize );

            // write out beliefs if we aren't training
            if( !isTrain )
            {
                // copy out the 10,12,14 movements
                iMod = i % d->nMovements;
                if( iMod == 10 || iMod == 12 || iMod == 14 )
                {
                    CUDAMEMCPY( &beliefOut[wIt*d->nBeliefs], d->belief_dev, sizeof(float) * d->nBeliefs, cudaMemcpyDeviceToHost );
                    wIt++;
                }
            }
        }
    }

    fclose( dataFile );

    FREE( d->dataSet );
    CUDAFREE( d->dataSet_dev );

    return beliefOut;
}

void CopyDestinToDevice( Destin *d )
{
    uint i;

    // copy individual nodes
    for( i=0; i < d->nNodes; i++ )
    {
        CopyNodeToDevice( &d->nodes_host[i] );
    }

    // copy belief and input information
    CUDAMEMCPY( d->belief_dev,        d->belief,        sizeof(float) * d->nBeliefs,       cudaMemcpyHostToDevice );
    CUDAMEMCPY( d->inputPipeline_dev, d->inputPipeline, sizeof(float) * d->nInputPipeline, cudaMemcpyHostToDevice );
}

void CopyDestinFromDevice( Destin *d )
{
    uint i;

    // copy individual nodes
    for( i=0; i < d->nNodes; i++)
    {
        CopyNodeFromDevice( &d->nodes_host[i] );
    }

    // copy belief and input information
    CUDAMEMCPY( d->belief,        d->belief_dev,        sizeof(float) * d->nBeliefs,       cudaMemcpyDeviceToHost );
    CUDAMEMCPY( d->inputPipeline, d->inputPipeline_dev, sizeof(float) * d->nInputPipeline, cudaMemcpyDeviceToHost );
}

void DestroyDestin( Destin *d )
{
    uint i;

    for( i=0; i < d->nNodes; i++ )
    {
        DestroyNode( &d->nodes_host[i] );
    }

    FREE(d->nodes_host);
    FREE(d->inputPipeline);
    FREE(d->belief);
    FREE(d->nBeliefsPerNode);
    FREE(d->layerSize);

    for( i=0; i < d->nLayers; i++ )
    {
        FREE( d->nodeRef[i] );
        CUDAFREE( d->stats_dev[i] );
    }

    FREE( d->nodeRef );
    FREE( d->stats_dev );

    CUDAFREE(d->nodes_dev);
    CUDAFREE(d->inputPipeline_dev);
    CUDAFREE(d->belief_dev);
    CUDAFREE(d->inputOffsets_dev);
    FREE(d);
}

// grab a node at a particular layer, row, and column
Node *GetNodeFromDestin( Destin *d, uint l, uint r, uint c )
{
    // check layer bounds
    if( l >= d->nLayers )
    {
        fprintf(stderr, "GetNodeFromDestin(): layer requested is out of range!\n");
        return NULL;
    }

    uint layerSizeSqRoot = (uint) sqrt( d->layerSize[l] );

    // check row bounds
    if( r >= layerSizeSqRoot )
    {
        fprintf(stderr, "GetNodeFromDestin(): row requested is out of range!\n");
        return NULL;
    }

    // check column bounds
    if( c >= layerSizeSqRoot )
    {
        fprintf(stderr, "GetNodeFromDestin(): column requested is out of range!\n");
        return NULL;
    }

    // grab the node index
    uint nIdx = d->nodeRef[l][r*layerSizeSqRoot+c];
    return &d->nodes_host[nIdx];
}

// present the network with an image pointed to by image_dev.  update the network if doUpdate is true,
// otherwise just change the belief.
void FormulateBelief(Destin *d, bool isTrain, float *image_dev)
{
    dim3 blockSize(d->nNodes, d->maxNb);

    CalculateDistances<<<blockSize,d->maxNs,sizeof(float)*d->maxNs*2>>>(d->nodes_dev, image_dev);
    if( isTrain )
    // normalize belief and update winner
    {
        NormalizeBeliefGetWinner<<<d->nNodes,d->maxNb,sizeof(float)*d->maxNb*4>>>(d->nodes_dev);
        UpdateWinner<<<d->nNodes,d->maxNs>>>(d->nodes_dev, image_dev);
    }
    else
    // just normalize the belief
    {
        NormalizeBelief<<<d->nNodes,d->maxNb,sizeof(float)*d->maxNb*2>>>(d->nodes_dev);
    }

    CUDAMEMCPY( d->inputPipeline_dev, d->belief_dev, sizeof(float)*d->nInputPipeline, cudaMemcpyDeviceToDevice );

    CUDACHECKERROR();

    return;
}

// zeros-out the beliefs (and input registers) for each node.  useful for "restarting" the network
// between presentations of distinct elements
void ClearBeliefs( Destin *d )
{
    uint i;

    for( i=0; i < d->nInputPipeline; i++ )
    {
        d->inputPipeline[i] = 0;
    }

    for( i=0; i < d->nBeliefs; i++ )
    {
        d->belief[i] = 0;
    }
    
    // copy to card
    CUDAMEMCPY( d->inputPipeline_dev, d->inputPipeline, sizeof(float) * d->nInputPipeline, cudaMemcpyHostToDevice);
    CUDAMEMCPY( d->belief_dev,        d->belief,        sizeof(float) * d->nBeliefs,       cudaMemcpyHostToDevice);
}
