#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<string.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "macros.h"
#include "node.h"
#include "destin.h"

#define PI 3.14159
#define NITS 20000
#define NMEANS 1

#define ASTK 15
#define NCOLS 8
void PrintIt( Destin *d )
{
    uint i, j, n;
    uint l, nIdx, k;
    Node *nPtr;

    /*
    printf("\033[2J\033[1;1H");

    for( nIdx=0, l=0; l < d->nLayers; nIdx += d->layerSize[l++] )
    {
        uint nb = d->nodes[nIdx].nb;

        for( n=nIdx; n < nIdx + d->layerSize[l]; n += NCOLS )
        {
            for( i=0; i < nb; i++ )
            {
                for( k=0; k < NCOLS && n-nIdx+k < d->layerSize[l]; k++ )
                {
                    nPtr = &d->nodes[n+k];
                    uint nAstk = (uint) (nPtr->pBelief[i] * ASTK);

                    for( j=0; j < nAstk; j++ )
                    {
                        printf("*");
                    }
                    for( j=nAstk; j < ASTK; j++ )
                    {
                        printf("-");
                    }

                    if( i == nPtr->winner && i == nPtr->genWinner )
                    {
                        printf(" W\t");
                    } else if (i == nPtr->winner) {
                        printf(" w\t");
                    } else if (i == nPtr->genWinner ) {
                        printf(" g\t");
                    } else {
                        printf("  \t");
                    }
                }
                printf("\n");
            }
            printf("\n");
        }

        printf("\n");
    }
    */

    for( n=0; n < d->nNodes; n++ )
    {
        printf("node %d\n", n);
        nPtr = &d->nodes[n];
        for( i=0; i < nPtr->nb; i++ )
        {
            printf("%0.3f ", nPtr->pBelief[i]);
        }
        printf("\n");
    }
}

void TrainDestin( Destin *d, char *dataFileName, char *labelsFileName )
{
    // check if destin passed is initialized
    if( d == NULL )
    {
        fprintf(stderr, "Destin 0x%p not initialized!\n", d);
        exit(1);
    }
    
    FILE *dataFile;
    FILE *labelFile;
    
    dataFile = fopen(dataFileName, "r");
    if( !dataFile ) {
        fprintf(stderr, "Cannot open data file %s\n", dataFileName);
        exit(1);
    }

    labelFile = fopen(labelsFileName, "r");
    if( !labelFile ) {
        fprintf(stderr, "Cannot open data file %s\n", labelsFileName);
        exit(1);
    }

    size_t inputFrameSize = d->layerSize[0] * 16;

    // allocate space for data set
    MALLOC(d->dataSet, float, inputFrameSize);

    uint i;
    uint j;
    uint lTrain;
    uint label;

    uint nBatches = 6;
    uint batch;

    float muSumSqDiff = 0;

    SaveDestin( d, "destintmp.des");

    lTrain = -1;
    for( batch=0; batch < nBatches; batch++ )
    {
        i = 0;

        // while the whole file hasn't been read...
        while( i < NITS * 3 )//!feof(dataFile) )
        {
            fread( d->dataSet, sizeof(float), inputFrameSize, dataFile );
            fread( &label, sizeof(uint), 1, labelFile );

            if( label > d->nc )
            {
                fprintf(stderr, "bad input label -- %d > %d!\n", label, d->nc);
            }

            for( j=0; j < d->nc; j++ )
            {
                d->inputLabel[j] = 0;
            }

            d->inputLabel[label] = 1;

            // a new digit is picked up every d->nMovements movements.
            if( i % d->nMovements == 0 )
            {
                ClearBeliefs( d );
            }

            if( i % NITS == 0 && batch == 0 )
                lTrain++;

            for( j=0; j < d->nLayers; j++ )
            {
                if( j <= lTrain )
                    d->layerMask[j] = 1 && ((i % d->nLayers) >= j);
            }

            FormulateBelief( d, d->dataSet );
            muSumSqDiff += d->muSumSqDiff;

            if( i % 1000 == 0 )
            {
                printf("iteration %d: %0.5f\n", i, muSumSqDiff);
                SaveDestin( d, "destintmp.des");
                DisplayFeatures( d );
                cvWaitKey(25);

                uint nIdx;
                for( nIdx=0; nIdx < d->nodes[d->nNodes-1].nb; nIdx++ )
                {
                    printf("%0.2f ", d->nodes[d->nNodes-1].pBelief[nIdx]);
                }
                printf("\n");

                muSumSqDiff = 0;
            }

            i++;
        }

        SaveDestin( d, "destintmp.des");

        for( j=0; j < d->nc; j++ )
        {
            d->inputLabel[j] = 0;
        }


        rewind(dataFile);
        rewind(labelFile);
    }

    return;
}

void TestDestin( Destin *d, char *dataFileName, char *labelsFileName, bool generative )
{
    FILE *beliefsFile;
    beliefsFile = fopen("beliefs.dat", "w");
    
    FILE *dataFile;
    FILE *labelFile;
    
    dataFile = fopen(dataFileName, "r");
    if( !dataFile ) {
        fprintf(stderr, "Cannot open data file %s\n", dataFileName);
        exit(1);
    }

    labelFile = fopen(labelsFileName, "r");
    if( !labelFile ) {
        fprintf(stderr, "Cannot open data file %s\n", labelsFileName);
        exit(1);
    }

    size_t inputFrameSize = d->layerSize[0] * 16;

    // allocate space for data set
    MALLOC(d->dataSet, float, inputFrameSize);

    uint j;
    uint lTrain;
    uint label;

    uint nBatches = 5;
    uint batch;

    float muSumSqDiff = 0;
    
    FILE *matlabFile;
    matlabFile = fopen("destin_final.m", "w");
    uint n;

    Node * nTmp;
    nTmp = &d->nodes[d->nNodes-1];

    uint i;
    fprintf(matlabFile, "nd = %d;\n", nTmp->ns);
    fprintf(matlabFile, "nb = %d;\n", nTmp->nb);
    fprintf(matlabFile, "T = %f;\n", nTmp->temp);
    fprintf(matlabFile, "mu = [", n);
    for( i=0; i < nTmp->nb; i++ )
    {
        for( j=0; j < nTmp->ns; j++ )
        {
            fprintf(matlabFile, "%0.5f ", nTmp->mu[i*nTmp->ns+j]);
        }
        fprintf(matlabFile, ";\n");
    }
    fprintf(matlabFile, "];\n");
    
    fprintf(matlabFile, "sigma = [", n);
    for( i=0; i < nTmp->nb; i++ )
    {
        for( j=0; j < nTmp->ns; j++ )
        {
            fprintf(matlabFile, "%0.5f ", nTmp->sigma[i*nTmp->ns+j]);
        }
        fprintf(matlabFile, ";\n");
    }

    fclose(matlabFile);

    // set up stuff for generative display
    float *outFrame;
    MALLOC( outFrame, float, 16*16 );

    CvSize size;
    size.height = 16;
    size.width = 16;
    IplImage* inFrame_ipl  = cvCreateImageHeader(size, IPL_DEPTH_32F, 1);
    IplImage* outFrame_ipl = cvCreateImageHeader(size, IPL_DEPTH_32F, 1);

    outFrame_ipl->imageData = (char *) outFrame;
    outFrame_ipl->imageDataOrigin = outFrame_ipl->imageData;

    size.height = 128;
    size.width = 128;

    IplImage *destinIn  = cvCreateImage( size, IPL_DEPTH_32F, 1 );
    IplImage *destinOut = cvCreateImage( size, IPL_DEPTH_32F, 1 );

    i=0;

    ResetStarvTrace( d );

    while( i < d->nMovements * 15000 )
    {
        fread( d->dataSet, sizeof(float), inputFrameSize, dataFile );
        fread( &label, sizeof(uint), 1, labelFile );

        // a new digit is picked up every d->nMovements movements.
        if( i % d->nMovements == 0 )
        {
            ClearBeliefs( d );
        }
        
        FormulateBelief( d, d->dataSet );
    
        printf("iteration %d\n", i);

        if( generative )
        {
            uint m;
            if( i % d->nMovements == d->nMovements - 1 )
            {
                uint n;

                for( m=0; m < 10; m++ )
                {
                    GenerateInputFromBelief( d, outFrame );

                    inFrame_ipl->imageData = (char *) d->dataSet;
                    inFrame_ipl->imageDataOrigin = inFrame_ipl->imageData;

                    cvResize(inFrame_ipl, destinIn, CV_INTER_NN);
                    cvResize(outFrame_ipl, destinOut, CV_INTER_NN);

                    cvShowImage("Destin In", destinIn);
                    cvShowImage("Destin Out", destinOut);

                    cvWaitKey(10);
                }
            }
        } else {
            if( i % d->nMovements == d->nMovements - 1 )
            {
                uint j;
                fprintf(beliefsFile, "%d, ", label);
                for( j=0; j < d->nodes[d->nNodes-1].nb; j++ )
                {
                    fprintf(beliefsFile, "%0.5f, ", d->nodes[d->nNodes-1].pBelief[j]);
                }
                fprintf(beliefsFile, "\n");
            }
        }

        i++;
    }

    fclose( beliefsFile );
    fclose( dataFile );

    cvReleaseImage(&destinIn);
    cvReleaseImage(&destinOut);
    cvReleaseImageHeader(&inFrame_ipl);
    cvReleaseImageHeader(&outFrame_ipl);

    FREE( d->dataSet );
    FREE( outFrame );
}

// run an iteration with the CPU implementations of the kernels
void FormulateBelief( Destin *d, float *image )
{
    uint n, l, i;

    d->muSumSqDiff = 0;

    for( n=0, l=0; l < d->nLayers; l++ )
    {
        for( i=0; i < d->layerSize[l]; i++, n++ )
        {
            if( d->layerMask[l] == 1 )
            {
                GetObservation( d->nodes, image, n );
                CalculateDistances( d->nodes, n );

                NormalizeBeliefGetWinner( d->nodes, n );

                UpdateWinner( d->nodes, d->inputLabel, n );
                d->muSumSqDiff += d->nodes[n].muSqDiff;
            }
            else
            {
                GetObservation( d->nodes, image, n );
                CalculateDistances( d->nodes, n );

                NormalizeBeliefGetWinner( d->nodes, n );
            }
        }
    }

    // copy node's belief to parent node's input
    memcpy( d->inputPipeline, d->belief, sizeof(float)*d->nInputPipeline );
}

uint GenFromPMF( float *pmf, uint len )
{
    uint i;

    float n = 0;                    // normalizing const
    float c = 0;                    // cumulative sum
    float r = (float) rand() / RAND_MAX;    // random num chosen

    // get norm const
    for( i=0; i < len; i++ )
    {
        n += pmf[i];
    }

    // sample from pmf
    for( i=0; i < len; i++ )
    {
        c += pmf[i] / n;
        if( r < c )
            break;
    }

    return i;
}

// box-muller normal rng
double NormRND(double m, double s)
{
    double u1,u2;

    u1 = (double) rand() / RAND_MAX;
    u2 = (double) rand() / RAND_MAX;

    return sqrt(-2*log(u1)) * cos(2*PI*u2) * s + m;
}

void Boltzmannize( Node *n, double *b )
{
    uint i;

    double maxBoltz = 0;
    for( i=0; i < n->nb; i++ )
    {
        if( b[i] * n->temp > maxBoltz )
        {
            maxBoltz = b[i] * n->temp;
        }

    }

    double boltzSum = 0;
    for( i=0; i < n->nb; i++ )
    {
        b[i] = exp(n->temp * b[i] - maxBoltz);
        boltzSum += b[i];
    }

    for( i=0; i < n->nb; i++ )
    {
        b[i] /= boltzSum;
    }
}

void CalculateBelief( Node *n, double *x, double *b )
{
    uint i, j;
    double bSum, bDiff, bNorm;

    bNorm = 0;
    for( i=0; i < n->nb; i++ )
    {
        bSum = 0;
        for( j=0; j < n->ns - n->nc; j++ )
        {
            bDiff = x[j] - n->mu[i*n->ns + j];
            bDiff *= bDiff;
            bDiff *= n->starv[i];

            bSum += bDiff / n->sigma[i*n->ns+j];
        }
        b[i] = 1/sqrt(bSum);
        bNorm += b[i];
    }

    for( i=0; i < n->nb; i++ )
    {
        b[i] /= bNorm;
    }

    Boltzmannize( n, b );
}

double BeliefMSE( uint n, double *b, double *bx )
{
    uint i;
    double diff;
    double mse = 0;

    for( i=0; i < n; i++ )
    {
        diff = b[i] - bx[i];
        mse += diff * diff;
    }

    return mse;
}

// generate a belief sample by metropolis
void SampleInputFromBelief( Node *n, float *xf )
{
    uint i, j;
    double x[n->ns];
    double xGrad[n->ns];
    double xEps[n->ns];

    double pBelief[n->nb];
    double bx[n->nb];
    double bxEps[n->nb];

    double bMSE, bEpsMSE;

    double gradEps = 1e-10;

    uint nIts, nit;

    nIts = 50000;

    for( i=0; i < n->nb; i++ )
    {
        pBelief[i] = (double) n->pBelief[i];
    }

    // init x to fuzzy distance of winning centroid
    for( i=0; i < n->ns; i++ )
    {
        x[i] = NormRND(n->mu[n->genWinner*n->ns+i], n->sigma[n->genWinner*n->ns+i]);

        if( x[i] > 1 ) x[i] = 1;
        if( x[i] < 0 ) x[i] = 0;

        printf("%f ", n->observation[i]);
    }
    printf("\n");
    
    double g = 0.1;

    while( true )
    //for( nit=0; nit < nIts; nit++ )
    {
        CalculateBelief( n, x, bx );
        bMSE = BeliefMSE( n->nb, bx, pBelief );

        for( i=0; i < n->ns; i++ )
        {
            for( j=0; j < n->ns; j++ )
            {
                xEps[j] = x[j];
            }

            xEps[i] += gradEps;

            CalculateBelief( n, xEps, bxEps );
            bEpsMSE = BeliefMSE( n->nb, bxEps, pBelief );

            xGrad[i] = (bEpsMSE - bMSE) / gradEps;
        }

        for(i=0; i < n->ns - n->nc; i++ )
        {
            x[i] -= g * xGrad[i];

            if( x[i] > 1 ) x[i] = 1;
            if( x[i] < 0 ) x[i] = 0;
        }
        CalculateBelief( n, x, bx );

        /*
        printf("pBelief: ");
        for( i=0; i < n->nb; i++ )
        {
            printf("%0.3f ", pBelief[i]);
        }
        printf("\n");
        
        printf("bx:      ");
        for( i=0; i < n->nb; i++ )
        {
            printf("%0.3f ", bx[i]);
        }
        printf("\n");
        */
        printf("%f\n", bMSE);
    }
    

    for( i=0; i < n->ns; i++ )
    {
        xf[i] = (float) x[i];
        n->genObservation[i] = xf[i];
    }
}

void GenerateInputFromBelief( Destin *d, float *frame )
{
    uint n, j, nMeans, bIdx;
    int i;
    float sampledInput[d->maxNs], sample;
    Node *nTmp;

    // initialize the frame
    for( i=0; i < d->layerSize[0] * d->nodes[0].ni; i++ )
    {
        frame[i] = 0;
    }

    for( nMeans=0; nMeans < NMEANS; nMeans++ )
    {
        // proceed down the network to the input layer
        for( n = d->nNodes-1; n >= d->layerSize[0]; n-- )
        {
            printf("node %d\n", n);
            nTmp = &d->nodes[n];

            SampleInputFromBelief( nTmp, sampledInput );

            // pass sampled input to children's previous belief
            for( i=0; i < 4; i++ )
            {
                uint muCol = i*nTmp->children[i]->nb;
                float maxBelief = 0;
                uint genWinner = 0;

                /*
                float norm = 0;
                for( j=0; j < nTmp->children[i]->nb; j++ )
                    norm += sampledInput[muCol+j];
                */

                for( j=0; j < nTmp->children[i]->nb; j++ )
                {
                    nTmp->children[i]->pBelief[j] = sampledInput[muCol+j];// / norm;
                    if( nTmp->children[i]->pBelief[j] > maxBelief )
                    {
                        genWinner = j;
                        maxBelief = nTmp->children[i]->pBelief[j];
                    }
                }
                nTmp->children[i]->genWinner = genWinner;
            }
        }

        // output winning centroids to frame
        for( n=0; n < d->layerSize[0]; n++ )
        {
            nTmp = &d->nodes[n];
            
            //SampleInputFromBelief( nTmp, sampledInput );

            for( i=0; i < nTmp->ni; i++ )
            {
                sampledInput[i] = nTmp->mu[nTmp->genWinner*nTmp->ns+i];
                frame[nTmp->inputOffsets[i]] += log(sampledInput[i]);
            }
        }
    }

    // normalize output to 0-1.
    float frameMax = 0;
    float frameMin = 1;

    for( i=0; i < d->layerSize[0] * d->nodes[0].ni; i++ )
    {
        frame[i] = exp(frame[i] / NMEANS);

        if( frame[i] <= frameMin )
        {
            frameMin = frame[i];
        }

        if( frame[i] > frameMax )
        {
            frameMax = frame[i];
        }
    }

    frameMax -= frameMin;

    for( i=0; i < d->layerSize[0] * d->nodes[0].ni; i++ )
    {
        frame[i] -= frameMin;
        frame[i] /= frameMax;
    }

    /*
    // feed frame up
    for( i=0; i < d->nLayers; i++ )
    {
    FormulateBelief( d, frame );
    }
     */
}

void DisplayFeatures( Destin *d )
{
    uint i, j, n, u, b, sqrtPatch;

    uint width, height;

    sqrtPatch = (uint) sqrt(d->nodes[0].ni);

    height = d->layerSize[0] * sqrtPatch;
    width = d->nodes[0].nb * sqrtPatch;

    float *frame;

    MALLOC( frame, float, width*height );

    for( n=0; n < d->layerSize[0]; n++ )
    {
        for( b=0; b < d->nodes[n].nb; b++ )
        {
            for( u=0, i=0; i < sqrtPatch; i++ )
            {
                for( j=0; j < sqrtPatch; j++, u++ )
                {
                    frame[(n*sqrtPatch+i) * sqrtPatch*d->nodes[n].nb + j + b*sqrtPatch] = d->nodes[n].mu[b*d->nodes[n].ns+u];
                }
            }
        }
    }

    CvSize size;
    size.height = height;
    size.width = width;
    IplImage *featuresOut = cvCreateImageHeader(size, IPL_DEPTH_32F, 1);
    featuresOut->imageData = (char *) frame;
    featuresOut->imageDataOrigin = featuresOut->imageData;

    size.height = height * 2;
    size.width = width * 2;
    IplImage *bigImg = cvCreateImage(size, IPL_DEPTH_32F, 1);
    cvResize(featuresOut, bigImg, CV_INTER_CUBIC);

    cvShowImage("Features!!", bigImg);

    cvReleaseImageHeader(&featuresOut);
    cvReleaseImage(&bigImg);

    FREE( frame );
}

void ResetStarvTrace( Destin *d )
{
    uint i, j;
    Node *nTmp;

    for( i=0; i < d->nNodes; i++ )
    {
        nTmp = &d->nodes[i];

        for( j=0; j < nTmp->nb; j++ )
        {
            nTmp->starv[j] = 1;
        }
    }
}
