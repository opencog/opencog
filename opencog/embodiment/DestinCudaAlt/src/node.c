#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "destin.h"
#include "node.h"
#include "macros.h"

#define EPSILON     1e-8

// CPU implementation of GetObservation kernel
void GetObservation( Node *n, float *framePtr, uint nIdx )
{
    n = &n[nIdx];

    uint i, j;
    uint ni, nb, np, ns, nc;

    ni = n->ni;
    nb = n->nb;
    np = n->np;
    ns = n->ns;
    nc = n->nc;

    if( n->inputOffsets == NULL )
    {
        // normal input
        for( i=0; i < ni; i++ )
        {
            n->observation[i] = n->input[i];
        }
    } else {
        for( i=0; i < ni; i++ )
        {
            n->observation[i] = framePtr[n->inputOffsets[i]];
        }
    }

    for( i=0; i < nb; i++ )
    {
        n->observation[i+ni] = n->pBelief[i] * n->gamma;
    }

    for( i=0; i < np; i++ )
    {
        n->observation[i+ni+nb] = n->parent_pBelief[i] * n->lambda;
    }

    for( i=0; i < nc; i++ )
    {
        n->observation[i+ni+nb+np] = 0;
    }
}

// CPU implementation of CalculateDistances kernel
void CalculateDistances( Node *n, uint nIdx )
{
    float delta;
    float sumEuc, sumMal;

    n = &n[nIdx];

    uint i, j;
    uint ni = n->ni;
    uint nb = n->nb;
    uint ns = n->ns;
    uint nc = n->nc;

   // iterate over each belief
    for( i=0; i < n->nb; i++ )
    {
        sumEuc = 0;
        sumMal = 0;
        uint bRow = i*ns;

        // iterate over each state for belief
        for( j=0; j < ns-nc; j++ )
        {
            delta = n->observation[j] - n->mu[bRow+j];

            delta *= delta;
            delta *= n->starv[i];

            sumEuc += delta;
            sumMal += delta / n->sigma[bRow+j];
        }

        n->genObservation[i] = sumMal;

        sumEuc = sqrt(sumEuc);
        sumMal = sqrt(sumMal);

        n->beliefEuc[i] = ( sumEuc < EPSILON ) ? 1 : (1 / sumEuc);
        n->beliefMal[i] = ( sumMal < EPSILON ) ? 1 : (1 / sumMal);
    }
}

// CPU implementation of NormalizeBelief kernel
void NormalizeBeliefGetWinner( Node *n, uint nIdx )
{
    n = &n[nIdx];
    
    float normEuc = 0;
    float normMal = 0;

    float maxEucVal = n->beliefEuc[0];
    uint maxEucIdx = 0;
    
    float maxMalVal = n->beliefMal[0];
    uint maxMalIdx = 0;

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
        if( n->beliefMal[i] > maxMalVal )
        {
            maxMalVal = n->beliefMal[i];
            maxMalIdx = i;
        }
    }
    
    float maxBoltzEuc = 0;
    float maxBoltzMal = 0;

    // normalize beliefs to sum to 1
    for( i=0; i < n->nb; i++ )
    {
        n->beliefEuc[i] = ( normEuc < EPSILON ) ? (1 / (float) n->nb) : (n->beliefEuc[i] / normEuc);
        n->beliefMal[i] = ( normMal < EPSILON ) ? (1 / (float) n->nb) : (n->beliefMal[i] / normMal);

        // get maximum temp to normalize boltz normalization
        if( n->beliefEuc[i] * n->temp > maxBoltzEuc )
            maxBoltzEuc = n->beliefEuc[i] * n->temp;
        if( n->beliefMal[i] * n->temp > maxBoltzMal )
            maxBoltzMal = n->beliefMal[i] * n->temp;

        //n->pBelief[i] = n->beliefMal[i];
    }

    // boltzmann
    float boltzEuc = 0;
    float boltzMal = 0;

    for( i=0; i < n->nb; i++ )
    {
        n->beliefEuc[i] = exp(n->temp * n->beliefEuc[i] - maxBoltzEuc);
        n->beliefMal[i] = exp(n->temp * n->beliefMal[i] - maxBoltzMal);

        boltzEuc += n->beliefEuc[i];
        boltzMal += n->beliefMal[i];
    }

    for( i=0; i < n->nb; i++ )
    {
        n->beliefEuc[i] /= boltzEuc;
        n->beliefMal[i] /= boltzMal;

        n->pBelief[i] = n->beliefMal[i];
    }

    n->winner = maxEucIdx;
    n->genWinner = maxMalIdx;
}

// CPU implementation for UpdateWinner kernel
void UpdateWinner( Node *n, uint *label, uint nIdx )
{
    n = &n[nIdx];

    uint i;
    uint winnerOffset = n->winner*n->ns;
    float delta;

    n->nCounts[n->winner]++;

    n->muSqDiff = 0;

    uint ncStart = n->ni + n->nb + n->np;

    for( i=0; i < n->ns; i++ )
    {
        if( i < ncStart )
        {
            delta = n->observation[i] - n->mu[winnerOffset+i];
        } else {
            delta = (float) label[i - ncStart] - n->mu[winnerOffset+i];
        }

        float dTmp = (1 / (float) n->nCounts[n->winner]) * delta;

        n->mu[winnerOffset+i] += dTmp;
        n->muSqDiff += dTmp * dTmp;

        n->sigma[winnerOffset+i] += n->beta * (delta*delta - n->sigma[winnerOffset+i]);
    }
    
    for( i=0; i < n->nb; i++ )
    {
        n->starv[i] = n->starv[i] * (1 - n->starvCoeff) + n->starvCoeff * (i == n->winner);
    }
}
