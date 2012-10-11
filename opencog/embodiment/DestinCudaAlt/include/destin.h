#ifndef __DESTIN_H
#define __DESTIN_H

#include <stdbool.h>
#include "macros.h"
#include "node.h"

/* Destin Struct Definition */
typedef struct Destin {
    uint nInputPipeline;                // number of beliefs to copy to next nodes' input
    uint maxNb;                         // max number of beliefs for all nodes (important for kernels)
    uint maxNs;
    uint nc;                            // number of classes to discriminate
    uint nBeliefs;                      // number of beliefs ( sum over all layers of centroids per node * number of nodes per layer )
    uint nNodes;                        // number of nodes in the entire destin network
    uint nMovements;                    // number of movements per digit presentation
    uint nLayers;                       // number of layers in network
    float muSumSqDiff;
    uint *nb;                           // number of beliefs in a node of a layer
    
    struct Node * nodes;                // pointer to list of host nodes

    float       * belief;               // concatonated belief vector for all nodes
    float       * temp;                 // temperatures for each layer
    float       * dataSet;              // pointer to dataset
    float       * inputPipeline;        // concatonated input for all internal layer nodes

    uint        * inputLabel;           // input label (used during supervised training)
    uint        * layerSize;            // size for each layer ( nodes per layer )
    uint        * layerMask;
} Destin;
/* Destin Struct Definition End */


/* Destin Functions Begin */
Destin * CreateDestin(                  // create destin from a config file
                    char *              // filename
        );

Destin * InitDestin(                    // initialize Destin.
                    uint,               // input dimensionality for first layer
                    uint,               // number of layers
                    uint *,             // belief dimensionality for each layer
                    uint,               // number of classes
                    float,              // beta coeff
                    float,              // lambda coeff
                    float,              // gamma coeff
                    float *,            // temperature for each layer
                    float,              // starv coeff
                    uint                // number of movements per digit presentation
                );

void LinkParentBeliefToChildren(        // link the belief from a parent to the child for advice
                    Destin *            // initialized destin pointer
                );

void TrainDestin(                       // train destin.
                 Destin *,              // initialized destin pointer
                 char *,                // filename for data file
                 char *                 // filename for label file
                );

void TestDestin(                        // test destin.
                 Destin *,              // initialized destin pointer
                 char *,                // filename for data file
                 char *,                // filename for label file
                 bool                   // generative/output ?
                );

void SaveDestin(                        // save destin to disk
                Destin *,               // network to save
                char *                  // filename to save to
        );

Destin * LoadDestin(                    // load destin from disk
                Destin *,               // network to save
                char *                  // filename to load from
        );

void ResetStarvTrace(                   // reset the starv trace to 1's
            Destin *
        );

void DestroyDestin(
                    Destin *
                  );

void FormulateBelief(                   // form belief operation.  gets the current belief from Destin
                    Destin *,           // network to obtain belief from
                    float *             // input
                );

void GenerateInputFromBelief(
                    Destin *,
                    float *
        );

void DisplayFeatures(
                    Destin *
        );

void ClearBeliefs(                      // cleanse the pallette
                  Destin *              // pointer to destin object
                 );

/* Destin Functions End */

#endif
