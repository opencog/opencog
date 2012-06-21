#ifndef __DESTIN_H
#define __DESTIN_H

#include "node.h"

/* Destin Struct Definition */
struct Destin {
    /* HOST VARIABLES BEGIN */
    int nNodes;                         // number of nodes
    int nBeliefs;                       // number of beliefs
    int nInputPipeline;                 // number of beliefs to copy to next nodes' input
    int maxNs;                          // max number of states for all nodes (important for kernels)
    int maxNb;                          // max number of beliefs for all nodes (important for kernels)
    int nMovements;                     // number of movements per digit presentation
    
    Node     * nodes_host;              // pointer to list of host nodes
    float    * inputPipeline;           // concatonated input for all internal layer nodes
    float    * belief;                  // concatonated belief vector for all nodes
    int      * layerSize;               // size for each layer
    int        nLayers;                 // number of layers in network
    
    float    * dataSet;                 // pointer to dataset

    /* HOST VARIABLES END */

    /* DEVICE VARIABLES BEGIN */
    CudaNode * nodes_dev;               // pointer to list of device nodes
    float    * inputPipeline_dev;       // pointer to input block
    float    * belief_dev;              // pointer to belief block
    float    * dataSet_dev;             // pointer to dataset
    /* DEVICE VARIABLES END */
};


/* Destin Struct Definition End */


/* Destin Functions Begin */
Destin * InitDestin(                    // initialize Destin.
                    int,                // input dimensionality for first layer
                    int,                // number of layers
                    int *,              // belief dimensionality for each layer
                    int                 // number of movements per digit presentation
                );

void RunDestin(                         // train destin.
                 Destin *,              // initialized destin pointer
                 char *,                // filename for data file
                 char *,                // filename for belief file (only if in feed-forward)
                 bool                   // true: train. false: feed-forward
                );

void PrintNetwork(                      // print out the network for debuggin'
                Destin *,               // network to print
                float *                 // pointer to input image frame
                );

void CopyDestinToDevice(                // copy Destin in host mem to device mem
                    Destin *            // pointer to use for host->device copy
                );


void CopyDestinFromDevice(              // copy Destin from device mem to host mem
                    Destin *            // pointer to use for device->host copy
                );


void DestroyDestin(
                    Destin *
                  );


void FormulateBelief(                   // form belief operation.  gets the current belief from Destin
                    Destin *,           // network to obtain belief from
                    bool,               // switch for training/feed-forward
                    float *             // input
                );

void ClearBeliefs(                      // cleanse the pallette
                  Destin *              // pointer to destin object
                 );

/* Destin Functions End */

#endif
