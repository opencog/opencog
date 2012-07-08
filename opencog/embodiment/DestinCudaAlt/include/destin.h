#ifndef __DESTIN_H
#define __DESTIN_H

#include "node.h"

/* Destin Struct Definition */
struct Destin {
    /* HOST VARIABLES BEGIN */

    uint nNodes;                        // number of nodes in the entire destin network
    uint nBeliefs;                      // number of beliefs ( sum over all layers of centroids per node * number of nodes per layer )
    uint nInputPipeline;                // number of beliefs to copy to next nodes' input
    uint maxNs;                         // max number of states for all nodes (important for kernels)
    uint maxNb;                         // max number of beliefs for all nodes (important for kernels)
    uint nMovements;                    // number of movements per digit presentation
    
    int * nBeliefsPerNode;              // number of beliefs ( centroids or states ) per node for each layer
    Node     * nodes_host;              // pointer to list of host nodes
    int     ** nodeRef;                 // allows easy indexing of nodes by layer, row, and col
    float    * inputPipeline;           // concatonated input for all internal layer nodes
    float    * belief;                  // concatonated belief vector for all nodes
    uint     * layerSize;               // size for each layer ( nodes per layer )
    uint       nLayers;                 // number of layers in network
    
    float    * dataSet;                 // pointer to dataset

    /* HOST VARIABLES END */

    /* DEVICE VARIABLES BEGIN */
    CudaNode * nodes_dev;               // pointer to list of device nodes
    float    * inputPipeline_dev;       // pointer to input block
    float    * belief_dev;              // pointer to belief block
    float    * dataSet_dev;             // pointer to dataset
    float   ** stats_dev;               // pointer to stats block. Root pointer on host, elements of the pointer array are device pointers.
    uint    * inputOffsets_dev;        // cuda memory block to hold input offsets for the input layer
    /* DEVICE VARIABLES END */
};


/* Destin Struct Definition End */


/* Destin Functions Begin */
Destin * InitDestin(                    // initialize Destin.
                    uint,                // input dimensionality for first layer
                    uint,                // number of layers
                    uint *,              // belief dimensionality for each layer
                    uint                 // number of movements per digit presentation
                );

void LinkParentBeliefToChildren(        // link the belief from a parent to the child for advice
                    Destin *            // initialized destin pointer
                );

float * RunDestin(                      // train destin.
                 Destin *,              // initialized destin pointer
                 char *,                // filename for data file
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

Node *GetNodeFromDestin(
              Destin *,                 // pointer to destin struct
              uint,                     // layer to grab from
              uint,                     // row to grab
              uint                      // column to grab
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
