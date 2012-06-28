#ifndef __DESTIN_WRAPPER_H
#define __DESTIN_WRAPPER_H

#include <stdbool.h>
#include "node.h"
#include "mex.h"

// wrapper to call the init for a node struct.  return a new node struct.
Node * Wrapper_InitNode( int, mxArray **, int, const mxArray **, Node *);

// wrapper to call the cleanup for a node struct.
void Wrapper_DeallocateNode( Node * );

// wrapper to make a node persistent in matlab's heap
void Wrapper_MakeNodePersistent( Node * );

// wrapper to train/test a node node
void Wrapper_RunNode( int, mxArray **, int, const mxArray **, bool, Node *);

// wrapper to return belief/starvation/mu/sigma
void Wrapper_GetState( int, mxArray**, Node * );

// main entry point
void mexFunction( int, mxArray **, int, const mxArray **);

#endif
