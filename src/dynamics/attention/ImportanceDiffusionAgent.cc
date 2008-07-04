/*
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * Written by Joel Pitt <joel@fruitionnz.com>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <gsl/gsl_matrix_float.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

#include "ImportanceDiffusionAgent.h"

#include <platform.h>

#include <CogServer.h>
#include <Link.h>

namespace opencog
{

ImportanceDiffusionAgent::ImportanceDiffusionAgent()
{

}

ImportanceDiffusionAgent::~ImportanceDiffusionAgent()
{

}

void ImportanceDiffusionAgent::run(CogServer* server)
{

    a = server->getAtomSpace();
    spreadImportance();

}


void ImportanceDiffusionAgent::spreadImportance()
{
    gsl_matrix* connections;
    //gsl_permutation* permutation;
    //int permutationSign;
    gsl_vector* stiVector;
    gsl_vector* result;
    int errorNo;

    // The source and destinations of STI
    std::map<Handle,int> diffusionAtomsMap;
    int totalDiffusionAtoms = 0;

    std::vector<Handle> links;
    std::vector<Handle>::iterator hi;
    std::back_insert_iterator< std::vector<Handle> > out_hi(links);

    //printf("Begin diffusive importance spread.\n");
    // Get all HebbianLinks
    a->getHandleSet(out_hi, HEBBIAN_LINK, true);

    //cout << "Diffusion atoms and their matrix indices" << endl;
    for (hi = links.begin(); hi != links.end(); hi++) {
        // Get all atoms in outgoing set of link
        std::vector<Handle> targets;
        std::vector<Handle>::iterator targetItr;
        Handle linkHandle = *hi;
        float val;
        val = a->getTV(*hi).toFloat();
        if (val == 0.0f) {
            continue;
        }

        targets = TLB::getAtom(linkHandle)->getOutgoingSet();

        for (targetItr = targets.begin(); targetItr != targets.end();
                targetItr++) {
            if (diffusionAtomsMap.find(*targetItr) == diffusionAtomsMap.end()) {
                diffusionAtomsMap[*targetItr] = totalDiffusionAtoms;
                //cout << TLB::getAtom((*targetItr))->toString() << " = " << totalDiffusionAtoms << endl;
                totalDiffusionAtoms++;
            }
        }
    }
    // diffusionNodes now contains all atoms involved in spread.
    
    //printf("%d total diffusion atoms.\n", totalDiffusionAtoms);
    //printf("Creating norm sti vector.\n");
    // go through diffusionNodes and add each to the sti vector.
    // position in stiVector matches set (set is ordered and won't change
    // unless you add to it.
    stiVector = gsl_vector_alloc(totalDiffusionAtoms);
    for (std::map<Handle,int>::iterator i=diffusionAtomsMap.begin();
            i != diffusionAtomsMap.end(); i++) {
        Handle dAtom = (*i).first;
        gsl_vector_set(stiVector,(*i).second,a->getNormalisedZeroToOneSTI(dAtom,false));
    }
    
    //printf("Initial normalised STI values\n");
    //printVector(stiVector);

//#ifdef DEBUG
//    cout << diffusionNodes;
//    cout << stiVector;
//#endif

    // set connectivity matrix size
    // size is dependent on the number of atoms that are connected
    // by a HebbianLink in some way.
    connections = gsl_matrix_alloc(totalDiffusionAtoms, totalDiffusionAtoms);

    for (hi=links.begin(); hi != links.end(); hi++) {
        // Get all atoms in outgoing set of link
        std::vector<Handle> targets;
        std::vector<Handle>::iterator targetItr;

        std::map<Handle,int>::iterator sourcePosItr;
        std::map<Handle,int>::iterator targetPosItr;
        int sourceIndex;
        int targetIndex;
        Type type;
        float val;

        val = a->getTV(*hi).toFloat();
        if (val == 0.0f) continue;
        type = TLB::getAtom(*hi)->getType(); 
        // Not necessary/correct, just swap indices below
        //if (type == INVERSE_HEBBIAN_LINK) {
        //    val = -val;
        //}
        //cout << TLB::getAtom((*hi))->toString() << endl;

        targets = TLB::getAtom(*hi)->getOutgoingSet();
        if (ClassServer::isAssignableFrom(ORDERED_LINK,type)) {

            // Add only the source index
            sourcePosItr = diffusionAtomsMap.find(targets[0]);
            //if ((*sourcePosItr).first == (* targets.end())) {
            //    cout << "Can't find source in list of diffusionNodes. "
            //            "Something odd has happened" << endl;
            //}
            sourceIndex = (*sourcePosItr).second;
            //cout << "Ordered link with source index " << sourceIndex << endl;

            // and then spread from index 1 (since source is at index 0)
            //(*sourcePosItr).first;
            for (targetItr = (targets.begin())++; targetItr != targets.end(); 
                     targetItr++) {
                targetPosItr = diffusionAtomsMap.find(*targetItr);
                targetIndex = (*targetPosItr).second;
                // source and target indicies swapped because inverse
                gsl_matrix_set(connections,sourceIndex,targetIndex,val);
            }
        } else {
            std::vector<Handle>::iterator sourceItr;
            // Add the indices of all targets as sources
            // then go through all pairwise combinations
            //cout << "Unordered link" << endl;
            for (sourceItr = targets.begin(); sourceItr != targets.end();
                    sourceItr++) {
                for (targetItr = targets.begin(); targetItr != targets.end();
                        targetItr++) {
                    sourcePosItr = diffusionAtomsMap.find(*sourceItr);
                    sourceIndex = (*sourcePosItr).second;
                    targetPosItr = diffusionAtomsMap.find(*targetItr);
                    targetIndex = (*targetPosItr).second;
                    gsl_matrix_set(connections,targetIndex,sourceIndex,val);
                }
            }
        }
    }
    // Set diagonal values to 1.0
    for (unsigned int j = 0; j < connections->size2; j++) {
        float sumProb = 0.0f;
        for (unsigned int i = 0; i < connections->size1; i++) {
            if (i != j) sumProb += gsl_matrix_get(connections,i,j);
        }
        if (sumProb > 0.5f) {
            for (unsigned int i = 0; i < connections->size1; i++) {
                gsl_matrix_set(connections,i,j,gsl_matrix_get(connections,i,j) / (sumProb/0.5));
            }
            sumProb = 0.5f;
        }
        //cout << sumProb << endl;
        gsl_matrix_set(connections,j,j,1.0-sumProb);
    }
    
    //printf("Hebbian connection matrix\n");
    //printMatrix(connections);
    // solve
    //permutation = gsl_permutation_alloc(totalDiffusionAtoms);
    result = gsl_vector_alloc(totalDiffusionAtoms);
    //errorNo = gsl_linalg_LU_decomp(connections,permutation,&permutationSign);
    //printf("Hebbian connection matrix after LU decomp\n");
    //printMatrix(connections);
    //if (errorNo) {
    //    printf ("error: %s\n", gsl_strerror (errorNo));
    //}
    errorNo = gsl_blas_dgemv(CblasNoTrans,1.0,connections,stiVector,0.0,result);
    //errorNo = gsl_linalg_LU_solve(connections,permutation,stiVector,result);
    if (errorNo) {
        printf ("error: %s\n", gsl_strerror (errorNo));
    }

    //printf("Result\n");
    //printVector(result);

    // set the sti of all atoms based on new values in new 
    // vector from multiplication 
    for (std::map<Handle,int>::iterator i=diffusionAtomsMap.begin();
            i != diffusionAtomsMap.end(); i++) {
        Handle dAtom = (*i).first;
        double val = gsl_vector_get(result,(*i).second);
        setScaledSTI(dAtom,val);
    }

    // free memory!
    gsl_matrix_free(connections);
    //gsl_permutation_free(permutation);
    gsl_vector_free(stiVector);
    gsl_vector_free(result);
}

void ImportanceDiffusionAgent::setScaledSTI(Handle h, float scaledSTI)
{
    AttentionValue::sti_t val;

    val = a->getMinSTI(false) + (scaledSTI * ( a->getMaxSTI(false) - a->getMinSTI(false) ));
    a->setSTI(h,val);
    
}

void ImportanceDiffusionAgent::printMatrix(gsl_matrix* m) {

    for (unsigned int i = 0; i < m->size1; i++) {
        for (unsigned int j = 0; j < m->size2; j++) {
            float val = gsl_matrix_get(m,i,j);
            if (val != 0.0) 
                printf("%4.2f ", gsl_matrix_get(m,i,j));
            else printf("---- ");
        }
        printf("\n");
    }
}

void ImportanceDiffusionAgent::printVector(gsl_vector* m) {

    printf("[");
    for (unsigned int i = 0; i < m->size; i++) {
        printf("%4.2f\n ", gsl_vector_get(m,i));;
    }
    printf("]\n");
}

};

