/*
 * opencog/dynamics/attention/ImportanceDiffusionAgent.cc
 *
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
#ifdef HAVE_GSL

#include "ImportanceDiffusionAgent.h"

#include <time.h>
#include <math.h>
//#include <gsl/gsl_matrix_float.h>
//#include <gsl/gsl_linalg.h>
//#include <gsl/gsl_blas.h>

#include <opencog/atomspace/Link.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/platform.h>
#include <opencog/util/mt19937ar.h>

//#define DEBUG
namespace opencog
{

ImportanceDiffusionAgent::ImportanceDiffusionAgent(int decisionFunction)
{
    static const std::string defaultConfig[] = {
        //! Default value that normalised STI has to be above before
        //! being spread
        "ECAN_DIFFUSION_THRESHOLD","0.0",
        //! Maximum percentage of STI that is spread from an atom
        "ECAN_MAX_SPREAD_PERCENTAGE","0.5",
        "",""
    };
    setParameters(defaultConfig);

    //! @todo won't respond to the parameters being changed later
    //! (not a problem at present, but could get awkward with, for example,
    //! automatic parameter adaptation)
    maxSpreadPercentage = (float) (config().get_double("ECAN_MAX_SPREAD_PERCENTAGE"));

    switch (decisionFunction) {
    case HYPERBOLIC:
        spreadDecider = (SpreadDecider*) new HyperbolicDecider(0.5);
        break;
    case STEP:
        spreadDecider = (SpreadDecider*) new StepDecider();
        break;
    }
    setDiffusionThreshold((float) (config().get_double("ECAN_DIFFUSION_THRESHOLD")));

}

ImportanceDiffusionAgent::~ImportanceDiffusionAgent()
{

}

void ImportanceDiffusionAgent::setMaxSpreadPercentage(float p)
{ maxSpreadPercentage = p; }

float ImportanceDiffusionAgent::getMaxSpreadPercentage() const
{ return maxSpreadPercentage; }

void ImportanceDiffusionAgent::setDiffusionThreshold(float p)
{
    diffusionThreshold = p;
}

float ImportanceDiffusionAgent::getDiffusionThreshold() const
{ return diffusionThreshold; }

void ImportanceDiffusionAgent::run(CogServer* server)
{
    a = server->getAtomSpace();
    spreadDecider->focusBoundary = diffusionThreshold * (a->getMaxSTI()
            - a->getAttentionalFocusBoundary());
#ifdef DEBUG
    totalSTI = 0;
#endif
    spreadImportance();
}

int ImportanceDiffusionAgent::makeDiffusionAtomsMap(std::map<Handle,int> &diffusionAtomsMap,
        std::vector<Handle> links)
{
    int totalDiffusionAtoms = 0;
    std::vector<Handle>::iterator hi;

#ifdef DEBUG
    logger().fine("Finding atoms involved with STI diffusion and their matrix indices");
#endif
    for (hi = links.begin(); hi != links.end(); hi++) {
        // Get all atoms in outgoing set of links
        std::vector<Handle> targets;
        std::vector<Handle>::iterator targetItr;
        Handle linkHandle = *hi;
        float val;
        val = a->getTV(*hi).toFloat();
        if (val == 0.0f) {
            continue;
        }

        targets = dynamic_cast<Link*>(TLB::getAtom(linkHandle))->getOutgoingSet();

        for (targetItr = targets.begin(); targetItr != targets.end();
                targetItr++) {
            if (diffusionAtomsMap.find(*targetItr) == diffusionAtomsMap.end()) {
                diffusionAtomsMap[*targetItr] = totalDiffusionAtoms;
#ifdef DEBUG
                logger().fine("%s = %d",
                        TLB::getAtom((*targetItr))->toString().c_str(),
                        totalDiffusionAtoms);
#endif
                totalDiffusionAtoms++;
            }
        }
    }
    // diffusionAtomsMap now contains all atoms involved in spread as keys and
    // their matrix indices as values.
    return totalDiffusionAtoms;
    
}

void ImportanceDiffusionAgent::makeSTIVector(bvector* &stiVector,
        int totalDiffusionAtoms, std::map<Handle,int> diffusionAtomsMap) {
    // go through diffusionAtomsMap and add each to the STI vector.
    // position in stiVector matches set (set is ordered and won't change
    // unless you add to it.
    
    // alloc
    stiVector = new bvector(totalDiffusionAtoms);    
    // zero vector
    //gsl_vector_set_zero(stiVector);

    for (std::map<Handle,int>::iterator i=diffusionAtomsMap.begin();
            i != diffusionAtomsMap.end(); i++) {
        Handle dAtom = (*i).first;
        (*stiVector)((*i).second) = a->getNormalisedZeroToOneSTI(dAtom,false);
#ifdef DEBUG
        totalSTI += a->getSTI(dAtom);
#endif
    }
    
#ifdef DEBUG
    if (logger().getLevel() >= Logger::FINE) {
        logger().fine("Initial normalised STI values");
        printVector(stiVector);
    }
#endif
}

void ImportanceDiffusionAgent::makeConnectionMatrix(bmatrix* &connections_,
        int totalDiffusionAtoms, std::map<Handle,int> diffusionAtomsMap,
        std::vector<Handle> links)
{
    std::vector<Handle>::iterator hi;
    // set connectivity matrix size, size is dependent on the number of atoms
    // that are connected by a HebbianLink in some way.
//    connections = gsl_matrix_alloc(totalDiffusionAtoms, totalDiffusionAtoms);
    connections_ = new bmatrix(totalDiffusionAtoms, totalDiffusionAtoms, totalDiffusionAtoms * totalDiffusionAtoms);
    // To avoid having to dereference pointers everywhere
    bmatrix& connections = *connections_;
    
    // obviously the use of sparse matrixes means this isn't necessary
//    gsl_matrix_set_zero(connections);

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

        targets = dynamic_cast<Link*>(TLB::getAtom(*hi))->getOutgoingSet();
        if (ClassServer::isAssignableFrom(ORDERED_LINK,type)) {
            Handle sourceHandle;

            // Add only the source index
            sourcePosItr = diffusionAtomsMap.find(targets[0]);
#ifdef DEBUG
            if (sourcePosItr == diffusionAtomsMap.end()) {
                // This case should never occur
                logger().warn("Can't find source in list of diffusionNodes. "
                        "Something odd has happened");
            }
#endif
            sourceHandle = (*sourcePosItr).first;
            // If source atom isn't within diffusionThreshold, then skip
            if (!spreadDecider->spreadDecision(a->getSTI(sourceHandle))) {
                continue;
            }
            sourceIndex = (*sourcePosItr).second;
#ifdef DEBUG
            logger().fine("Ordered link with source index %d.", sourceIndex);
#endif
            // Then spread from index 1 (since source is at index 0)
            
            targetItr = targets.begin();
            targetItr++;
            for (; targetItr != targets.end();
            //for (targetItr = (targets.begin())++; targetItr != targets.end(); 
                     targetItr++) {
                targetPosItr = diffusionAtomsMap.find(*targetItr);
                targetIndex = (*targetPosItr).second;
                if (type == INVERSE_HEBBIAN_LINK) {
                    // source and target indices swapped because inverse
                    //gsl_matrix_set(connections,sourceIndex,targetIndex,val);
                    connections(sourceIndex,targetIndex) = val;
                } else {
                    //gsl_matrix_set(connections,targetIndex,sourceIndex,val);
                    connections(targetIndex,sourceIndex) = val;
                }
            }
        } else {
            std::vector<Handle>::iterator sourceItr;
            // Add the indices of all targets as sources
            // then go through all pairwise combinations
#ifdef DEBUG
            logger().fine("Unordered link");
#endif
            for (sourceItr = targets.begin(); sourceItr != targets.end();
                    sourceItr++) {
                Handle sourceHandle;
                sourceHandle = (*sourceItr);
                // If source atom isn't within diffusionThreshold, then skip
                if (!spreadDecider->spreadDecision(a->getSTI(sourceHandle))) {
                    continue;
                }
                for (targetItr = targets.begin(); targetItr != targets.end();
                        targetItr++) {
                    if (*targetItr == *sourceItr) continue;
                    sourcePosItr = diffusionAtomsMap.find(*sourceItr);
                    sourceIndex = (*sourcePosItr).second;
                    targetPosItr = diffusionAtomsMap.find(*targetItr);
                    targetIndex = (*targetPosItr).second;
                    if (type == SYMMETRIC_INVERSE_HEBBIAN_LINK) {
                        connections(sourceIndex,targetIndex) = val;
                    } else {
                        connections(targetIndex,sourceIndex) = val;
                    }
                }
            }
        }
    }
    // Make sure columns sum to 1.0 and that no more than maxSpreadPercentage
    // is taken from source atoms
#ifdef DEBUG
    logger().fine("Sum probability for column:");
#endif
    for (unsigned int j = 0; j < connections.size2(); j++) {
        double sumProb = 0.0f;
        for (unsigned int i = 0; i < connections.size1(); i++) {
            if (i != j) sumProb += connections(i,j);
        }
#ifdef DEBUG
        logger().fine("%d before - %1.3f", j, sumProb);
#endif

        if (sumProb > maxSpreadPercentage) {
            for (unsigned int i = 0; i < connections.size1(); i++) {
                connections(i,j) = connections(i,j)
                        / (sumProb/maxSpreadPercentage) ;
            }
            sumProb = maxSpreadPercentage;
        }
#ifdef DEBUG
        logger().fine("%d after - %1.3f", j, sumProb);
#endif
        //gsl_matrix_set(connections,j,j,1.0-sumProb);
        connections(j,j) = 1.0-sumProb;
    }
    
#ifdef DEBUG
    logger().debug("Hebbian connection matrix");
    if (logger().getLevel() >= Logger::FINE) {
        printMatrix(connections_);
    }
#endif
}

void ImportanceDiffusionAgent::spreadImportance()
{
    bmatrix* connections;
    bvector* stiVector;
//    bvector* result;
//    int errorNo;

    // The source and destinations of STI
    std::map<Handle,int> diffusionAtomsMap;
    int totalDiffusionAtoms = 0;

    std::vector<Handle> links;
    std::vector<Handle>::iterator hi;
    std::back_insert_iterator< std::vector<Handle> > out_hi(links);

    logger().debug("Begin diffusive importance spread.");

    // Get all HebbianLinks
    a->getHandleSet(out_hi, HEBBIAN_LINK, true);

    totalDiffusionAtoms = makeDiffusionAtomsMap(diffusionAtomsMap, links);

    // No Hebbian Links or atoms?
    if (totalDiffusionAtoms == 0) { return; }

#ifdef DEBUG
    logger().debug("%d total diffusion atoms.", totalDiffusionAtoms);
    logger().fine("Creating norm sti vector.");
#endif

    makeSTIVector(stiVector,totalDiffusionAtoms,diffusionAtomsMap);

    makeConnectionMatrix(connections, totalDiffusionAtoms, diffusionAtomsMap, links);

//    result = gsl_vector_alloc(totalDiffusionAtoms);
//    errorNo = gsl_blas_dgemv(CblasNoTrans,1.0,connections,stiVector,0.0,result);

//    result = new bvector(totalDiffusionAtoms);
//    result = new bvector(prod(*connections, *stiVector));
    bvector result = prod(*connections, *stiVector);
    
/*    if (errorNo) {
        logger().error("%s\n", gsl_strerror (errorNo)); // XXX
    }*/

    if (logger().getLevel() >= Logger::FINE) {
        float normAF;
        normAF = (a->getAttentionalFocusBoundary() - a->getMinSTI(false)) / (float) ( a->getMaxSTI(false) - a->getMinSTI(false) );
        logger().fine("Result (AF at %.3f)\n",normAF);
        printVector(&result);
    }

    // set the sti of all atoms based on new values in results vector from
    // multiplication 
#ifdef DEBUG
    int totalSTI_After = 0;
#endif
    for (std::map<Handle,int>::iterator i=diffusionAtomsMap.begin();
            i != diffusionAtomsMap.end(); i++) {
        Handle dAtom = (*i).first;
//        double val = gsl_vector_get(result,(*i).second);
        double val = result((*i).second);
        setScaledSTI(dAtom,val);
#ifdef DEBUG
        totalSTI_After += a->getSTI(dAtom);
#endif
    }
#ifdef DEBUG
    if (totalSTI != totalSTI_After) {
        logger().warn("Total STI before diffusion (%d) != Total STI after (%d)",totalSTI,totalSTI_After);
    }
#endif

    // free memory!
/*    gsl_matrix_free(connections);
    gsl_vector_free(stiVector);
    gsl_vector_free(result);*/
    delete connections;
    delete stiVector;
//    delete result;
}

void ImportanceDiffusionAgent::setScaledSTI(Handle h, float scaledSTI)
{
    AttentionValue::sti_t val;

    val = a->getMinSTI(false) + (scaledSTI * ( a->getMaxSTI(false) - a->getMinSTI(false) ));
    a->setSTI(h,val);
    
}

void ImportanceDiffusionAgent::printMatrix(bmatrix* m) {

/*    for (unsigned int i = 0; i < m->size1; i++) {
        for (unsigned int j = 0; j < m->size2; j++) {
            float val = gsl_matrix_get(m,i,j);
            if (val != 0.0) 
                printf("%4.2f ", gsl_matrix_get(m,i,j));
            else printf("---- ");
        }
        printf("\n");
    }*/
    std::cout << m << std::endl;
}

void ImportanceDiffusionAgent::printVector(bvector* v) {

/*    printf("[");
    for (unsigned int i = 0; i < m->size; i++) {
        printf("%4.2f\n ", gsl_vector_get(m,i));;
    }
    printf("]\n");*/
    std::cout << v << std::endl;
}

// Static/Shared random number generator
RandGen* SpreadDecider::rng = NULL;

bool SpreadDecider::spreadDecision(AttentionValue::sti_t s)
{
    if (getRNG()->randfloat() < function(s))
        return true;
    else
        return false;
}

RandGen* SpreadDecider::getRNG() {
    if (!rng)
        rng = new opencog::MT19937RandGen((unsigned long) time(NULL));
    return rng;
}

float HyperbolicDecider::function(AttentionValue::sti_t s)
{
    return (tanh(shape*(s-focusBoundary))+1.0f)/2.0f;
}

float StepDecider::function(AttentionValue::sti_t s)
{
    return (s>focusBoundary ? 1.0f : 0.0f);
}

};

#endif // HAVE_GSL
