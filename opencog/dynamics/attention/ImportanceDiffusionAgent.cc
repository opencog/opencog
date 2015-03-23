/*
 * opencog/dynamics/attention/ImportanceDiffusionAgent.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
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
#include "ImportanceDiffusionAgent.h"

#include <time.h>
#include <math.h>
//#include <gsl/gsl_matrix_float.h>
//#include <gsl/gsl_linalg.h>
//#include <gsl/gsl_blas.h>

#include <opencog/atomspace/Link.h>
#include <opencog/dynamics/attention/atom_types.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/platform.h>
#include <opencog/util/mt19937ar.h>

#define DEBUG
namespace opencog
{

ImportanceDiffusionAgent::ImportanceDiffusionAgent(CogServer& cs) :
    Agent(cs)
{
    static const std::string defaultConfig[] = {
        //! Default value that normalised STI has to be above before
        //! being spread
        "ECAN_DIFFUSION_THRESHOLD","0.0",
        //! Maximum percentage of STI that is spread from an atom
        "ECAN_MAX_SPREAD_PERCENTAGE","1.0",
        "ECAN_ALL_LINKS_SPREAD","false",
        "",""
    };
    setParameters(defaultConfig);
    spreadDecider = NULL;

    //! @todo won't respond to the parameters being changed later
    //! (not a problem at present, but could get awkward with, for example,
    //! automatic parameter adaptation)
    maxSpreadPercentage = (float) (config().get_double("ECAN_MAX_SPREAD_PERCENTAGE"));

    setSpreadDecider(STEP);
    setDiffusionThreshold((float) (config().get_double("ECAN_DIFFUSION_THRESHOLD")));

    allLinksSpread = config().get_bool("ECAN_ALL_LINKS_SPREAD");

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("ImportanceDiffusionAgent.log", Logger::FINE, true));
}

void ImportanceDiffusionAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* ImportanceDiffusionAgent::getLogger()
{
    return log;
}

void ImportanceDiffusionAgent::setSpreadDecider(int type, float shape)
{
    if (spreadDecider) {
        delete spreadDecider;
        spreadDecider = NULL;
    }
    switch (type) {
    case HYPERBOLIC:
        spreadDecider = (SpreadDecider*) new HyperbolicDecider(_cogserver, shape);
        break;
    case STEP:
        spreadDecider = (SpreadDecider*) new StepDecider(_cogserver);
        break;
    }
    
}

ImportanceDiffusionAgent::~ImportanceDiffusionAgent()
{
    if (spreadDecider) {
        delete spreadDecider;
        spreadDecider = NULL;
    }
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

void ImportanceDiffusionAgent::run()
{
    a = &_cogserver.getAtomSpace();
    spreadDecider->setFocusBoundary(diffusionThreshold);
#ifdef DEBUG
    totalSTI = 0;
#endif
    spreadImportance();
}

#define toFloat getMean

int ImportanceDiffusionAgent::makeDiffusionAtomsMap(std::map<Handle,int> &diffusionAtomsMap,
        std::vector<Handle> links)
{
    int totalDiffusionAtoms = 0;
    std::vector<Handle>::iterator hi;

#ifdef DEBUG
    log->fine("Finding atoms involved with STI diffusion and their matrix indices");
#endif
    for (hi = links.begin(); hi != links.end(); ++hi) {
        // Get all atoms in outgoing set of links
        std::vector<Handle> targets;
        std::vector<Handle>::iterator targetItr;
        Handle linkHandle = *hi;
        float val = a->getTV(linkHandle)->toFloat();
        if (val == 0.0f) {
            continue;
        }

        targets = a->getOutgoing(linkHandle);

        for (targetItr = targets.begin(); targetItr != targets.end();
                ++targetItr) {
            if (diffusionAtomsMap.find(*targetItr) == diffusionAtomsMap.end()) {
                diffusionAtomsMap[*targetItr] = totalDiffusionAtoms;
#ifdef DEBUG
                log->fine("%s = %d",
                        a->atomAsString(*targetItr).c_str(),
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
            i != diffusionAtomsMap.end(); ++i) {
        Handle dAtom = (*i).first;
// For some reason I thought linearising -ve and +ve separately might
// be a good idea, but this messes up the conservation of STI
//      (*stiVector)((*i).second) = a->getNormalisedSTI(dAtom,false)+1.0f)/2.0f);
        (*stiVector)((*i).second) = a->getNormalisedZeroToOneSTI(dAtom,false);
#ifdef DEBUG
        totalSTI += a->getSTI(dAtom);
#endif
    }
    
#ifdef DEBUG
    if (log->isFineEnabled()) {
        log->fine("Initial normalised STI values");
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

    for (hi=links.begin(); hi != links.end(); ++hi) {
        // Get all atoms in outgoing set of link
        std::vector<Handle> targets;
        std::vector<Handle>::iterator targetItr;

        std::map<Handle,int>::iterator sourcePosItr;
        std::map<Handle,int>::iterator targetPosItr;
        int sourceIndex;
        int targetIndex;
        Type type;

        float val = a->getTV(*hi)->toFloat();
        if (val == 0.0f) continue;
        //val *= diffuseTemperature;
        type = a->getType(*hi); 

        targets = a->getOutgoing(*hi);
        if (classserver().isA(type,ORDERED_LINK)) {
            Handle sourceHandle;

            // Add only the source index
            sourcePosItr = diffusionAtomsMap.find(targets[0]);
#ifdef DEBUG
            if (sourcePosItr == diffusionAtomsMap.end()) {
                // This case should never occur
                log->warn("Can't find source in list of diffusionNodes. "
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
            log->fine("Ordered link with source index %d.", sourceIndex);
#endif
            // Then spread from index 1 (since source is at index 0)
            
            targetItr = targets.begin();
            ++targetItr;
            for (; targetItr != targets.end();
            //for (targetItr = (targets.begin())++; targetItr != targets.end(); 
                     ++targetItr) {
                targetPosItr = diffusionAtomsMap.find(*targetItr);
                targetIndex = (*targetPosItr).second;
                if (type == INVERSE_HEBBIAN_LINK) {
                    // source and target indices swapped because inverse
                    //gsl_matrix_set(connections,sourceIndex,targetIndex,val);
                    connections(sourceIndex,targetIndex) += val;
                } else {
                    //gsl_matrix_set(connections,targetIndex,sourceIndex,val);
                    connections(targetIndex,sourceIndex) += val;
                }
            }
        } else {
            std::vector<Handle>::iterator sourceItr;
            // Add the indices of all targets as sources
            // then go through all pairwise combinations
#ifdef DEBUG
            log->fine("Unordered link");
#endif
            for (sourceItr = targets.begin(); sourceItr != targets.end();
                    ++sourceItr) {
                Handle sourceHandle;
                sourceHandle = (*sourceItr);
                // If source atom isn't within diffusionThreshold, then skip
                if (!spreadDecider->spreadDecision(a->getSTI(sourceHandle))) {
                    continue;
                }
                for (targetItr = targets.begin(); targetItr != targets.end();
                        ++targetItr) {
                    if (*targetItr == *sourceItr) continue;
                    sourcePosItr = diffusionAtomsMap.find(*sourceItr);
                    sourceIndex = (*sourcePosItr).second;
                    targetPosItr = diffusionAtomsMap.find(*targetItr);
                    targetIndex = (*targetPosItr).second;
                    if (type == SYMMETRIC_INVERSE_HEBBIAN_LINK) {
                        connections(sourceIndex,targetIndex) += val;
                    } else {
                        connections(targetIndex,sourceIndex) += val;
                    }
                }
            }
        }
    }
    // Make sure columns sum to 1.0 and that no more than maxSpreadPercentage
    // is taken from source atoms
#ifdef DEBUG
    log->fine("Sum probability for column:");
#endif
    for (unsigned int j = 0; j < connections.size2(); j++) {
        double sumProb = 0.0f;
        for (unsigned int i = 0; i < connections.size1(); i++) {
            if (i != j) sumProb += connections(i,j);
        }
#ifdef DEBUG
        log->fine("%d before - %1.3f", j, sumProb);
#endif

        if (sumProb > maxSpreadPercentage) {
            for (unsigned int i = 0; i < connections.size1(); i++) {
                connections(i,j) = connections(i,j)
                        / (sumProb/maxSpreadPercentage) ;
            }
            sumProb = maxSpreadPercentage;
        }
#ifdef DEBUG
        log->fine("%d after - %1.3f", j, sumProb);
#endif
        //gsl_matrix_set(connections,j,j,1.0-sumProb);
        connections(j,j) = 1.0-sumProb;
    }
    
#ifdef DEBUG
    if (log->isFineEnabled()) {
        log->debug("Hebbian connection matrix:");
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
    std::back_insert_iterator< std::vector<Handle> > out_hi(links);

    log->debug("Begin diffusive importance spread.");

    // Get all HebbianLinks
    if (allLinksSpread) {
      a->getHandlesByType(out_hi, LINK, true);
    } else {
      a->getHandlesByType(out_hi, HEBBIAN_LINK, true);
    }

    totalDiffusionAtoms = makeDiffusionAtomsMap(diffusionAtomsMap, links);

    // No Hebbian Links or atoms?
    if (totalDiffusionAtoms == 0) { return; }

#ifdef DEBUG
    log->debug("%d total diffusion atoms.", totalDiffusionAtoms);
    log->fine("Creating normalized STI vector.");
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

    if (log->isFineEnabled()) {
        float normAF;
        normAF = (a->getAttentionalFocusBoundary() - a->getMinSTI(false)) / (float) ( a->getMaxSTI(false) - a->getMinSTI(false) );
        log->fine("Result (AF at %.3f)\n",normAF);
        printVector(&result,normAF);
//        printVector(result,0.5f);
    }

    // set the sti of all atoms based on new values in results vector from
    // multiplication 
#ifdef DEBUG
    int totalSTI_After = 0;
#endif
    for (std::map<Handle,int>::iterator i=diffusionAtomsMap.begin();
            i != diffusionAtomsMap.end(); ++i) {
        Handle dAtom = (*i).first;
//        double val = gsl_vector_get(result,(*i).second);
        double val = result((*i).second);
        setScaledSTI(dAtom,val);
#ifdef DEBUG
        totalSTI_After += a->getSTI(dAtom);
#endif
    }
#if 0 //def DEBUG
    if (totalSTI != totalSTI_After) {
        // This warning is often logged because of floating point round offs
        // when converting from normalised to actual STI values after diffusion.
        log->warn("Total STI before diffusion (%d) != Total STI after (%d)",totalSTI,totalSTI_After);
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

    val = (AttentionValue::sti_t) (a->getMinSTI(false) + (scaledSTI * ( a->getMaxSTI(false) - a->getMinSTI(false) )));
/*
    AtomSpace *a = _cogserver.getAtomSpace();
    float af = a->getAttentionalFocusBoundary();
    scaledSTI = (scaledSTI * 2) - 1;
    if (scaledSTI <= 1.0) {
        val = a->getMinSTI(false) + (scaledSTI * ( a->getMinSTI(false) - af ));
    } else {
        val = af + (scaledSTI * (a->getMaxSTI(false) - af ));
    }
*/
    a->setSTI(h,val);
    
}

void ImportanceDiffusionAgent::printMatrix(bmatrix* m)
{
    typedef boost::numeric::ublas::compressed_matrix<float>::iterator1 it1_t;
    typedef boost::numeric::ublas::compressed_matrix<float>::iterator2 it2_t;

    for (it1_t it1 = m->begin1(); it1 != m->end1(); it1++) {
        for (it2_t it2 = it1.begin(); it2 != it1.end(); it2++) {
            log->fine("(%d,%d) %f", it2.index1(), it2.index2(), *it2);
        }
    }
}

void ImportanceDiffusionAgent::printVector(bvector* v, float threshold)
{
    typedef boost::numeric::ublas::vector<float>::iterator it_t;

    for (it_t it = v->begin(); it != v->end(); ++it) {
        if (*it > threshold) {
            log->fine("(%d) %f +", it.index(), *it);
        }
        else {
            log->fine("(%d) %f", it.index(), *it);
        }
    }
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

RandGen* SpreadDecider::getRNG()
{
    if (!rng)
        rng = new opencog::MT19937RandGen((unsigned long) time(NULL));
    return rng;
}

double HyperbolicDecider::function(AttentionValue::sti_t s)
{
    AtomSpace& a = _cogserver.getAtomSpace();
    // Convert boundary from -1..1 to 0..1
    float af = a.getAttentionalFocusBoundary();
    float minSTI = a.getMinSTI(false);
    float maxSTI = a.getMaxSTI(false);
    float norm_b = focusBoundary > 0.0f ?
        af + (focusBoundary * (maxSTI - af)) :
        af + (focusBoundary * (af - minSTI ));
    // norm_b is now the actual STI value, normalise to 0..1
    norm_b = (norm_b - minSTI) / (float) ( maxSTI - minSTI );
    // Scale s to 0..1
    float norm_s = (s - minSTI) / (float) ( maxSTI - minSTI );
    return (tanh(shape*(norm_s-norm_b))+1.0f)/2.0f;
}

void HyperbolicDecider::setFocusBoundary(float b)
{
    // Store as -1..1 since exact 0..1 mapping of boundary
    // will change based on min/max STI
    focusBoundary = b;
}

double StepDecider::function(AttentionValue::sti_t s)
{
    return (s>focusBoundary ? 1.0f : 0.0f);
}

void StepDecider::setFocusBoundary(float b)
{
    AtomSpace& a = _cogserver.getAtomSpace();
    // Convert to an exact STI amount
    float af = a.getAttentionalFocusBoundary();
    focusBoundary = (b > 0.0f)?
        (int) (af + (b * (a.getMaxSTI(false) - af))) :
        (int) (af + (b * (af - a.getMinSTI(false))));

}

};

