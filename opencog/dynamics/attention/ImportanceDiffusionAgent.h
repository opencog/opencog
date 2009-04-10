/*
 * opencog/dynamics/attention/ImportanceDiffusionAgent.h
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

#ifndef _OPENCOG_IMPORTANCE_DIFFUSION_AGENT_H
#define _OPENCOG_IMPORTANCE_DIFFUSION_AGENT_H

#include <string>

#include <math.h>
//#include <gsl/gsl_matrix.h>
//#include <gsl/gsl_vector.h>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/server/Agent.h>
#include <opencog/util/Logger.h>
#include <opencog/util/RandGen.h>

typedef boost::numeric::ublas::vector<float> bvector;
typedef boost::numeric::ublas::compressed_matrix<float> bmatrix;

namespace opencog
{

class CogServer;

class SpreadDecider
{
    virtual float function(AttentionValue::sti_t s) = 0;

protected:
    static RandGen *rng;

public:
    SpreadDecider() {}
    virtual ~SpreadDecider() {}

    virtual void setFocusBoundary(float b = 0.0f) {};
    RandGen* getRNG();

    bool spreadDecision(AttentionValue::sti_t s);
};

class HyperbolicDecider : SpreadDecider
{
    float function(AttentionValue::sti_t s);
public:
    float shape;
    float focusBoundary;
    HyperbolicDecider(float _s=10.0f):
        shape(_s), focusBoundary(0.0f) {}
    virtual ~HyperbolicDecider() {}
    void setFocusBoundary(float b = 0.0f);
};

class StepDecider : SpreadDecider
{
    float function(AttentionValue::sti_t s);

public:
    int focusBoundary;
    StepDecider() {}
    virtual ~StepDecider() {}
    void setFocusBoundary(float b = 0.0f);
};

/** Spreads short term importance along HebbianLinks using a diffusion approach.
 *
 * Spreads along:
 * \arg SymmetricHebbianLinks
 * \arg InverseHebbianLinks
 * \arg AsymmetricHebbianLink
 * \arg SymmetricInverseHebbianLink
 *
 * @todo Optionally spread long term importance?
 */
class ImportanceDiffusionAgent : public Agent
{

private:
    AtomSpace* a;

    //! Total amount spread during recent runs.
    opencog::recent_val<long> amountSpread;

    //! Maximum percentage of importance to spread
    float maxSpreadPercentage;

    //! Value that normalised STI has to be above before being spread
    //! Is a normalised value from -1 to 1. 0 == AF
    float diffusionThreshold;

    //! Spread importance along Hebbian links.
    //! @todo split into sub functions instead of one giant beast.
    void spreadImportance();

    //! print a gsl matrix to stdout
    void printMatrix(bmatrix *m);
    //! print a gsl vector to stdout
    void printVector(bvector *m, float threshold = 1.0f);

    //! Set the STI of h from a scaled 0..1 STI value
    void setScaledSTI(Handle h, float scaledSTI);

    //! Map each atom involved in important diffusion with an index
    int makeDiffusionAtomsMap(std::map<Handle,int> &i,std::vector<Handle> links);

    //! Make vector of original scaled STI values
    void makeSTIVector(bvector* &stiVector, int totalDiffusionAtoms,
            std::map<Handle,int> diffusionAtomsMap);

    //! Construct matrix representing HebbianLinks between all atoms
    //! in diffusionAtomsMap.
    void makeConnectionMatrix(bmatrix* &connections, int totalDiffusionAtoms,
            std::map<Handle,int> diffusionAtomsMap, std::vector<Handle> links);

    SpreadDecider* spreadDecider;

    //! For checking that STI is conserved
    int totalSTI;
public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::ImportanceDiffusionAgent");
        return _ci;
    }

    enum { HYPERBOLIC, STEP };
    void setSpreadDecider(int type, float shape = 30);

    ImportanceDiffusionAgent();
    virtual ~ImportanceDiffusionAgent();
    virtual void run(CogServer *server);

    /** Set the maximum percentage of importance that can be spread.
     * @param p the maximum percentage of importance that can be spread.
     */
    void setMaxSpreadPercentage(float p);

    /** Get the maximum percentage of importance that can be spread.
     * @return the maximum percentage of importance that can be spread.
     */
    float getMaxSpreadPercentage() const;

    void setDiffusionThreshold(float p);
    float getDiffusionThreshold() const;
}; // class

}; // namespace

#endif // _OPENCOG_IMPORTANCE_DIFFUSION_AGENT_H
