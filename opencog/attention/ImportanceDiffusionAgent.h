/*
 * opencog/attention/ImportanceDiffusionAgent.h
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

#ifndef _OPENCOG_IMPORTANCE_DIFFUSION_AGENT_H
#define _OPENCOG_IMPORTANCE_DIFFUSION_AGENT_H

#include <math.h>
#include <string>

#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <opencog/util/RandGen.h>
#include <opencog/util/recent_val.h>
#include <opencog/cogserver/server/Agent.h>
#include "SpreadDecider.h"

typedef boost::numeric::ublas::vector<double> bvector;
typedef boost::numeric::ublas::compressed_matrix<double> bmatrix;

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

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

    //! Total amount spread during recent runs.
    opencog::recent_val<long> amountSpread;

    //! Maximum percentage of importance to spread
    double maxSpreadPercentage;

    //! Value that normalised STI has to be above before being spread
    //! Is a normalised value from -1 to 1. 0 == AF
    double diffusionThreshold;

    //! Whether to spread STI across all types of Links and not just HebbianLinks.
    //! If there are multiple links between the same two Atoms, then it will add up their strengths.
    bool allLinksSpread;

    //! Spread importance along Hebbian links.
    //! @todo split into sub functions instead of one giant beast.
    void spreadImportance();

    //! print a gsl matrix to stdout
    void printMatrix(bmatrix *m);
    //! print a gsl vector to stdout
    void printVector(bvector *m, double threshold = 1.0);

    //! Set the STI of h from a scaled 0..1 STI value
    void setScaledSTI(Handle h, double scaledSTI);

    //! Map each atom involved in important diffusion with an index
    int makeDiffusionAtomsMap(std::map<Handle,int> &i,HandleSeq links);

    //! Make vector of original scaled STI values
    void makeSTIVector(bvector* &stiVector, int totalDiffusionAtoms,
            std::map<Handle,int> diffusionAtomsMap);

    //! Construct matrix representing HebbianLinks between all atoms
    //! in diffusionAtomsMap.
    void makeConnectionMatrix(bmatrix* &connections, int totalDiffusionAtoms,
            std::map<Handle,int> diffusionAtomsMap, HandleSeq links);

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
    void setSpreadDecider(int type, double shape = 30);

    ImportanceDiffusionAgent(CogServer&);
    virtual ~ImportanceDiffusionAgent();
    virtual void run();

    /** Set the maximum percentage of importance that can be spread.
     * @param p the maximum percentage of importance that can be spread.
     */
    void setMaxSpreadPercentage(double);

    /** Get the maximum percentage of importance that can be spread.
     * @return the maximum percentage of importance that can be spread.
     */
    double getMaxSpreadPercentage() const;

    void setDiffusionThreshold(double);
    double getDiffusionThreshold() const;
}; // class

typedef std::shared_ptr<ImportanceDiffusionAgent> ImportanceDiffusionAgentPtr;

/** @}*/
} // namespace

#endif // _OPENCOG_IMPORTANCE_DIFFUSION_AGENT_H
