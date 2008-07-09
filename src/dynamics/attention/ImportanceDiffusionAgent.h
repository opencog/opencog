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

#ifndef _IMPORTANCE_DIFFUSION_AGENT_H
#define _IMPORTANCE_DIFFUSION_AGENT_H

#ifdef HAVE_GSL

#include <string>
#include <math.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include <Logger.h>

#include <AtomSpace.h>
#include <MindAgent.h>
#include <AttentionValue.h>

namespace opencog
{

//! Default value that normalised STI has to be above before being spread
const float MA_DEFAULT_DIFFUSION_THRESHOLD = 0.5f;
//! Maximum percentage of STI that is spread from an atom
const float MA_DEFAULT_MAX_SPREAD_PERCENTAGE = 0.5f;

class CogServer;

/** Spreads short term importance along HebbianLinks using a diffusion approach.
 *
 * Currently spreads along Symmetric and Inverse HebbianLinks.
 *
 * @todo Optionally spread long term importance?
 * @todo Convert from using GSL matrices to a more efficient approach - either
 * using sparse matrices or directly using the STI values to generate a
 * temporary sparse matrix
 */
class ImportanceDiffusionAgent : public MindAgent
{

private:
    AtomSpace* a;

    //! Total amount spread during recent runs.
    opencog::recent_val<long> amountSpread;

    //! Maximum percentage of importance to spread
    float maxSpreadPercentage;

    //! Value that normalised STI has to be above before being spread
    float diffusionThreshold;

    //! Spread importance along Hebbian links.
    //! @todo split into sub functions instead of one giant beast.
    void spreadImportance();

    //! print a gsl matrix to stdout
    void printMatrix(gsl_matrix *m);
    //! print a gsl vector to stdout
    void printVector(gsl_vector *m);

    void setScaledSTI(Handle h, float scaledSTI);

public:

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
#endif // HAVE_GSL
#endif // _IMPORTANCE_DIFFUSION_AGENT_H
