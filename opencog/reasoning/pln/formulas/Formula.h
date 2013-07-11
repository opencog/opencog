/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#ifndef _INFERENCE_FORMULA
#define _INFERENCE_FORMULA

#include <iostream>
#include <cstdio>
#include <string>

#include <boost/shared_ptr.hpp>

#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/CompositeTruthValue.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>

#define LIMIT_TRUTH_VALUES 1
#define IS_NAN(f) (!((f)==(f)))

#define DefaultU 10000

using namespace opencog;

namespace opencog {
namespace pln {

typedef boost::shared_ptr<TruthValue> TruthValuePtr;
typedef boost::shared_ptr<IndefiniteTruthValue> IndefiniteTruthValuePtr;

typedef std::vector<TruthValuePtr> TVSeq;

class ArityFreeFormula
{
public:

    /***
     * One or more of the following member functions should be
     * overridden in deriving classes; the general form is:
     *
     *   ResultType compute(const TruthValue* tv1,...,const TruthValue* tvN) const;
     *
     * for 1<=N<=4. For larger values, the general N-arry case is:
     *
     *   ResultType compute(TVSeqType tvs) const;
     ***/

    virtual TruthValue* compute(const TVSeq&, long U = DefaultU) const = 0;

    /**
     U is the universe size...
    */


    /**
     * That method calls compute above but appends TVsuper to TVsub and sum
     * Nsub and Nsuper
     *
     * Note I call it compute2 because gcc apparently get confused
     * between long U and TVType**, it should work fine once TVType** is replaced
     * by stl container
     */
    virtual TruthValue* compute2(const TVSeq& TVsub, const TVSeq& TVsuper,
                                 long U = DefaultU) const {
        OC_ASSERT(TVsub.size() == TVsuper.size(),
                  "That is the current assumption otherwise it is hard to know"
                  " what is TVsub and what is TVsuper in TVs");

        TVSeq TVs(TVsub.begin(), TVsub.end());
        TVs.insert(TVs.end(), TVsuper.begin(), TVsuper.end());

        return compute(TVs, U);
    }

public:

    ///a simple error hanlder; may be overridden
    virtual void handleError(const std::string& error) const {
        std::cerr << "ERROR: " << error << std::endl;
        exit(1);
    }

    virtual ~ArityFreeFormula() {};

    TruthValue* checkTruthValue(TruthValue* tv) const {
        //if (Log::getDefaultLevel() >= 3) printf("\nRaw result: %f", tv->getMean());

        if (LIMIT_TRUTH_VALUES) {
            if (tv->getMean() < 0) {
                if (tv->getType() == SIMPLE_TRUTH_VALUE) {
                    ((SimpleTruthValue*)tv)->setMean(0);
                } else if (tv->getType() == INDEFINITE_TRUTH_VALUE) {
                    ((IndefiniteTruthValue*)tv)->setMean(0);
                } else {
                    throw new RuntimeException(TRACE_INFO, "Cannot limit truth values of this type of TV\n");
                }
            }
            if (tv->getMean() > 1) {
                if (tv->getType() == SIMPLE_TRUTH_VALUE) {
                    ((SimpleTruthValue*)tv)->setMean(1);
                } else if (tv->getType() == INDEFINITE_TRUTH_VALUE) {
                    ((IndefiniteTruthValue*)tv)->setMean(1);
                } else {
                    throw new RuntimeException(TRACE_INFO, "Cannot limit truth values of this type of TV\n");
                }
            }
            if (tv->getCount() < 0) {
                if (tv->getType() == SIMPLE_TRUTH_VALUE) {
                    ((SimpleTruthValue*)tv)->setCount(0);
                } else {
                    throw new RuntimeException(TRACE_INFO, "Cannot limit truth values of this type of TV\n");
                }
            }
        } else {
            if ((tv->getMean() < 0)
                || (tv->getMean() > 1)
                || (tv->getCount() < 0))
                return NULL;
        }

        if ((IS_NAN(tv->getMean())) || (IS_NAN(tv->getCount()))) {
            return NULL;
        }

        return tv;
    }

};


/** @class Formula
 _TVN is the arity.
*/

template<int _TVN>
class Formula : public ArityFreeFormula
{
    friend class SubsetEvalFormula;
public:
    const int TVN;
    Formula() : TVN(_TVN) {}

    /**
     * Simple compute do not handle CompositeTruthValues.
     * It's called by compute() method,
     * which handles CTVs.
     */
    virtual TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;

    /**
     * Partition the TV input to N/TVN groups of TVN entries.
     * Calculate the formula for each group.
     * Combine the results into a single array of TVs.
     * The size of the array is N/TVN.
     * Like simpleCompute, this method does not handle CTVs.
     */
    virtual TVSeq multiCompute(const TVSeq& TV, long U = DefaultU) const;

    /**
     * Check if there is any CompositeTruthValue object in the array
     * of TruthValues and,
     * if so, computes different results for primaryTVs
     * and for each handle-versioned TVS
     */
    TruthValue* compute(const TVSeq& TV, long U = DefaultU) const;
};


}} //~namespace opencog::pln
#endif
