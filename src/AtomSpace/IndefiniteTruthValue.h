/*
 * src/AtomSpace/IndefiniteTruthValue.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
 *            Fabricio Silva <fabricio@vettalabs.com>
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

#ifndef _INDEFINITE_TRUTH_VALUE_H_
#define _INDEFINITE_TRUTH_VALUE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "TruthValue.h"
#include "utils.h"
#include <vector>

using namespace std;

/*
 * Indefinite probabilities are in the form ([L,U],b,N). In practical work,
 * N will be hold constant and thus we have only ([L,U],b).
 */
class IndefiniteTruthValue : public TruthValue
{

private:
    float mean;
    float U;
    float L;
    float confidenceLevel; //referred as "b" in the paper
    bool symmetric;
    float diff;//used in inference rule procedure in order to compute L1 and U1

    vector<float*> firstOrderDistribution;

    void init(float c = 0.0f, float d = 0.0f, float e = DEFAULT_CONFIDENCE_LEVEL);
    void copy(const IndefiniteTruthValue&);

public:
    IndefiniteTruthValue();
    IndefiniteTruthValue(float, float, float c = DEFAULT_CONFIDENCE_LEVEL);
    IndefiniteTruthValue(IndefiniteTruthValue const&);

    IndefiniteTruthValue* clone() const;
    IndefiniteTruthValue& operator=(const TruthValue& rhs) throw (RuntimeException);

    virtual bool operator==(const TruthValue& rhs) const;

    static IndefiniteTruthValue* fromString(const char*);

    float getMean() const;
    float getU() const;
    float getL() const;
    float getConfidenceLevel() const;
    float getDiff() const;
    vector<float*> getFirstOrderDistribution() const;

    void setMean(float);
    void setU(float);
    void setL(float);
    void setConfidenceLevel(float);
    void setDiff(float);
    void setFirstOrderDistribution(vector<float*>);

    float getCount() const;
    float getConfidence() const;
    float getU_() const;
    float getL_() const;
    bool isSymmetric() const;

    float toFloat() const;
    std::string toString() const;
    TruthValueType getType() const;

    static float DEFAULT_CONFIDENCE_LEVEL;
    static float DEFAULT_K;
    static void setDefaultConfidenceLevel(float c) {
        DEFAULT_CONFIDENCE_LEVEL = c;
    }
    static void setDefaultK(float k) {
        DEFAULT_K = k;
    }
};
#endif /*_INDEFINITE_TRUTH_VALUE_H_*/
