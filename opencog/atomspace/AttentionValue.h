/*
 * opencog/atomspace/AttentionValue.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Tony Lofthouse <tony_lofthouse@btinternet.com>
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

#ifndef _OPENCOG_ATTENTION_VALUE_H
#define _OPENCOG_ATTENTION_VALUE_H

#include <string>

#include <limits.h>

#include <opencog/atomspace/types.h>

namespace opencog
{

class AtomSpaceImpl;

struct AttentionValue {

public:
    typedef short sti_t;   // short-term importance type
    typedef short lti_t;   // long-term importance type
    typedef unsigned short vlti_t; // very long-term importance type

    static const int DISPOSABLE = 0; //Status flag for vlti
    static const int NONDISPOSABLE = 1; //Status flag for vlti

    // CLASS CONSTANTS
    static const sti_t DEFAULTATOMSTI = 0;
    static const lti_t DEFAULTATOMLTI = 0;
    static const vlti_t DEFAULTATOMVLTI = DISPOSABLE;

    static const sti_t MAXSTI = SHRT_MAX;
    static const lti_t MAXLTI = SHRT_MAX;
    static const sti_t MINSTI = SHRT_MIN;
    static const lti_t MINLTI = SHRT_MIN;

    static const AttentionValue& DEFAULT_AV() {
        static AttentionValue* instance = 
            new AttentionValue(DEFAULTATOMSTI, 
                               DEFAULTATOMLTI, 
                               DEFAULTATOMVLTI);
        return *instance;
    }

private:

    //CLASS FIELDS
    sti_t m_STI;
    lti_t m_LTI;
    vlti_t m_VLTI; //Needs to be chnaged to a bit field after debugging
    static AttentionValue* m_defaultAV;

public:
    virtual ~AttentionValue() {}
    // CLASS CONSTRUCTORS

    // @param int STI: The STI value to set for the atom
    // @param int LTI: The LTI value to set for the atom
    // @param unsigned short VLTI: The VLTI flag value to set for this atom
    AttentionValue(sti_t STI = DEFAULTATOMSTI,
                   lti_t LTI = DEFAULTATOMLTI,
                   vlti_t VLTI = DEFAULTATOMVLTI);

    // PUBLIC GET/SET PROPERTIES

    // return STI property value
    virtual sti_t getSTI() const;
    virtual float getScaledSTI() const;

    // return LTI property value
    virtual lti_t getLTI() const;

    // return VLTI property value
    virtual vlti_t getVLTI() const;

    // PUBLIC METHODS

    // Decays short term importance
    void  decaySTI();

    // Returns const string "[sti_val, lti_val, vlti_val]"
    // @param none
    virtual std::string toString() const;

    // Returns An AttentionValue* cloned from this AttentionValue
    // @param none
    virtual AttentionValue* clone() const;

    // Compares two AttentionValues and returns true if the
    // elements are equal false otherwise
    // @param none
    virtual bool operator==(const AttentionValue& av) const;
    inline bool operator!=(const AttentionValue& rhs) const
         { return !(*this == rhs); }

    template <typename T>
    struct STISort {
        T *a;
        STISort(T *_a): a(_a) {};
        bool operator()(const Handle& h1, const Handle& h2)
        {
            return a->getAV(h1).getSTI() > a->getAV(h2).getSTI();
        }

    };

    template <typename T>
    struct LTIAndTVAscendingSort {
        T *a;
        LTIAndTVAscendingSort(T *_a): a(_a) {};
        bool operator()(const Handle& h1, const Handle& h2)
        {
            lti_t lti1, lti2;
            float tv1, tv2;

            tv1 = fabs(a->getTV(h1).getMean());
            tv2 = fabs(a->getTV(h2).getMean());

            lti1 = a->getAV(h1).getLTI();
            lti2 = a->getAV(h2).getLTI();

            if (lti1 < 0)
                tv1 = lti1 * (1.0f - tv1);
            else
                tv1 = lti1 * tv1;

            if (lti2 < 0)
                tv2 = lti2 * (1.0f - tv2);
            else
                tv2 = lti2 * tv2;

            return tv1 < tv2;
        }
    };

    template <typename T>
    struct LTIThenTVAscendingSort {
        T *a;
        LTIThenTVAscendingSort(T *_a): a(_a) {};
        bool operator()(const Handle& h1, const Handle& h2)
        {
            lti_t lti1, lti2;
            lti1 = a->getAV(h1).getLTI();
            lti2 = a->getAV(h2).getLTI();
            if (lti1 != lti2) return lti1 < lti2;

            float tv1, tv2;
            tv1 = a->getTV(h1).getMean();
            tv2 = a->getTV(h2).getMean();
            return tv1 < tv2;
        }
    };

    // STATIC METHODS

    // Returns a shared AttentionValue with default STI, LTI, VLTI values
    // @param none
    static const AttentionValue& getDefaultAV();

    // factory methods
    static AttentionValue* factory();
    static AttentionValue* factory(sti_t sti);
    static AttentionValue* factory(float scaledSti);
    static AttentionValue* factory(sti_t sti, lti_t lti);
    static AttentionValue* factory(sti_t sti, lti_t lti, vlti_t vlti);
};

class AttentionValueHolder
{
    friend class AtomSpace;
    friend class AtomSpaceImpl;

protected:
    AttentionValue attentionValue;

    /** Sets the AttentionValue object */
    virtual void setAttentionValue(const AttentionValue &a) {
        attentionValue = a;
    }


public:
    /** Returns the AttentionValue object */
    virtual const AttentionValue& getAttentionValue() const {
        return attentionValue;
    }
};

} // namespace opencog 

#endif // _OPENCOG_ATTENTION_VALUE_H
