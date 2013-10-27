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
#include <opencog/atomspace/Handle.h>

#ifdef ZMQ_EXPERIMENT
	#include "ProtocolBufferSerializer.h"
#endif

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class Atom;
class AtomSpaceImpl;
class AttentionBank;

//! stores attention in three components: short-term, long-term and very long-term
class AttentionValue;
typedef std::shared_ptr<AttentionValue> AttentionValuePtr;
class AttentionValue
{
#ifdef ZMQ_EXPERIMENT
    friend class ProtocolBufferSerializer;
#endif

public:
    typedef short sti_t;   //!< short-term importance type
    typedef short lti_t;   //!< long-term importance type
    typedef unsigned short vlti_t; //!< very long-term importance type

    static const int DISPOSABLE = 0; //!< Status flag for vlti

    // CLASS CONSTANTS
    static const sti_t DEFAULTATOMSTI;   //!< short-term importance default
    static const lti_t DEFAULTATOMLTI;   //!< long-term importance default
    static const vlti_t DEFAULTATOMVLTI; //!< very long-term default

    static const sti_t MAXSTI = SHRT_MAX;
    static const lti_t MAXLTI = SHRT_MAX;
    static const sti_t MINSTI = SHRT_MIN;
    static const lti_t MINLTI = SHRT_MIN;

	//! to be used as default attention value
    static AttentionValuePtr DEFAULT_AV() {
        static AttentionValuePtr instance = 
            std::make_shared<AttentionValue>(DEFAULTATOMSTI, 
                               DEFAULTATOMLTI, 
                               DEFAULTATOMVLTI);
        return instance;
    }

private:

    //CLASS FIELDS
    sti_t m_STI;   //!< short-term importance
    lti_t m_LTI;   //!< long-term importance
    vlti_t m_VLTI; //!< represents the number of processes that currently need the
                   //!< atom as nondisposable. So it's only disposable if this is 0
    static AttentionValuePtr m_defaultAV; //! default attention value

public:
    virtual ~AttentionValue() {}
    // CLASS CONSTRUCTORS

	/**
     * @param STI (int): The STI value to set for the atom
     * @param LTI (int): The LTI value to set for the atom
     * @param VLTI (unsigned short): The VLTI value to set for this atom
     */
    AttentionValue(sti_t STI = DEFAULTATOMSTI,
                   lti_t LTI = DEFAULTATOMLTI,
                   vlti_t VLTI = DEFAULTATOMVLTI);

    // PUBLIC GET/SET PROPERTIES

    //! return STI property value
    virtual sti_t getSTI() const;
    virtual float getScaledSTI() const;

    //! return LTI property value
    virtual lti_t getLTI() const;

    //! return VLTI property value
    virtual vlti_t getVLTI() const;

    // PUBLIC METHODS

    //! Decays short term importance
    void  decaySTI();

    //! Returns const string "[sti_val, lti_val, vlti_val]"
    // @param none
    virtual std::string toString() const;

    //! Returns An AttentionValue* cloned from this AttentionValue
    // @param none
    virtual AttentionValuePtr clone() const;

    //! Compares two AttentionValues and returns true if the
    //! elements are equal false otherwise
    // @param none
    virtual bool operator==(const AttentionValue& av) const;
    inline bool operator!=(const AttentionValue& rhs) const
         { return !(*this == rhs); }

	//! functor for comparing atom's attention value
    struct STISort : public AtomComparator  {
        STISort() {};
        virtual bool test(AtomPtr, AtomPtr);
    };

	//! functor for comparing atom's attention value
    struct LTIAndTVAscendingSort : public AtomComparator  {
        LTIAndTVAscendingSort() {};
        virtual bool test(AtomPtr, AtomPtr);
    };

	//! functor for comparing atom's attention value
    struct LTIThenTVAscendingSort : public AtomComparator {
        LTIThenTVAscendingSort() {};
        virtual bool test(AtomPtr, AtomPtr);
    };


    // STATIC METHODS

    //! Returns a shared AttentionValue with default STI, LTI, VLTI values
    // @param none
    static AttentionValuePtr getDefaultAV();

	//!@{
    //! factory method
    static AttentionValuePtr factory();
    static AttentionValuePtr factory(sti_t);
    static AttentionValuePtr factory(float scaledSti);
    static AttentionValuePtr factory(sti_t, lti_t);
    static AttentionValuePtr factory(sti_t, lti_t, vlti_t);
    //!@}
};
#define createAV std::make_shared<AttentionValue>


//! envelope for an AttentionValue
class AttentionValueHolder 
    : public std::enable_shared_from_this<AttentionValueHolder> 
{
    friend class AtomSpaceImpl;
    friend class AttentionBank;
#ifdef ZMQ_EXPERIMENT
    friend class ProtocolBufferSerializer;
#endif

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

typedef std::shared_ptr<AttentionValueHolder> AttentionValueHolderPtr;

/** @}*/
} // namespace opencog 

#endif // _OPENCOG_ATTENTION_VALUE_H
