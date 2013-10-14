/*
 * opencog/atomspace/NullTruthValue.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#ifndef _OPENCOG_NULL_TRUTH_VALUE_TV_H
#define _OPENCOG_NULL_TRUTH_VALUE_TV_H

#include <opencog/atomspace/TruthValue.h>
#ifdef ZMQ_EXPERIMENT
#include "ProtocolBufferSerializer.h"
#endif

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

//! a special type of TruthValue
class NullTruthValue : public TruthValue
{

    friend class TruthValue;
#ifdef ZMQ_EXPERIMENT
    friend class ProtocolBufferSerializer;
#endif

public:
    NullTruthValue();
    bool isNullTv() const;
    float getMean() const throw (RuntimeException);
    float getCount() const throw (RuntimeException);
    float getConfidence() const  throw (RuntimeException);
    std::string toString() const;
    TruthValueType getType() const throw (RuntimeException);
    TruthValuePtr clone() const;
    TruthValue* rawclone() const;

    virtual bool operator==(const TruthValue& rhs) const;

protected:
    TruthValuePtr merge(TruthValuePtr) throw (RuntimeException);
};

/** @}*/
} // namespace

#endif // _OPENCOG_NULL_TRUTH_VALUE_TV_H
