/*
 * opencog/atomspace/NullTruthValue.cc
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

#include "NullTruthValue.h"

#include <opencog/util/exceptions.h>

using namespace opencog;

NullTruthValue::NullTruthValue() {};

bool NullTruthValue::isNullTv() const
{
    return true;
}

bool NullTruthValue::operator==(const TruthValue& rhs) const
{
    const NullTruthValue *ntv = dynamic_cast<const NullTruthValue *>(&rhs);
    if (ntv) return true;
    return false;
}

std::string NullTruthValue::toString() const
{
    return "(null TV)";
}

float NullTruthValue::getMean() const throw (RuntimeException)
{
    throw RuntimeException(TRACE_INFO, "Cannot call getMean() method of a NullTruthvalue");
}

float NullTruthValue::getCount() const throw (RuntimeException)
{
    throw RuntimeException(TRACE_INFO, "Cannot call getCount() method of a NullTruthvalue");
}

float NullTruthValue::getConfidence() const throw (RuntimeException)
{
    throw RuntimeException(TRACE_INFO, "Cannot call getConfidence() method of a NullTruthvalue");
}

TruthValueType NullTruthValue::getType() const throw (RuntimeException)
{
    throw RuntimeException(TRACE_INFO, "Cannot call getType() method of a NullTruthvalue");
}

TruthValuePtr NullTruthValue::merge(TruthValuePtr) throw (RuntimeException)
{
    throw RuntimeException(TRACE_INFO, "Cannot call merge() method of a NullTruthvalue");
}

TruthValuePtr NullTruthValue::clone() const
{
    const TruthValuePtr nullo(new NullTruthValue());
    return nullo;
}

