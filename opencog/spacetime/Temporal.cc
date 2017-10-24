/*
 * opencog/spacetime/Temporal.cc
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

#include "Temporal.h"

#include <iostream>

#include <limits.h>
#include <stdlib.h>

#include <boost/format.hpp>

#include <opencog/util/platform.h>

//#define DPRINTF printf
#define DPRINTF(...)

using boost::format;
using boost::str;

using namespace opencog;

std::ostream& operator<<(std::ostream& out, const Temporal& t)
{
    return (out << t.toString());
}

// Values of A, B and NORMAL for the UNDEFINED_TEMPORAL Temporal instance
// These values should not compose a valid Temporal, btw.
// In other words, if someone pass these values for a Temporal constructor it will throw an exception.
#define UNDEFINED_A OCTIME_MAX
#define UNDEFINED_B OCTIME_MAX
#define UNDEFINED_NORMAL true

// USED TO SEEK MEMORY LEAK
//int Temporal::numTemporals = 0;

void Temporal::init(octime_t a, octime_t b, bool normal)
{

    // USED TO SEEK MEMORY LEAK
    //numTemporals++;
    //cout << "Total temporals: " << numTemporals << endl;

    // sanity checks
    if (normal) {
        if (b > a) {
            throw InvalidParamException(TRACE_INFO,
                                        str(format("Cannot create a Temporal (normal-distribution) with the variance (%1%) greater than the mean (%2%). This causes negative lower bound.") % a % b).c_str());
        }
        octime_t sum = a + b;
        // if the addition caused an overflow
        if (sum < a) {
            throw InvalidParamException(TRACE_INFO,
                                        str(format("Temporal - Upper bound reached when creating a Temporal (normal-distribution): %1%.") % sum).c_str());
        }
    } else {
        if (a > b) {
            throw InvalidParamException(TRACE_INFO,
                                        str(format("Cannot create a Temporal (uniform-distribution) with lower bound (%1%) greater than upper bound (%2%)") % b % a).c_str());
        }
    }
    this->normal = normal;
    this->a = a;
    this->b = b;
}

Temporal Temporal::UndefinedTemporalFactory()
{
    Temporal t(0);
    t.a = UNDEFINED_A;
    t.b = UNDEFINED_B;
    t.normal = UNDEFINED_NORMAL;
    return t;
};


const Temporal& Temporal::undefined_temporal()
{
    static const Temporal instance = UndefinedTemporalFactory();
    return instance;
}

Temporal::Temporal(octime_t a, octime_t b, bool normal)
{
    init(a, b, normal);
}

Temporal::Temporal(octime_t timestamp)
{
    init(timestamp, timestamp, false);
}

Temporal::~Temporal()
{
}

bool Temporal::isNormal() const
{
    return normal;
}

octime_t Temporal::getA() const
{
    return a;
}

octime_t Temporal::getB() const
{
    return b;
}

octime_t Temporal::getLowerBound() const
{
    if (normal) {
        return (a - b);
    } else {
        return a;
    }
}

octime_t Temporal::getUpperBound() const
{
    if (normal) {
        return (a + b);
    } else {
        return b;
    }
}

std::string Temporal::toString() const
{
    if (*this == UNDEFINED_TEMPORAL) return "UNDEFINED_TEMPORAL";
    return str(format("(%1%,%2%,%3%)") % (normal ? "NORMAL" : "UNIFORM") % a % b);
}

std::string Temporal::getTimeNodeName() const
{
    std::string name;
    if (normal) {
        name = str(format("%1%:%2%:%3%") % a % b % normal);
    } else if (a == b) {
        name = str(format("%1%") % a);
    } else {
        name = str(format("%1%:%2%") % a % b);
    }
    return name;
}

std::string Temporal::getTimeNodeName(octime_t timestamp)
{
    // NOTE: The strictly correct way to implement this would be like follows:
    //   return Temporal(timestamp).getTimeNodeName();
    // However, for performance reasons, this is implemented as bellow:

    return str(format("%1%") % timestamp);
}

Temporal Temporal::getFromTimeNodeName(const char* timeNodeName)
{
    const char* nextToken = timeNodeName;
    octime_t a = (octime_t) strtoull(nextToken,NULL,10);

    DPRINTF("Temporal::getFromTimeNodeName: %llu %llu %llu / %s\n", a, a, (octime_t)atof(timeNodeName), timeNodeName);

    while (*nextToken && *nextToken != ':') {
        nextToken++;
    }
    if (!(*nextToken)) {
        return Temporal(a);
    }

    octime_t b = (octime_t)strtoull(++nextToken,NULL,10);

    DPRINTF("Temporal::getFromTimeNodeName: %llu %llu / %s\n", b, b, nextToken);

    while (*nextToken && *nextToken != ':') {
        nextToken++;
    }
    if (!(*nextToken)) {
        return Temporal(a, b);
    }
    bool normal = atoi(++nextToken) != 0; // must be true, actually.
    return Temporal(a, b, normal);
}

int Temporal::compareTo(const Temporal* other) const
{
    octime_t low1 = this->getLowerBound();
    octime_t low2 = other->getLowerBound();
    if(low1 < low2)
        return -1;
    else if(low2 < low1)
        return 1;
    octime_t up1 = this->getUpperBound();
    octime_t up2 = other->getUpperBound();
    if(up1 < up2)
        return -1;
    else if(up2 < up1)
        return 1;
    // if lower and upper bounds are equal, make distinction if
    // they have different distribution (nomal < uniform)
    int result;
    if (this->isNormal()) {
        result = (other->isNormal()) ? 0 : -1;
    } else {
        result = (other->isNormal()) ? 1 : 0;
    }
    return result;
}

bool Temporal::operator<(const Temporal& rhs) const
{
    return (compareTo(&rhs) < 0);
}

bool Temporal::operator>(const Temporal& rhs) const
{
    return (compareTo(&rhs) > 0);
}

bool Temporal::operator<=(const Temporal& rhs) const
{
    return !(operator>(rhs));
}

bool Temporal::operator>=(const Temporal& rhs) const
{
    return !(operator<(rhs));
}

bool Temporal::operator==(const Temporal& rhs) const
{
    return (normal == rhs.isNormal() &&
            a == rhs.getA() &&
            b == rhs.getB());
}

bool Temporal::operator!=(const Temporal& rhs) const
{
    return !(operator==(rhs));
}

Temporal* Temporal::clone() const
{
    return new Temporal(a, b, normal);
}
