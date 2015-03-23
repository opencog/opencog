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

#include <opencog/util/platform.h>

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

std::ostream& operator<<(std::ostream& out, const Temporal& t)
{
    return (out << t.toString());
}

// Values of A, B and NORMAL for the UNDEFINED_TEMPORAL Temporal instance
// These values should not compose a valid Temporal, btw.
// In other words, if someone pass these values for a Temporal constructor it will throw an exception.
#define UNDEFINED_A UINT_MAX
#define UNDEFINED_B UINT_MAX
#define UNDEFINED_NORMAL true

// USED TO SEEK MEMORY LEAK
//int Temporal::numTemporals = 0;

void Temporal::init(unsigned long a, unsigned long b, bool normal) throw (InvalidParamException)
{

    // USED TO SEEK MEMORY LEAK
    //numTemporals++;
    //cout << "Total temporals: " << numTemporals << endl;

    // sanity checks
    if (normal) {
        if (b > a) {
            throw InvalidParamException(TRACE_INFO,
                                        "Cannot create a Temporal (normal-distribution) with the variance (%lu) greater than the mean (%lu). This causes negative lower bound.", b,  a);
        }
        unsigned long long sum = (unsigned long long) a + b;
        if (sum > (unsigned long long) UINT_MAX) {
            throw InvalidParamException(TRACE_INFO,
                                        "Temporal - Upper bound reached when creating a Temporal (normal-distribution): %lu.", sum);
        }
    } else {
        if (a > b) {
            throw InvalidParamException(TRACE_INFO,
                                        "Cannot create a Temporal (uniform-distribution) with lower bound (%lu) greater than upper bound (%lu)", b, a);
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

Temporal::Temporal(unsigned long a, unsigned long b, bool normal)
{
    init(a, b, normal);
}

Temporal::Temporal(unsigned long timestamp)
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

unsigned long Temporal::getA() const
{
    return a;
}

unsigned long Temporal::getB() const
{
    return b;
}

unsigned long Temporal::getLowerBound() const
{
    if (normal) {
        return (a - b);
    } else {
        return a;
    }
}

unsigned long Temporal::getUpperBound() const
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
    char buf[1 << 8];
    char* answer = buf;
    sprintf(answer, "(%s,%lu,%lu)", (normal ? "NORMAL" : "UNIFORM"), a, b);
    return answer;
}

std::string Temporal::getTimeNodeName() const
{
    char buffer[1000];
    if (normal) {
        sprintf(buffer, "%lu:%lu:%d", a, b, normal);
    } else if (a == b) {
        sprintf(buffer, "%lu", a);
    } else {
        sprintf(buffer, "%lu:%lu", a, b);
    }
    return buffer;
}

std::string Temporal::getTimeNodeName(unsigned long timestamp)
{
    // NOTE: The strictly correct way to implement this would be like follows:
    //   return Temporal(timestamp).getTimeNodeName();
    // However, for performance reasons, this is implemented as bellow:
    char buffer[1000];
    sprintf(buffer, "%lu", timestamp);
    return buffer;
}

Temporal Temporal::getFromTimeNodeName(const char* timeNodeName)
{
    const char* nextToken = timeNodeName;
    unsigned long a = (unsigned long) strtoul(nextToken,NULL,10);

    DPRINTF("Temporal::getFromTimeNodeName: %ld %lu %lu / %s\n", a, a, (unsigned long)atof(timeNodeName), timeNodeName);

    while (*nextToken && *nextToken != ':') {
        nextToken++;
    }
    if (!(*nextToken)) {
        return Temporal(a);
    }
    unsigned long b = (unsigned long)strtoul(++nextToken,NULL,10);

    DPRINTF("Temporal::getFromTimeNodeName: %ld %lu / %s\n", b, b, nextToken);

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
    unsigned long low1 = this->getLowerBound();
    unsigned long low2 = other->getLowerBound();
    if(low1 < low2)
        return -1;
    else if(low2 < low1)
        return 1;
    unsigned long up1 = this->getUpperBound();
    unsigned long up2 = other->getUpperBound();
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
