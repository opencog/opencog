/*
 * opencog/spacetime/Temporal.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
 *            Carlos Lopes <dlopes@vettalabs.com>
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

#ifndef _OPENCOG_TEMPORAL_H
#define _OPENCOG_TEMPORAL_H

#include <string>
#include <limits.h>
#include <boost/functional/hash.hpp>

#include <opencog/util/platform.h>
#include <opencog/util/exceptions.h>

using namespace std;

typedef uint64_t octime_t;

#define OCTIME_MAX numeric_limits<octime_t>::max()

/** \addtogroup grp_spacetime
 *  @{
 */

namespace opencog
{

class Temporal
{

private:

    /**
     * Initializes the attributes of this Temporal object. Used by constructors only.
     */
    void init(octime_t, octime_t, bool);

    /**
     * Creates a unique instance for UNDEFINED_TEMPORAL object
     */
    static Temporal UndefinedTemporalFactory();

public:
    // USED TO SEEK MEMORY LEAK
    //static int numTemporals;

    /**
     * Returns a temporal object that works as flag argument for lookup in TemporalTable
     */
    static const Temporal& undefined_temporal();

    /**
     * Constructs a Temporal object with the given parameters:
     * @param a         If using normal distribution, the mean value. Otherwise, the lower bound value.
     * @param b         If using normal distribution, the variance value (mean +/- variance). Otherwise, the upper bound value.
     * @param normal    True for normal distribution. False for uniform distribution (default).
     * @exception This constructor throws an RuntimeException if:
     *   - a uniform distribution is used (normal argument is ommited or false) and "a" is greater than "b".
     *   - a normal distribution is used (normal argument is true) and "b" is greater than "a" (this causes negative value for the lower bound).
     */
    Temporal(octime_t, octime_t, bool = false);

    /**
     * Special constructor for timestamps only. Using this constructor passing a timestamp argument T is
     * equivalent to call the more comprehensive constructor passing as follows: Temporal(T,T,false).
     * @param timestamp The value of both lower and upper bounds, since this is a just time point.
     */
    Temporal(octime_t);

    /**
     * Destructor
     */
    virtual ~Temporal();

    /**
     * @return True if this Temporal object uses the normal distribution. False, otherwise, i.e., uses uniform distribution.
     */
    bool isNormal() const;

    /**
     * @return The A attribute of this Temporal: If using normal distribution, the mean value. Otherwise, the lower bound value.
     */
    octime_t getA() const;

    /**
     * @return The B attribute of this Temporal: If using normal distribution, the variance value (mean +/- variance). Otherwise, the upper bound value.
     */
    octime_t getB() const;

    /**
     * @return the Lower bound of this Temporal object
     */
    octime_t getLowerBound() const;

    /**
     * @return the upper bound of this Temporal object
     */
    octime_t getUpperBound() const;


    /**
     * @return a String representation of this Temporal Object, of the form (<Distribution>,<A value>,<B value>)
     * Or, in other words:
     *   - If using normal distribution, it's of the form:  (NORMAL,<mean value>,<variance value>)
     *   - If using uniform distribution, it's of the form: (UNIFORM,<lower bound value>,<upper bound value>)
     */
    virtual std::string toString() const;

    /**
     * Gets the name of the TimeNode that represents this Temporal object
     */
    std::string getTimeNodeName() const;

    /**
     * Gets the name of the TimeNode that represents the given timestamp value
     */
    static std::string getTimeNodeName(octime_t);

    /**
     * Gets the Temporal object that corresponds to the TimeNode with the given name.
     */
    static Temporal getFromTimeNodeName(const char* timeNodeName);

    /**
     * Compare 2 Temporal objects.
     * @param otherTemporal the given Temporal object to be compared with this object
     * @return 0, if this object is equals to the Temporal object
     *    1, if this object is considered greater than the given Temporal object
     *    -1, if this object is considerer smaller than the given Temporal object
     * NOTES:
     *  1) a Temporal object T1 is considered smaller than a Temporal object T2 when:
     *   - T1's lower bound is smaller than T2's lower bound; or
     *   - T1's lower bound is equals to T2's lower bound and T1's upper bound is smaller than T2's upper bound; or
     *   - T1's bounds are equals to T2's bounds and T1 is of normal-distribution type while T2 is of uniform-distribution type.
     *  2) a Temporal object T1 is considered equals to a Temporal object T2 only when all their attributes (i.e., a, b and normal) are equals.
     */
    virtual int compareTo(const Temporal* object) const;

    /**
     * Operators for making comparison between Temporal objects easier.
     */
    virtual bool operator<(const Temporal& rhs) const;
    virtual bool operator>(const Temporal& rhs) const;
    virtual bool operator<=(const Temporal& rhs) const;
    virtual bool operator>=(const Temporal& rhs) const;
    virtual bool operator==(const Temporal& rhs) const;
    virtual bool operator!=(const Temporal& rhs) const;

    /**
     * Clones a Temporal object, using heap allocation.
     */
    virtual Temporal* clone() const;

private:
    bool normal;
    octime_t a;
    octime_t b;

};

struct hashTemporal {
    int operator()(Temporal* tl) const {
        int hashCode =  boost::hash<octime_t>()(tl->getA());
        return(hashCode);
    }
};

struct equalTemporal {
    bool operator()(Temporal* t1, Temporal* t2) const {
        return !t1->compareTo(t2);
    }
};

#define UNDEFINED_TEMPORAL Temporal::undefined_temporal()

} // namespace opencog

std::ostream& operator<<(std::ostream& out, const opencog::Temporal& t);

/** @}*/
#endif // _OPENCOG_TEMPORAL_H
