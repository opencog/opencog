/*
 * src/AtomSpace/utils.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef OPENCOG_UTILS_H
#define OPENCOG_UTILS_H

#include <list>
#include <string>
#include <sstream>
#include <vector>

#include <boost/variant.hpp>
#include <boost/foreach.hpp>

#include <opencog/util/tree.h>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Temporal.h>

#include <opencog/util/platform.h>
#include <opencog/util/exceptions.h>

/// Note! This does not re-define std::for_each!
#define foreach BOOST_FOREACH

/// Convert a boost::variant into a Handle
#define v2h(v) boost::get<Handle>(v)

namespace opencog
{

// Specific toString method for float numbers (force 3 digits after decimal point)
std::string toString(double data);

#if 0
/**
 * Returns a copy of the given string surrounded by ANSI bold tags.
 *
 * @return Copy of the given string surrounded by ANSI bold tags.
 */
std::string bold(const char*);

/**
 * Returns the string representation of an integer surrounded by ANSI bold
 * tags.
 *
 * @return The string representation of an integer surrounded by ANSI bold
 * tags.
 */
std::string bold(int i);
#endif

template <class _Key> struct hash2int { };

inline size_t __hash2int(const char* __s)
{
    unsigned long __h = 0;
    size_t size = 2 * sizeof(int);
    for (size_t i = 0; i < size; i++, __s++) {
        __h = 5 * __h + *__s;
    }
    return size_t(__h);
}

template <> struct hash2int<char *> {
    size_t operator()(const char* __s) const {
        return __hash2int(__s);
    }
};


/**
 * Initializes the reference time that will be used for getting current elapsed times
 */
void initReferenceTime();
/**
 * Gets the elapsed time (in milliseconds) since the reference time initialized with
 * initReferenceTime() function. The initReferenceTime() function must be called before
 * this function be called by the first time.
 */
unsigned long getElapsedMillis();

/** STL-Listifies a TemporalEntry */
template<typename T, typename OutT>
void to_list(OutT outIt, T inEntry)
{
    while (inEntry) {
        *(outIt++) = inEntry->handleTemporalPair; //handle;
        inEntry = inEntry->next;
    }
}


template<typename T>
bool empty(const T& c)
{
    return c.empty();
}

/** Used very rarely, one or twice, in learning behaviour and PLN backinference */
template < typename ForwardIter,
typename OutputIter,
typename UnaryPred >
OutputIter copy_if(ForwardIter begin, ForwardIter end, OutputIter dest, UnaryPred f)
{
    while (begin != end) {
        if (f(*begin))
            *dest++ = *begin;
        ++begin;
    }
    return dest;
}

template<typename T>
std::ostream& operator<<(std::ostream& out, const TypeWrapper<T>&);

template<typename T>
std::ostream& operator<<(std::ostream& out, const TypeWrapper<T>& t)
{
    return (out << t.value);
}

typedef TypeWrapper<Temporal> TimeStampWrapper;
typedef TypeWrapper<int> IntegerWrapper;
typedef TypeWrapper<float> FloatWrapper;
typedef TypeWrapper<bool> BoolWrapper;
typedef TypeWrapper<unsigned char> ByteWrapper;
typedef TypeWrapper<signed char> CharWrapper;
typedef TypeWrapper<short int> ShortIntegerWrapper;
//typedef TypeWrapper<ShortFloat> ShortFloatWrapper;

/** Checks whether the character is printable */
bool visible(char c);

/** Converts an int to an STL string */
std::string i2str(int v);

/** Tokenize a string and produce a std::vector list of items */
class StringTokenizer : public std::vector<std::string>
{
public:
    StringTokenizer(const std::string &rStr, const std::string &rDelimiters = " ,\n");
    std::vector<std::string> WithoutEmpty() const;
};

#define _Int(s) atoi((s).c_str())

/** @class Listener
 \brief The abstract listener interface.
*/

struct Listener {
    virtual ~Listener() {}
    virtual void OnUpdate(const void*) = 0;
};

} // namespace opencog

#endif /* OPENCOG_UTILS_H */
