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

//typedef unsigned int uint;
//typedef unsigned long ulong;
typedef tree<Vertex> vtree;

// Specific toString method for float numbers (force 3 digits after decimal point)
std::string toString(double data);

/**
 * Returns a string from the given argument by using the << operator
 */
//Nil: I comment that template because the exact same one is in StringManipulator
//template <typename T>
//std::string toString(T data)
//{
//    std::ostringstream oss;
//    oss << data;
//    return oss.str();
//}

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

enum padAlignment { CENTER, LEFT, RIGHT };
std::string padstr(const char*, unsigned int, padAlignment) throw (InvalidParamException);

/**
 * This method reads a line from a string and returns
 * the rest of the string.
 *
 * @param Where the next line should be read from.
 * @param A reference to a string where the read next line will be.
 * @return The rest of the string after reading the line.
 */
const char *nextLine(const char *, std::string&);

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

/** STL-Listifies a HandleEntry */

template<typename T, typename OutT>
void h_to_list(OutT outIt, T inEntry)
{
    while (inEntry) {
        *(outIt++) = inEntry->handle;
        inEntry = inEntry->next;
    }
}

template<typename T>
bool empty(const T& c)
{
    return c.empty();
}

template<typename LinkT, typename ArgT1, typename ArgT2>
LinkT make_tuple(const ArgT1& arg1, const ArgT2& arg2)
{
    return LinkT(arg1, arg2);
}

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

template<typename InputT, typename evalT, typename ValT>
InputT GetBest(InputT start, InputT end, evalT op, ValT minVal)
{
    if (start == end)
        return end;

    ValT bestV = minVal, tempVal;

    InputT ret = start;
    while (++start != end)
        if ( (tempVal = op(*start)) > bestV) {
            bestV = tempVal;
            ret = start;
        }

    return ret;
}

/** Load the contents of a textfile \param fname to \param dest. */
bool LoadTextFile(const std::string fname, std::string& dest);

/** Checks whether a file exists */
bool exists(const char *fname);

/** Checks whether the strings are equal, when case is ignored. */
bool nocase_equal(const char *s1, const char *s2);

/** Converts a character to upper case (also some Scandinavian characters) */
char Isox(char s);

// Now in types.h
/*template<typename T>
struct TypeWrapper {
    T value;
    explicit TypeWrapper(T _val) : value(_val) {}
    T operator=(const TypeWrapper& rhs) {
        return (value = rhs.value);
    }
    bool operator==(const TypeWrapper& rhs) const {
        return value == rhs.value;
    }
    bool operator<(const TypeWrapper& rhs) const {
        return value < rhs.value;
    }
};*/

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

#define mva MakeVirtualAtom_slow

/// Handles are actually mostly types, here. The Handle/Type ambiguity
/// will be resolved soon enough. (says Ari, March 20, 2006)

tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4, const tree<Vertex>& t5);
tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4);
tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3);
tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1, const tree<Vertex>& t2);
tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1);
tree<Vertex> MakeVirtualAtom_slow(Vertex v);


/// Convert a real atom into vtree in which only NODEs are left as real atoms
/// while all links become virtual, ie. tree branches

struct less_tree_vertex : public std::binary_function<tree<Vertex>, tree<Vertex>, bool> {
    bool operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs) const;
    bool operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs,
                    tree<Vertex>::iterator ltop,
                    tree<Vertex>::iterator rtop) const;
};

/** Checks whether the character is printable */
bool visible(char c);

/** Converts an int to an STL string */
std::string i2str(int v);

/** Converts a string to upper case by return value */
std::string toupper(std::string k);

/** A function to create XML elements.
 \param elem The element's name (eg. "xml" in "<xml> text <xml>")
 \param pcdata The element data (eg. "text" in "<xml> text <xml>")
 \return The XML element.
*/
std::string XMLembed(const std::string &elem, const std::string &pcdata);

/** Tokenize a string and produce a std::vector list of items */
class StringTokenizer : public std::vector<std::string>
{
public:
    StringTokenizer(const std::string &rStr, const std::string &rDelimiters = " ,\n");
    std::vector<std::string> WithoutEmpty() const;
};

/** nocase_string: STL string class which ignores the case. */
struct nocase_string : public std::string {
public:
    nocase_string(const char* str);
    nocase_string(const nocase_string& str);
    nocase_string(const std::string& str);
    nocase_string();

    bool operator <(const char* rhs);
    bool operator <(const nocase_string& rhs);
    bool operator <(const std::string& rhs);
    bool operator ==(const char* rhs);
    bool operator ==(const nocase_string& rhs);
    bool operator ==(const std::string& rhs);
    bool operator !=(const char* rhs);
    bool operator !=(const nocase_string& rhs);
    bool operator !=(const std::string& rhs);
    void operator +=(nocase_string s);
    nocase_string operator+(nocase_string s);
};

#define _Int(s) atoi((s).c_str())

/** @class Listener
 \brief The abstract listener interface.
*/

struct Listener {
    virtual ~Listener() {}
    virtual void OnUpdate(const void*) = 0;
};

struct less_vtree : public std::binary_function<tree<Vertex>, tree<Vertex>, bool> {
    bool operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs) const;
};

} // namespace opencog

#endif /* OPENCOG_UTILS_H */
