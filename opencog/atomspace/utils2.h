/*
 * src/AtomSpace/utils2.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#ifndef UTILS_2_H
#define UTILS_2_H

#include <boost/variant.hpp>
#include <boost/foreach.hpp>

#include "tree.h"
#include "types.h"
#include "Temporal.h"

typedef unsigned int uint;
typedef unsigned long ulong;

/// Note! This does not re-define std::for_each!
#define foreach BOOST_FOREACH

/// Convert a boost::variant into a Handle
#define v2h(v) boost::get<Handle>(v)

namespace opencog
{

template<typename T>
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
};

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

typedef boost::variant <Handle,
                        TimeStampWrapper,
                        IntegerWrapper,
                        FloatWrapper,
                        BoolWrapper,
                        ByteWrapper,
                        CharWrapper,
                        ShortIntegerWrapper> Vertex;
//ShortFloatWrapper> Vertex; since ShortFloat is
//typedef'ed to float, this is a mistake

#define mva MakeVirtualAtom_slow

/// Handles are actually mostly types, here. The Handle/Type ambiguity
/// will be resolved soon enough. (says Ari, March 20, 2006)

#ifdef WIN32
tree<Vertex> MakeVirtualAtom_slow(Type T, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4, const tree<Vertex>& t5);
tree<Vertex> MakeVirtualAtom_slow(Type T, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4);
tree<Vertex> MakeVirtualAtom_slow(Type T, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3);
tree<Vertex> MakeVirtualAtom_slow(Type T, const tree<Vertex>& t1, const tree<Vertex>& t2);
tree<Vertex> MakeVirtualAtom_slow(Type T, const tree<Vertex>& t1);
tree<Vertex> MakeVirtualAtom_slow(Type T);
//tree<Vertex> MakeVirtualAtom_slow(Type T, std::string name)

#endif
//#else
tree<Vertex> MakeVirtualAtom_slow(Handle T, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4, const tree<Vertex>& t5);
tree<Vertex> MakeVirtualAtom_slow(Handle T, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4);
tree<Vertex> MakeVirtualAtom_slow(Handle T, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3);
tree<Vertex> MakeVirtualAtom_slow(Handle T, const tree<Vertex>& t1, const tree<Vertex>& t2);
tree<Vertex> MakeVirtualAtom_slow(Handle T, const tree<Vertex>& t1);
tree<Vertex> MakeVirtualAtom_slow(Handle T);
//tree<Vertex> MakeVirtualAtom_slow(Handle T, std::string name);
//#endif

tree<Vertex> MakeVirtualAtom_slow(Vertex T, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4, const tree<Vertex>& t5);
tree<Vertex> MakeVirtualAtom_slow(Vertex T, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4);
tree<Vertex> MakeVirtualAtom_slow(Vertex T, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3);
tree<Vertex> MakeVirtualAtom_slow(Vertex T, const tree<Vertex>& t1, const tree<Vertex>& t2);
tree<Vertex> MakeVirtualAtom_slow(Vertex T, const tree<Vertex>& t1);
tree<Vertex> MakeVirtualAtom_slow(Vertex T);


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

#endif
