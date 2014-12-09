/** iostream_combo.h --- 
 *
 * Copyright (C) 2012 OpenCog Foundation
 *
 * Authors: Nil Geisweiller <ngeiswei@gmail.com>
 *          Matt Chapman <Matt@NinjitsuWeb.com>
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


#ifndef _IOSTREAM_COMBO_H
#define _IOSTREAM_COMBO_H

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include "vertex.h"

/**
 * Set of functions to read and write combo programs in various
 * format. Current formats supported are combo (default) and python.
 */

namespace opencog { namespace combo {

enum class output_format {
    combo,
    python,
    scheme,
    output_format_count                // to get the number of formats
};

// like operator<< but can choose the output format
std::ostream& ostream_builtin(std::ostream&, const builtin&,
                              output_format fmt = output_format::combo);
std::ostream& ostream_argument(std::ostream&, const argument&,
                               const std::vector<std::string>& labels =
                               std::vector<std::string>(),
                               output_format fmt = output_format::combo);
std::ostream& ostream_vertex(std::ostream&, const vertex&,
                             const std::vector<std::string>& labels =
                             std::vector<std::string>(),
                             output_format fmt = output_format::combo);
std::ostream& ostream_combo_tree(std::ostream&, const combo_tree&,
                                 const std::vector<std::string>& labels =
                                 std::vector<std::string>(),
                                 output_format fmt = output_format::combo);
template<typename Iter>
std::ostream& ostream_combo_it(std::ostream& out, Iter it,
                               const std::vector<std::string>& labels =
                               std::vector<std::string>(),
                               output_format fmt = output_format::combo) {
    switch(fmt) {
    case(output_format::combo):
        ostream_vertex(out, *it, labels, fmt);
        if (it.number_of_children() > 0) {
            out << "(";
            auto sib = it.begin();
            ostream_combo_it(out, sib++, labels, fmt);
            for (; sib != it.end(); ++sib)
                ostream_combo_it(out << " ", sib, labels, fmt);
            out << ")";
        }
        return out;
    case(output_format::python): {
        bool is_infix = *it == id::logical_and or *it == id::logical_or;

        std::stringstream seperator;
        if (is_infix) {
            ostream_vertex(seperator << " ", *it, labels, fmt) << " ";
        } else {
            ostream_vertex(out, *it, labels, fmt);
            seperator << ", ";
        }

        if (it.number_of_children() > 0) {
            out << "(";
            auto sib = it.begin();
            ostream_combo_it(out, sib++, labels, fmt);
            for (; sib != it.end(); ++sib)
                ostream_combo_it(out << seperator.str(), sib, labels, fmt);
            out << ")";
        }
        return out;
    }
    case(output_format::scheme):
        out << "(";
        ostream_vertex(out, *it, labels, fmt);
        for (auto sib = it.begin(); sib != it.end(); ++sib)
            ostream_combo_it(out << " ", sib, labels, fmt);
        out << ")";
        return out;
    default:
        OC_ASSERT(false, "Unsupported case");
        return out;
    }
}

// return false if the string has no match
bool builtin_str_to_vertex(const std::string& str, vertex& v);
bool ann_str_to_vertex(const std::string& str, vertex& v);
bool argument_str_to_vertex(const std::string& str, vertex& v);
bool contin_str_to_vertex(const std::string& str, vertex& v);
bool message_str_to_vertex(const std::string& str, vertex& v);
bool enum_str_to_vertex(const std::string& str, vertex& v);

template<class BUILTIN_ACTION, class PERCEPTION,
         class ACTION_SYMBOL, class INDEFINITE_OBJECT>
void str_to_vertex(const std::string& str, vertex& v)
{
    OC_ASSERT(!str.empty(), "input to string should not be empty.");
    // builtin, ann, argument, constant and message
    // the order may matter
    if (builtin_str_to_vertex(str, v)
       || argument_str_to_vertex(str, v)
       || contin_str_to_vertex(str, v)
       || enum_str_to_vertex(str, v)
       || ann_str_to_vertex(str, v)
       || message_str_to_vertex(str, v)) {
        return;
    }
    // builtin_action
    else if (builtin_action ba = BUILTIN_ACTION::get_instance(str)) {
        v = ba;
    }
    // perception
    else if (perception p = PERCEPTION::get_instance(str)) {
        v = p;
    }
    // action symbol
    else if (action_symbol as = ACTION_SYMBOL::get_instance(str)) {
        v = as;
    }
    // indefinite_object
    else if (indefinite_object i = INDEFINITE_OBJECT::get_instance(str)) {
        v = i;
    }
    // should be definite object then
    else {
        // Any word character (alphanumeric characters plus the 
        // underscore). If you find this too constraining, feel free to
        // relax.
        static const boost::regex e("[\\w-]+");
        OC_ASSERT(boost::regex_match(str, e),
                  "Lexical error: '%s' does not name a definite_object", str.c_str());
        v = str;
    }
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
vertex str_to_vertex(const std::string& str)
{
    vertex v;
    str_to_vertex<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(str, v);
    return v;
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
std::istream& stream_to_vertex(std::istream& in, vertex& v)
{
    std::string str;
    //use getline instead of in >> str to be sure to not
    //skip spaces in the message
    std::getline(in, str);
    str_to_vertex<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(str, v);
    return in;
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
void sub_strtree_to_combo_tree(const tree<std::string>& src,
                               tree<std::string>::iterator src_it,
                               combo_tree& dst, combo_tree::iterator dst_it)
{
    dst_it = dst.replace(dst_it, str_to_vertex<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(*src_it));
    dst.erase_children(dst_it);
    for (tree<std::string>::sibling_iterator sib = src_it.begin();
            sib != src_it.end(); ++sib)
        sub_strtree_to_combo_tree<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(src, tree<std::string>::iterator(sib), dst, dst.append_child(dst_it));
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
void strtree_to_combo_tree(const tree<std::string>& src, combo_tree& dst)
{
    dst = combo_tree(vertex());
    tree<std::string>::iterator src_it = src.begin();
    combo_tree::iterator dst_it = dst.begin();
    while (src_it != src.end()) {
        dst_it = dst.insert_after(dst_it, vertex());
        sub_strtree_to_combo_tree<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(src, src_it, dst, dst_it);
        src_it.skip_children();
        ++src_it;
    }
    dst.erase(dst.begin());
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
std::istream& stream_to_combo_tree(std::istream& in, combo_tree& tr)
{
    tree<std::string> tmp;
    in >> tmp;
    strtree_to_combo_tree<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(tmp, tr);
    return  in;
}

/**
 * ph2l where ph == "place holder" and l == "label"
 *
 * This method replaces place holders by labels in a string
 * containing a combo expression.
 *
 * For instance, "and($1 $2)" would be replaced by "and($fat $pretty)"
 * given the vector of labels {"fat", "pretty"}
 */
std::string ph2l(const std::string& ce,
                 const std::vector<std::string>& labels);

/**
 * l2ph where ph == "place holder" and l == "label"
 *
 * This is the converse of ph2l. Given, for instance "and($fat
 * $pretty)" and the vector of labels {"fat", "pretty"}, it returns
 * "and($1 $2)". If a variable is not in labels (that is it doesn't
 * correspond to a place holder) an OC_ASSERT is raised.
 */
std::string l2ph(const std::string& ce,
                 const std::vector<std::string>& labels);

/**
 * return the list of variables in a combo tree. Note that if some
 * variables are found several times they will appear several times as
 * well. So for instance,
 *
 * parse_combo_variables("and($small $fat $small)")
 *
 * returns
 *
 * {"small", "fat", "small"}
 */
std::vector<std::string> parse_combo_variables(const std::string& ce);

std::ostream& operator<<(std::ostream&, const ann_type&);
std::ostream& operator<<(std::ostream&, const builtin&);
std::ostream& operator<<(std::ostream&, const wild_card&);
std::ostream& operator<<(std::ostream&, const argument&);
// output argument $n when positive, !$n when negative 
std::ostream& ostream_abbreviate_literal(std::ostream&, const argument&,
                                         const std::vector<std::string>& labels =
                                         std::vector<std::string>());
std::ostream& operator<<(std::ostream&, const vertex&);

}} // ~ namespace opencog::combo

#endif // _IOSTREAM_COMBO_H
