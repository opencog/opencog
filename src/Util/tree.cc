/*
 * src/Util/tree.cc
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

#include "tree.h"
#include <boost/spirit/core.hpp>

namespace
{
using namespace boost::spirit;
using std::string;
using namespace Util;

tree<string> tr;
tree<string>::iterator at = tr.begin();

void begin_internal(const char* from, const char* to)
{
    at = tr.empty()
         ? tr.insert(at, string(from, to - 1))
         : tr.append_child(at, string(from, to - 1));
}
void end_internal(const char)
{
    at = tr.parent(at);
}
void add_leaf(const char* from, const char* to)
{
    if (tr.empty())
        at = tr.insert(at, string(from, to));
    else
        tr.append_child(at, string(from, to));
}

struct TreeGrammar : public grammar<TreeGrammar> {
    std::vector<string> v;

    template<typename ScannerT>
    struct definition {
        definition(const TreeGrammar&) {
            term =
                lexeme_d[(+( anychar_p - ch_p('(') - ch_p(')') - space_p))]
                [&add_leaf];
            beg =
                lexeme_d[(+( anychar_p - ch_p('(') - ch_p(')') - space_p)) >> '('];
            expr =
                (beg[&begin_internal] >> + expr >> ch_p(')')[&end_internal]) |
                term;
            //expr=term | (term >> '(' >> +expr >> ')');
        }
        rule<ScannerT> expr, beg, term;

        const rule<ScannerT>& start() const {
            return expr;
        }
    };
};

tree<std::string> parse_string_tree(const std::string& str)
{
    TreeGrammar tg;
    tr.clear();
    at = tr.begin();
    parse(str.c_str(), tg, space_p);

    tree<std::string> tmp(tr);
    tr.clear();
    return tmp;
}
} //~namespace

std::istream& operator>>(std::istream& in, Util::tree<std::string>& t)
{
    t.clear();
    std::string str, tmp;
    int nparen = 0;
    do {
        in >> tmp;
        nparen += count(tmp.begin(), tmp.end(), '(') - count(tmp.begin(), tmp.end(), ')');
        str += tmp + ' ';
    } while (in.good() && nparen > 0);
    if (nparen != 0) {
        std::cerr << "Paren mismatch parsing tree: '" << str << "'" << std::endl;
        exit(1);
    }
    t = parse_string_tree(str);
    return in;
}
