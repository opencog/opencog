/*
 * Fuzzy.h
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * Author: Leung Man Hin <https://github.com/leungmanhin>
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

#ifndef FUZZY_H
#define FUZZY_H

#include <opencog/query/FuzzyPatternMatch.h>

namespace opencog
{
namespace nlp
{

class Fuzzy :
    public FuzzyPatternMatch
{
    public:
        Fuzzy(AtomSpace*, Type, const HandleSeq&);
        ~Fuzzy();

        virtual void set_pattern(const Variables&, const Pattern&);

        virtual bool accept_starter(const NodePtr);

        virtual void similarity_match(const Handle&, const Handle&);

        virtual HandleSeq get_solns();

    private:
       Handle pattern;

       HandleSeq pat_nodes;

       Type rtn_type;

       HandleSeq excl_list;

       std::map<Handle, std::pair<double, size_t>> solns;

       HandleSeqSeq dup_check;
};

}
}

#endif  // FUZZY_H
