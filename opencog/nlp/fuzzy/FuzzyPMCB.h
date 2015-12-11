/*
 * FuzzyPMCB.h
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

#ifndef FUZZYPMCB_H
#define FUZZYPMCB_H

#include <opencog/query/FuzzyPatternMatch.h>

namespace opencog
{
namespace nlp
{

class FuzzyPMCB :
    public FuzzyPatternMatch
{
    public:
        FuzzyPMCB(AtomSpace*, Type, const HandleSeq&);
        ~FuzzyPMCB();

        virtual void similarity_match(const Handle&, const Handle&, HandleSeq&);
//    virtual bool link_match(const LinkPtr&, const LinkPtr&);

//    virtual void set_pattern(const Variables& vars,
//                             const Pattern& pat)
//    {
//       InitiateSearchCB::set_pattern(vars, pat);
//        DefaultPatternMatchCB::set_pattern(vars, pat);
//    }
};

}
}

#endif  // FUZZYPMCB_H
