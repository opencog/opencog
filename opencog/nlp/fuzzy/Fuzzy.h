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

#include <opencog/atomutils/FuzzyMatchBasic.h>

namespace opencog
{
namespace nlp
{

class Fuzzy :
    public FuzzyMatchBasic
{
    public:
        Fuzzy(Type, const HandleSeq&, bool);
        virtual ~Fuzzy();

    protected:
        virtual void start_search(const Handle&);
        virtual bool accept_starter(const Handle&);
        virtual bool try_match(const Handle&);
        virtual RankedHandleSeq finished_search(void);

    private:
        // The type of atom that we want
        Type rtn_type;

        // The atoms that we don't want in the solutions
        HandleSeq excl_list;

        // A flag to decide whether or not to accept duplicate solutions
        bool dup_check;

        // The solutions
        RankedHandleSeq solns;

        // A vector for storing the "contents" of the accepted solutions
        // mainly to avoid returning duplicate solutions
        HandleSeqSeq solns_contents;

        std::set<Handle> solns_seen;
};

}
}

#endif  // FUZZY_H

