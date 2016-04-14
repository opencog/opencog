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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomutils/FuzzyMatchBasic.h>

namespace opencog
{
namespace nlp
{

class Fuzzy :
    public FuzzyMatchBasic
{
    public:
        Fuzzy(AtomSpace*);
        Fuzzy(AtomSpace*, Type, const HandleSeq&);
        virtual ~Fuzzy();

        double fuzzy_compare(const Handle&, const Handle&);

    protected:
        virtual void start_search(const Handle&);
        virtual bool accept_starter(const Handle&);
        virtual bool try_match(const Handle&);
        virtual RankedHandleSeq finished_search(void);

    private:
        double NODE_WEIGHT = 0.5;
        double RARENESS_WEIGHT = 0.2;
        double LINGUISTIC_RELATION_WEIGHT = 0.3;

        AtomSpace* as;

        HandleSeq target_words;
        HandleSeq target_winsts;

        // The type of atom that we want
        Type rtn_type;

        // The atoms that we don't want in the solutions
        HandleSeq excl_list;

        // The solutions
        RankedHandleSeq solns;
        OrderedHandleSet solns_seen;

        std::map<UUID, double> tfidf_words;
        std::map<std::pair<UUID, UUID>, double> scores;

        void calculate_tfidf(const HandleSeq&);

        void compare(HandleSeq&, HandleSeq&, double, double&, bool);

        double get_score(const Handle&, const Handle&, bool);
};

}
}

#endif  // FUZZY_H

