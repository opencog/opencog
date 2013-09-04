/*
 * opencog/learning/moses/eda/scoring.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _EDA_SCORING_H
#define _EDA_SCORING_H

#include "eda.h"
#include <opencog/util/functional.h>

namespace opencog { 
namespace moses {

template<typename ScoreT>
struct scored_instance : public tagged_item<instance, ScoreT>
{
    typedef tagged_item<instance, ScoreT> super;

    scored_instance(const instance& i, const ScoreT& s) : super(i, s) { }
    scored_instance(const instance& i) : super(i) { }
    scored_instance() { }
    template<class T1, class T2>
    scored_instance(const std::pair<T1, T2>& p) : super(p) { }

    bool operator<(const scored_instance& other) const
    {
        return this->second < other.second;
    }
    bool operator>(const scored_instance& other) const
    {
        return this->second > other.second;
    }
};

} // ~namespace moses
} // ~namespace opencog

#endif
