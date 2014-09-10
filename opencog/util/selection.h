/*
 * opencog/util/selection.h
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

#ifndef _OPENCOG_SELECTION_H
#define _OPENCOG_SELECTION_H

#include "functional.h"
#include "numeric.h"
#include <iterator>
#include "dorepeat.h"
#include "RandGen.h"
#include "mt19937ar.h"

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

struct tournament_selection
{
    tournament_selection(unsigned int t_size_, RandGen& _rng = randGen())
    : t_size(t_size_), rng(_rng)
    {
        OC_ASSERT(t_size > 0);
    }

    unsigned int t_size;
    RandGen& rng;

    /**
     * Selects randomly n_select elements in [from, to) such that each
     * element is the winner of a random selection of t_size elements
     * in [from, to).  That is, a tournament is played n_select times.
     * The winner of each tournament is the largest element from a set
     * of t_size randomly choosen elements.
     *
     * The winners of each tournament are appended to dst.
     */
    template<typename In, typename Out>
    void operator()(In from, In to, Out dst, unsigned int n_select) const
    {
        typename std::iterator_traits<In>::difference_type d =
           distance(from, to);

        dorepeat (n_select) {
            In res = from + rng.randint(d);
            dorepeat (t_size - 1) {
                In tmp = from + rng.randint(d);
                if (*res < *tmp)
                    res = tmp;
            }
            *dst++ = *res;
        }
    }
};

/**
 * Select an iterator randomly from the interval [from, to), according
 * to the probability distribution described by the elements (of type
 * ScoreT) of these iterators.  The higher the ScoreT, the higher the
 * probability of being selected.
 *
 * To work effectively, the following assumption must hold:
 *    sum = sum_{x in [from, to) } (*x)
 * where (*x) is the dereferenced iterator.  That is, the sum
 * should be the sum of all of the iterated elements.
 */
template<typename It, typename ScoreT>
It roulette_select(It from, It to, ScoreT sum, RandGen& rng = randGen())
{
    sum = ScoreT(double(sum) * rng.randdouble());
    do {
        sum -= *from++;
    } while ((sum > 0) && (from != to));

    // Do not use sum >=0 in the above. For integer-valued arrays,
    // it can happen that *from are all zero just at the point where
    // sum == 0, which causes the above to increment to the end of
    // the array.
    return --from;
}

template<typename It>
It roulette_select(It from, It to, RandGen& rng = randGen())
{
    typedef typename std::iterator_traits<It>::value_type score_type;
    return roulette_select(from, to,
                           std::accumulate(from, to, score_type(0)),
                           rng);
}

template<typename NodeT>
class NodeSelector
{
public:
    typedef NodeT value_type;
    typedef std::vector<std::pair<NodeT, int> > PSeq;

    NodeSelector(RandGen& _rng = randGen()) : rng(_rng) {
    }

    NodeT select(int arity) const {
        return roulette_select
               (boost::make_transform_iterator
                (_byArity[arity].begin(),
                 select2nd<typename PSeq::value_type>()),
                boost::make_transform_iterator
                (_byArity[arity].end(), select2nd<typename PSeq::value_type>()),
                _aritySums[arity], rng).base()->first;
    }
    int select_arity(int from) const {
        //could make this slightly faster by caching the partial sums,
        //but who cares?
        return distance(_aritySums.begin(),
                        roulette_select(_aritySums.begin() + from,
                                        _aritySums.end(), rng));
    }

    void add(const NodeT& n, int arity, int prob) {
        if ((int)_byArity.size() <= arity) {
            _byArity.resize(arity + 1);
            _aritySums.resize(arity + 1, 0);
        }
        _byArity[arity].push_back(make_pair(n, prob));
        _aritySums[arity] += prob;
    }
private:
    RandGen& rng;
    std::vector<PSeq> _byArity;
    std::vector<int> _aritySums;
};

/** @}*/
} //~namespace opencog

#endif
