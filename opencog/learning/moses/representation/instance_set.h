/*
 * opencog/learning/moses/representation/instance_set.h
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
#ifndef _REP_INSTANCE_SET_H
#define _REP_INSTANCE_SET_H

#include <vector>

#include <boost/iterator/transform_iterator.hpp>

#include "field_set.h"
#include "scored_instance.h"
#include "../moses/types.h"

// This is a workaround to the improper implementation of
// emplace_back() in gcc-4.6, it uses assignement operator=, but since
// a const cannot be assigned a new value _fields has to be un-const
#if (__GNUC__ == 4) && (__GNUC_MINOR__ == 6)
#define GCC46_EMPLACE_BACK_WORKAROUND
#endif

#ifdef GCC46_EMPLACE_BACK_WORKAROUND
#define MAYBE_CONST
#else
#define MAYBE_CONST const
#endif

namespace opencog {
namespace moses {

template<typename ScoreT>
struct instance_set : public std::vector<scored_instance<ScoreT> >
{
    typedef std::vector<scored_instance<ScoreT> > super;
    typedef typename super::value_type value_type;
    typedef boost::transform_iterator < select_tag,
                                        typename super::iterator,
                                        ScoreT&,
                                        ScoreT& > score_iterator;
    typedef boost::transform_iterator < select_tag,
                                        typename super::const_iterator,
                                        const ScoreT&,
                                        const ScoreT& > const_score_iterator;
    typedef boost::transform_iterator < select_item,
                                        typename super::iterator,
                                        instance&,
                                        instance& > instance_iterator;
    typedef boost::transform_iterator < select_item,
                                        typename super::const_iterator,
                                        const instance&,
                                        const instance& > const_instance_iterator;

    // Create a deme initialized with n null instances.
    instance_set(unsigned int n, MAYBE_CONST field_set& fs)
        : super(n, instance(fs.packed_width())), _fields(fs),
          n_evals(0), n_best_evals(0) {}
    // Create a deme initialized with n instances of inst.
    instance_set(unsigned int n, const instance& inst, MAYBE_CONST field_set& fs)
        : super(n, inst), _fields(fs),
          n_evals(0), n_best_evals(0) {}
    // Create an empty deme.
    instance_set(MAYBE_CONST field_set& fs, const demeID_t& id = demeID_t())
        : _fields(fs), _id(id),
          n_evals(0), n_best_evals(0) {}

#ifdef GCC46_EMPLACE_BACK_WORKAROUND
    instance_set<ScoreT>& operator=(const instance_set<ScoreT>& rhs) {
        _fields = rhs.fields();
        _id = rhs.getID();
        n_evals = rhs.n_evals;
        n_best_evals = rhs.n_best_evals;
        return *this;
    }
#endif

    // Insert or erase instances so that the size becomes n.
    // In case of insertions, it will insert null instances.
    void resize(unsigned int n) {
        super::resize(n, instance(_fields.packed_width()));
    }

    const field_set& fields() const {
        return _fields;
    }

#ifdef GCC46_EMPLACE_BACK_WORKAROUND
    field_set& fields() {
        return _fields;
    }
#endif

    score_iterator begin_scores() {
        return score_iterator(this->begin());
    }
    score_iterator end_scores() {
        return score_iterator(this->end());
    }
    /*const_score_iterator begin_scores() const {
      return const_score_iterator(begin());
      }*/

    //instance_iterator begin_instances() { return instance_iterator(begin()); }
    //instance_iterator end_instances() { return instance_iterator(end()); }
    const_instance_iterator begin_instances() const {
        return const_instance_iterator(this->begin());
    }
    const_instance_iterator end_instances() const {
        return const_instance_iterator(this->end());
    }

    void setID(const demeID_t& id) { _id = id; }
    demeID_t getID() const { return _id; }
protected:
    MAYBE_CONST field_set& _fields;

    // ID of the deme
    demeID_t _id;

public:
    // Number of evaluations performed on that deme
    unsigned n_evals;

    // Number of evaluations performed on that deme to reach the best
    // candidate (of that deme)
    unsigned n_best_evals;
};

} // ~namespace moses
} // ~namespace opencog

#undef GCC46_EMPLACE_BACK_WORKAROUND
#undef MAYBE_CONST

#endif
