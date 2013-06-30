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
#ifndef _EDA_INSTANCE_SET_H
#define _EDA_INSTANCE_SET_H

#include "field_set.h"
#include "../eda/scoring.h"
#include "../moses/types.h"

namespace opencog {
namespace moses {

template<typename ScoreT>
struct instance_set : public vector<scored_instance<ScoreT> >
{
    typedef vector<scored_instance<ScoreT> > super;
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
    instance_set(unsigned int n, const field_set& fs)
        : super(n, instance(fs.packed_width())), _fields(fs) { }
    // Create a deme initialized with n instances of inst.
    instance_set(unsigned int n, const instance& inst, const field_set& fs)
        : super(n, inst), _fields(fs) { }
    // Create an empty deme.
    instance_set(const field_set& fs, const demeID_t& id = demeID_t())
        : _fields(fs), _id(id) { }

    // Insert or erase instances so that the size becomes n.
    // In case of insertions, it will insert null instances.
    void resize(unsigned int n) {
        super::resize(n, instance(_fields.packed_width()));
    }

    const field_set& fields() const {
        return _fields;
    }

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
    const field_set& _fields;
    demeID_t _id;
};

} // ~namespace moses
} // ~namespace opencog

#endif
