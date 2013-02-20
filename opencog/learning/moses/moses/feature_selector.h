/*
 * opencog/learning/moses/moses/feature_selector.cc
 *
 * Copyright (C) 2012 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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

#include <opencog/comboreduct/table/table.h>
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/learning/feature-selection/main/feature-selection.h>

namespace opencog {

/**
 * Struct in charge of selecting features maximize the amount of
 * information in combination with a candidate.
 */
struct feature_selector
{
    typedef std::set<combo::arity_t> feature_set;

    feature_selector(const combo::CTable& ctable,
                     const feature_selection_parameters& fs_params)
        : _ctable(ctable), _fs_params(fs_params)
    {}

    feature_selector(const combo::Table& table,
                     const feature_selection_parameters& fs_params)
        : _ctable(table.compressed()), _fs_params(fs_params)
    {}

    /// Return a feature set that correlates well with all the rows
    /// where the combo tree is predicting an incorrect result.
    /// The hope is that these will be used to build a more accurate
    /// models.
    feature_set operator()(const combo::combo_tree& tr) const
    {
        combo::CTable activ_ctable(_ctable.get_labels(), _ctable.get_signature());
        interpreter_visitor iv(tr);
        auto ai = boost::apply_visitor(iv);
        for (const combo::CTable::value_type& vct : _ctable) {
            vertex predicted_out = ai(vct.first.get_variant());
#define __LOOKAT_MOST_FREQUENT__
#ifdef __LOOKAT_MOST_FREQUENT__
            // Use only rows where the model is just plain wrong.  The
            // feature selector will then look for any features that
            // correlate well with these incorrect answers.
            Counter<vertex, unsigned> cnt = vct.second;
            vertex actual_out = cnt.most_frequent();
            if (predicted_out != actual_out)
                activ_ctable.insert(vct);
#else
            // Use only rows where the model is right, that is in the
            // case the prediction bscore is used because only
            // positive answers matter
            if (predicted_out == combo::id::logical_true)
                act_ctable.insert(vct);
#endif
        }
        // Call feature selection
        return select_features(activ_ctable, _fs_params);
        // return feature_set();
    }

    const combo::CTable& _ctable;
    feature_selection_parameters _fs_params;
};

} // ~namespace opencog
