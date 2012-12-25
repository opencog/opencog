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
                     feature_selection_parameters fs_params)
        : _ctable(ctable), _fs_params(fs_params)
    {}

    feature_selector(const combo::Table& table,
                     feature_selection_parameters fs_params)
        : _ctable(table.compressed()), _fs_params(fs_params)
    {}

    feature_set operator()(const combo::combo_tree& tr) const
    {
        combo::CTable activ_ctable(_ctable.get_labels(), _ctable.get_signature());
        // select active rows
        for (const combo::CTable::value_type& vct : _ctable)
            if (eval_binding(vct.first, tr) == combo::id::logical_true)
                activ_ctable.insert(vct);
        // Call feature selection
        return select_features(activ_ctable, _fs_params);
        // return feature_set();
    }

    const combo::CTable& _ctable;
    feature_selection_parameters _fs_params;
};

} // ~namespace opencog
