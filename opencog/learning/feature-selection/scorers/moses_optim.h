/** scorers/moses_optim.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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


#ifndef _OPENCOG_FS_SCORERS_OPTIM_H
#define _OPENCOG_FS_SCORERS_OPTIM_H

#include <opencog/util/numeric.h>

#include <opencog/comboreduct/combo/common_def.h>
#include <opencog/learning/moses/representation/field_set.h>
#include <opencog/learning/moses/representation/instance_scorer.h>
#include <opencog/learning/moses/moses/types.h>

namespace opencog {

using namespace moses;
using namespace combo;

/**
 * translate an instance into a feature set. Each feature is
 * represented by its index (the left most one is 0).
 */
std::set<arity_t> get_feature_set(const field_set& fields,
                                  const instance& inst);

/**
 * Wrapper to use a feature set scorer with MOSES's optimization
 * algorithms operating on a deme. Each deme is a binary string where
 * each bit represents whether a feature is selected or not.
 */
template<typename FSScorer>
struct deme_based_scorer : public iscorer_base
{
    deme_based_scorer(const FSScorer& fs_scorer, const field_set& fields)
        : _fs_scorer(fs_scorer), _fields(fields) {}

    /**
     * The feature set is represented by an instance encoding a
     * field of booleans. Each boolean represents whether its
     * corresponding feature is in the feature set of not.
     */
    composite_score operator()(const instance& inst) const
    {
        std::set<arity_t> fs = get_feature_set(_fields, inst);
        composite_score csc(_fs_scorer(fs), fs.size(), 0);
        // Logger
        if (logger().isFineEnabled()) {
            logger().fine()
               << "moses_based_scorer - Evaluate instance: " 
               << _fields.to_string(inst) << " " << csc;
        }
        // ~Logger
        return csc;
    }

    const FSScorer& _fs_scorer;
    const field_set& _fields;
};

} // ~namespace opencog

#endif // _OPENCOG_FS_SCORERS_OPTIM_H
