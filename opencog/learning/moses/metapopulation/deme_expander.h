/** deme_expander.h --- 
 *
 * Copyright (C) 2013 OpenCog Foundation
 *
 * Author: Nil Geisweiller 
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

#ifndef _OPENCOG_DEME_EXPANDER_H
#define _OPENCOG_DEME_EXPANDER_H

#include "../optimization/optimization.h"
#include "metapop_params.h"

namespace opencog {
namespace moses {

struct deme_expander
{
    typedef deme_t::iterator deme_it;
    typedef deme_t::const_iterator deme_cit;

    deme_expander(const type_tree& type_signature,
                  const reduct::rule& si_ca,
                  const reduct::rule& si_kb,
                  const cscore_base& sc,
                  optimizer_base& opt,
                  const metapop_parameters& pa = metapop_parameters()) :
        _rep(NULL), _deme(NULL),
        _optimize(opt),
        _type_sig(type_signature),
        simplify_candidate(si_ca),
        simplify_knob_building(si_kb),
        _cscorer(sc),
        _params(pa)
    {}

    ~deme_expander()
    {
        if (_rep) delete _rep;
        if (_deme) delete _deme;
    }

    /**
     * Create the deme
     *
     * @return return true if it creates deme successfully,otherwise false.
     */
    // bool create_deme(pbscored_combo_tree_set::const_iterator exemplar)
    bool create_deme(const combo_tree& exemplar);

    /**
     * Do some optimization according to the scoring function.
     *
     * @param max_evals the maximum number of evaluations of the scoring
     *                  function to perform.
     * @param max_time the maximum elapsed (wall-clock) time to allow.
     *
     * @return return the number of evaluations actually performed,
     */
    int optimize_deme(int max_evals, time_t max_time);

    void free_deme();

    // Structures related to the current deme
    representation* _rep; // representation of the current deme
    deme_t* _deme; // current deme
    optimizer_base &_optimize;

protected:
    /**
     * 
     */
    
    /**
     * Takes a feature set, a header of the input features and return
     * a vector of the names of the featire set.
     */
    string_seq fs_to_names(const feature_set& fs, const string_seq& ilabels) const;

    /**
     * Log selected features, the new ones and the ones part of the
     * exemplar.
     */
    void log_selected_features(const feature_set& selected_features,
                               const feature_set& xmplr_features,
                               const string_seq& ilabels) const;

    /**
     * Prune exemplar from non-selected features
     */
    void prune_xmplr(combo_tree& xmplr,
                     const feature_set& selected_features) const;

    const combo::type_tree& _type_sig;    // type signature of the exemplar
    const reduct::rule& simplify_candidate; // to simplify candidates
    const reduct::rule& simplify_knob_building; // during knob building
    const cscore_base& _cscorer; // composite score
    metapop_parameters _params;
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_DEME_EXPANDER_H
