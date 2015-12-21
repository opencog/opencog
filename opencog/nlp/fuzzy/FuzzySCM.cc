/*
 * FuzzySCM.cc
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

//#include <opencog/util/Logger.h>
//#include <opencog/atomutils/AtomUtils.h>
//#include <opencog/atomutils/FindUtils.h>
//#include <opencog/atoms/pattern/PatternUtils.h>
#include <opencog/atoms/pattern/PatternLink.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/nlp/types/atom_types.h>

#include "FuzzySCM.h"
#include "Fuzzy.h"

using namespace opencog::nlp;
using namespace opencog;

/**
 * The constructor for FuzzySCM.
 */
FuzzySCM::FuzzySCM()
{
    static bool is_init = false;
    if (is_init) return;
    is_init = true;
    scm_with_guile(init_in_guile, this);
}

/**
 * Init function for using with scm_with_guile.
 *
 * Creates the fuzzy scheme module and uses it by default.
 *
 * @param self   pointer to the FuzzySCM object
 * @return       null
 */
void* FuzzySCM::init_in_guile(void* self)
{
    scm_c_define_module("opencog nlp fuzzy", init_in_module, self);
    scm_c_use_module("opencog nlp fuzzy");
    return NULL;
}

/**
 * The main function for defining stuff in the fuzzy scheme module.
 *
 * @param data   pointer to the FuzzySCM object
 */
void FuzzySCM::init_in_module(void* data)
{
    FuzzySCM* self = (FuzzySCM*) data;
    self->init();
}

void opencog_nlp_fuzzy_init(void)
{
    static FuzzySCM fuzzy;
}

/**
 * The main init function for the FuzzySCM object.
 */
void FuzzySCM::init()
{
#ifdef HAVE_GUILE
    define_scheme_primitive("nlp-fuzzy-match", &FuzzySCM::do_nlp_fuzzy_match, this, "nlp fuzzy");
#endif
}

/**
 * Implement the "nlp-fuzzy-match" scheme primitive.
 *
 * @param
 * @return
 */
Handle FuzzySCM::do_nlp_fuzzy_match(Handle pat, Type rtn_type,
                                    const HandleSeq& excl_list)
{
#ifdef HAVE_GUILE
    AtomSpace* as = SchemeSmob::ss_get_env_as("nlp-fuzzy-match");

    Fuzzy fpm(as, rtn_type, excl_list);

    HandleSeq terms;
    terms.push_back(pat);

    std::set<Handle> no_vars;

    PatternLinkPtr slp(createPatternLink(no_vars, terms));
    slp->satisfy(fpm);

    std::vector<std::pair<Handle, double>> solns = fpm.get_solns();
    HandleSeq rtn_solns;

    for (auto soln : solns) {
        Handle l = as->add_link(REFERENCE_LINK, soln.first,
                                as->add_node(NUMBER_NODE, std::to_string(soln.second)));
        rtn_solns.push_back(l);
    }

    // The result_list contains a list of the grounded expressions.
    // Turn it into a true list, and return it.
    Handle results = as->add_link(LIST_LINK, rtn_solns);

    return results;
#else
    return Handle::UNDEFINED;
#endif
}

