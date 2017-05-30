/*
 * FuzzySCM.cc
 *
 * Copyright (C) 2015, 2016 OpenCog Foundation
 * All Rights Reserved
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
 * @param self  pointer to the FuzzySCM object
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
 * @param data  pointer to the FuzzySCM object
 */
void FuzzySCM::init_in_module(void* data)
{
    FuzzySCM* self = (FuzzySCM*) data;
    self->init();
}

/**
 * The main init function for the FuzzySCM object.
 */
void FuzzySCM::init()
{
    define_scheme_primitive("nlp-fuzzy-match", &FuzzySCM::do_nlp_fuzzy_match,
                            this, "nlp fuzzy");

    define_scheme_primitive("nlp-fuzzy-compare", &FuzzySCM::do_nlp_fuzzy_compare,
                            this, "nlp fuzzy");
}

/**
 * Implement the "nlp-fuzzy-match" scheme primitive. It calls the nlp fuzzy
 * matcher to search for potential solutions.
 *
 * @param pat        The input pattern
 * @param rtn_type   The return type (the type of atom we are looking for)
 * @param excl_list  The exclude-list, containing a list of atoms that we
 *                   don't want in the result set
 * @return           A list of solutions and their similarity scores
 */
Handle FuzzySCM::do_nlp_fuzzy_match(Handle pat, Type rtn_type,
                                    const HandleSeq& excl_list,
                                    bool af_only)
{
    AtomSpace* as = SchemeSmob::ss_get_env_as("nlp-fuzzy-match");

    Fuzzy fpm(as, rtn_type, excl_list, af_only);

    // A vector of solutions sorted in descending order of similarity
    RankedHandleSeq solns = fpm.perform_search(pat);
    HandleSeq rtn_solns;

    // Create NumberNodes to store the similarity scores, wrap together
    // with the Handles of the solutions in ReferenceLinks
    for (auto soln : solns) {
        Handle l = as->add_link(REFERENCE_LINK, soln.first,
                       as->add_node(NUMBER_NODE, std::to_string(soln.second)));
        rtn_solns.push_back(l);
    }

    // Wrap everything in a ListLink and then return it
    Handle results = as->add_link(LIST_LINK, rtn_solns);

    return results;
}

/**
 * Implement the "nlp-fuzzy-compare" scheme primitive. It calls the nlp fuzzy
 * matcher to search for estimating how similar two trees are.
 *
 * @param h1, h2  The trees being compared
 * @return        A similarity score
 */
Handle FuzzySCM::do_nlp_fuzzy_compare(Handle h1, Handle h2)
{
    AtomSpace* as = SchemeSmob::ss_get_env_as("nlp-fuzzy-compare");

    Fuzzy fpm(as);

    double score = fpm.fuzzy_compare(h1, h2);

    return as->add_node(NUMBER_NODE, std::to_string(score));
}

void opencog_nlp_fuzzy_init(void)
{
    static FuzzySCM fuzzy;
}
