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

#include <opencog/atoms/base/Handle.h>

namespace opencog
{
namespace nlp
{

class FuzzySCM
{
    private:
        static void* init_in_guile(void*);
        static void init_in_module(void*);
        void init(void);

        Handle find_approximate_match(Handle);
        Handle do_nlp_fuzzy_match(Handle, Type, const HandleSeq&,bool);
        Handle do_nlp_fuzzy_compare(Handle, Handle);

    public:
        FuzzySCM();
};

}
}

extern "C" {
void opencog_nlp_fuzzy_init(void);
};


#include <opencog/guile/SchemePrimitive.h>
#include <opencog/nlp/types/atom_types.h>

#include "Fuzzy.h"
#include "FuzzyMatchBasic.h"

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
    define_scheme_primitive("cog-fuzzy-match",
        &FuzzySCM::find_approximate_match, this, "nlp fuzzy");
    define_scheme_primitive("nlp-fuzzy-match", &FuzzySCM::do_nlp_fuzzy_match,
                            this, "nlp fuzzy");

    define_scheme_primitive("nlp-fuzzy-compare", &FuzzySCM::do_nlp_fuzzy_compare,
                            this, "nlp fuzzy");
}

Handle FuzzySCM::find_approximate_match(Handle hp)
{
	FuzzyMatchBasic fpm;
	RankedHandleSeq ranked_solns = fpm.perform_search(hp);
	HandleSeq solns;
	for (auto rs: ranked_solns)
		solns.emplace_back(rs.first);

	AtomSpacePtr asp = SchemeSmob::ss_get_env_as("cog-fuzzy-match");
	return asp->add_link(LIST_LINK, std::move(solns));
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
    AtomSpacePtr asp = SchemeSmob::ss_get_env_as("nlp-fuzzy-match");

    Fuzzy fpm(asp.get(), rtn_type, excl_list, af_only);

    // A vector of solutions sorted in descending order of similarity
    RankedHandleSeq solns = fpm.perform_search(pat);
    HandleSeq rtn_solns;

    // Create NumberNodes to store the similarity scores, wrap together
    // with the Handles of the solutions in ReferenceLinks
    for (auto soln : solns) {
        Handle l = asp->add_link(REFERENCE_LINK, soln.first,
                       asp->add_node(NUMBER_NODE, std::to_string(soln.second)));
        rtn_solns.push_back(l);
    }

    // Wrap everything in a ListLink and then return it
    Handle results = asp->add_link(LIST_LINK, std::move(rtn_solns));

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
    AtomSpacePtr asp = SchemeSmob::ss_get_env_as("nlp-fuzzy-compare");

    Fuzzy fpm(asp.get());

    double score = fpm.fuzzy_compare(h1, h2);

    return asp->add_node(NUMBER_NODE, std::to_string(score));
}

void opencog_nlp_fuzzy_init(void)
{
    static FuzzySCM fuzzy;
}
