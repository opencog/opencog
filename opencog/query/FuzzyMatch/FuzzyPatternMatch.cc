/*
 * FuzzyPatternMatch.cc
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemeSmob.h>
#include "FuzzyPatternMatchCB.h"
#include "FuzzyPatternMatch.h"

using namespace opencog;

FuzzyPatternMatch::FuzzyPatternMatch()
{

}

/**
 * Implement the "cog-fuzzy-match" scheme primitive.
 *
 * Use Pattern Matcher to find the candidates, estimate the similarity by
 * computing the edit distance for each of the candidate, eventually return one
 * or more of the candidates that are the most similar to the query hypergraph.
 *
 * @param hg  The query hypergraph
 * @return    One or more similar hypergraphs
 */
HandleSeq FuzzyPatternMatch::find_approximate_matches(Handle hg)
{
#ifdef HAVE_GUILE
    AtomSpace* as = SchemeSmob::ss_get_env_as("cog-fuzzy-match");

    FuzzyPatternMatchCB fpmcb(as);

    std::set<Handle> vars;

    HandleSeq preds;
    preds.push_back(hg);

    HandleSeq negs;

    pme.match(&fpmcb, vars, preds, negs);

    return fpmcb.solns;
#else
    return Handle::UNDEFINED;
#endif
}
