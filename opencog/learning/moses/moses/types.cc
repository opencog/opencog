/*
 * opencog/learning/moses/moses/types.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2014 Aidyia Limited
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

#include <sstream>
#include <string>

#include <boost/functional/hash.hpp>

#include "types.h"
#include "complexity.h"


namespace opencog { namespace moses {

using namespace std;

demeID_t::demeID_t(unsigned expansion)
    : string(to_string(expansion)) {}
demeID_t::demeID_t(unsigned expansion, unsigned breadth_first)
    : string(to_string(expansion) + "." + to_string(breadth_first)) {}

size_t scored_combo_tree_hash::operator()(const scored_combo_tree& sct) const
{
    size_t hash = 0;
    const combo::combo_tree& tr = sct.get_tree();
    for (combo::vertex vtx : tr) {
        boost::hash_combine(hash, combo::hash_value(vtx));
    }
    return hash;
}

bool scored_combo_tree_equal::operator()(const scored_combo_tree& tr1,
                                         const scored_combo_tree& tr2) const
{
    return tr1.get_tree() == tr2.get_tree();
}

// See header file for description.
bool sct_score_greater::operator()(const scored_combo_tree& bs_tr1,
                                   const scored_combo_tree& bs_tr2) const
{
    const composite_score csc1 = bs_tr1.get_composite_score();
    const composite_score csc2 = bs_tr2.get_composite_score();

    if (csc1 > csc2) return true;
    if (csc1 < csc2) return false;

    // If we are here, then they are equal.  We are desperate to break
    // a tie, because otherwise, the scored_combo_tree_ptr_set will discard
    // anything that compares equal, and we really don't want that.
    score_t sc1 = csc1.get_score();
    score_t sc2 = csc2.get_score();

    if (sc1 > sc2) return true;
    if (sc1 < sc2) return false;

    // Arghh, still tied!  The above already used complexity to break
    // the tie.  Lets look at how the size of the trees compare. Note
    // that size_tree_order uses tree size first, then the lexicographic
    // order on the trees themselves, next.
    return size_tree_order<vertex>()(bs_tr1.get_tree(), bs_tr2.get_tree());
}

// See header file for description.
bool sct_tree_greater::operator()(const scored_combo_tree& bs_tr1,
                                  const scored_combo_tree& bs_tr2) const
{
    // size_tree_order first uses tree size, then the lexicographic
    // order on the trees themselves, for comparisons.
    return size_tree_order<vertex>()(bs_tr1.get_tree(), bs_tr2.get_tree());
}

// the empty composite_score ctor returns the worst composite score
const composite_score worst_composite_score = composite_score();

composite_score::composite_score()
    : score(very_worst_score), complexity(least_complexity),
      complexity_penalty(0.0), diversity_penalty(0.0),
      penalized_score(very_worst_score)
{}

composite_score& composite_score::operator=(const composite_score &r)
{
    score = r.score;
    complexity = r.complexity;
    penalized_score = r.penalized_score;
    complexity_penalty = r.complexity_penalty;
    diversity_penalty = r.diversity_penalty;
    multiply_diversity = r.multiply_diversity;
    return *this;
}

bool composite_score::operator<(const composite_score &r) const
{
    score_t lef = penalized_score;
    score_t rig = r.penalized_score;

    if (isnan(lef))
        return !isnan(rig);
    else
        return (lef < rig)
                // Note: I've tried to see if the addition below
                // increases the performance when there is no
                // complexity penalty and it doesn't, over 100 runs
                // solving 3-parity is actually .5s slower (which
                // probably has no statistical significance given it
                // takes 13s in average). But I don't want to conclude
                // too fast, it could have an impact on other
                // problems.
                or (lef == rig and complexity > r.complexity);
}

// Check for equality, to within floating-point error.
bool composite_score::operator==(const composite_score &r) const
{
    // score_t and complexity_t are both defacto floats.
    // equality holds if they are equal within about 7 decimal places.
    // Note: this check is used in iostream_bscored_combo_treeUTest
    // and a simple equality test will fail on some cpu/os combos.
    #define FLOAT_EPS 1.0e-7
    #define CHK_EQ(NAME)  \
        ((NAME == r.get_##NAME()) || /* Try this cheap test first */ \
         (fabs(NAME) > 0.0f) ?  \
            (fabs(r.get_##NAME() / NAME - 1.0f) < FLOAT_EPS) : \
            (fabs(NAME - r.get_##NAME()) < FLOAT_EPS))
    return
        CHK_EQ(score)
        && CHK_EQ(complexity)
        && CHK_EQ(complexity_penalty)
        && CHK_EQ(diversity_penalty)
        && CHK_EQ(penalized_score)
        ;
}

static const std::string behavioral_score_prefix_str = "behavioral score:";

std::ostream& ostream_behavioral_score(std::ostream& out, const behavioral_score& bs)
{
    out << behavioral_score_prefix_str << " ";
    return ostreamContainer(out, bs, " ", "[", "]");
}

/** stream out a scored combo tree */
std::ostream& ostream_scored_combo_tree(std::ostream& out, const scored_combo_tree& sct)
{
    out << sct.get_score() << " " << sct.get_tree() << std::endl
        << "weight=" << sct.get_weight() << " " 
        << sct.get_composite_score();

    const behavioral_score& bs = sct.get_bscore();
    if (0 < bs.size()) {
        out << std::endl;
        ostream_behavioral_score(out, bs);
    }
    out << std::endl;

    return out;
}


/// Stream in a scored_combo_tree, using any format that vaguely
/// resembles ostream_scored_combo_tree.  We assume that the combo tree
/// is always preceeded by the score, for backwards compatibility.
///
scored_combo_tree istream_scored_combo_tree(std::istream& in)
{
    static const char* score_str = "score";
    static const char* weight_str = "weight";
    static const char* complexity_str = "complexity";
    static const char* complexity_penalty_str = "complexity penalty";
    static const char* diversity_penalty_str = "diversity penalty";
    static const char* behavioral_score_str = "behavioral score";

    // parse score
    score_t sc;
    in >> sc;

    // parse combo tree
    combo::combo_tree tr;
    in >> tr;

    // parse the rest
    complexity_t cpx = 0;
    score_t cpx_penalty = 0, diversity_penalty = 0;
    score_t weight = 0.0;
    behavioral_score bs;

    // The following parser assumes that the input is of the form
    // key=value or key:value, and is separated by commas, spaces
    // or tabs. Any brackets [] braces {} or parens () in the input
    // are ignored.
    std::string line;
    std::getline(in, line);
    char* pline = strdup(line.c_str());
    char* key = pline;
    while (true) {
        char* eq = strpbrk(key, ":=");
        if (NULL == eq) break;
        *eq = 0x0;
        eq++;
        char *tail = strpbrk(eq, " ,[](){}\t");
        if (tail) *tail = 0x0;

        double val = atof(eq);

        // large string case statement
        if (0 == strcmp(key, score_str)) {
            sc = val;
        }
        else if (0 == strcmp(key, complexity_str)) {
            cpx = val;
        }
        else if (0 == strcmp(key, weight_str)) {
            weight = val;
        }
        else if (0 == strcmp(key, complexity_penalty_str)) {
            cpx_penalty = val;
        }
        else if (0 == strcmp(key, diversity_penalty_str)) {
            diversity_penalty = val;
        }

        if (tail) {
             // next key starts at the first non-junk character
             key = tail+1;
             key += strspn(key, " ,[](){}\t");
        }
        else break;
      
    }
    free(pline);

    // OK, now we assume that the behavioral score is on a line all
    // by itself, and consists of a squence of numbers, surrounded
    // by brackets.
    std::getline(in, line);
    pline = strdup(line.c_str());
    key = pline;
    char* eq = strpbrk(key, ":=");
    if (eq) {
        *eq = 0x0;
        eq++;

        if (0 == strcmp(key, behavioral_score_str)) {
            istringstream iss(eq);
            istreamContainer(iss, back_inserter(bs), "[", "]");
        }
    }
    free(pline);

    // assign to candidate
    combo::combo_tree tr_test = tr;

    composite_score cs(sc, cpx, cpx_penalty, diversity_penalty);
    scored_combo_tree sct(tr, /* default demeID */ 0, cs, bs);
    sct.set_weight(weight);
    return sct;
}



} // ~namespace moses
} // ~namespace opencog
