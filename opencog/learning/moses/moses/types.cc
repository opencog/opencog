/*
 * opencog/learning/moses/moses/types.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2014 Aidyia Limited
 * All Rights Reserved
 *
 * Written by Moshe Looks, Nil Geisweiller, Linas Vepstas
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

// std::regex will only be implemented in gcc 4.9. Meanwhile we use
// boost::regex
#include <boost/regex.hpp>

#include "types.h"
#include "complexity.h"


namespace opencog { namespace moses {

using namespace std;

demeID_t::demeID_t(unsigned expansion)
    : string(to_string(expansion)) {}
demeID_t::demeID_t(unsigned expansion, unsigned breadth_first)
    : string(to_string(expansion) + "." + to_string(breadth_first)) {}
demeID_t::demeID_t(unsigned expansion, unsigned breadth_first, unsigned ss_deme)
    : string(to_string(expansion) + "." +
             to_string(breadth_first) + ".SS-" +
             to_string(ss_deme)) {}

bool scored_combo_tree::operator==(const scored_combo_tree& r) const {
    return get_tree() == r.get_tree()
        and get_demeID() == r.get_demeID()
        and get_bscore() == r.get_bscore()
        and get_weight() == r.get_weight()
        and get_composite_score() == r.get_composite_score();
}

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
    // Note: this check is used in iostream_bscored_combo_treeUTest
    // and a simple equality test will fail on some cpu/os combos.
    return isApproxEq(score, r.get_score())
        and isApproxEq(complexity, r.get_complexity())
        and isApproxEq(complexity_penalty, r.get_complexity_penalty())
        and isApproxEq(diversity_penalty, r.get_diversity_penalty())
        and isApproxEq(penalized_score, r.get_penalized_score());
}

static const std::string behavioral_score_prefix_str = "behavioral score:";

std::ostream& ostream_behavioral_score(std::ostream& out, const behavioral_score& bs)
{
    out << behavioral_score_prefix_str << " ";
    return ostreamContainer(out, bs, " ", "[", "]");
}

std::ostream& ostream_scored_combo_tree(std::ostream& out,
                                        const scored_combo_tree& sct,
                                        bool output_score,
                                        bool output_cscore,
                                        bool output_demeID,
                                        bool output_bscore)
{
    if (output_score)
        out << sct.get_score() << " ";
    out << sct.get_tree();

    // Is this really used?
    static const bool output_weight = false;
    if (output_weight)
        out << " weight:" << sct.get_weight();

    if (output_cscore)
        out << " " << sct.get_composite_score();

    if (output_demeID)
        out << " demeID: " << sct.get_demeID();

    if (output_bscore and sct.get_bscore().size() > 0)
        ostream_behavioral_score(out << " ", sct.get_bscore());

    return out << std::endl;
}


/// Stream in a scored_combo_tree, using the format that resembles
/// ostream_scored_combo_tree. Score, combo tree and composite score
/// must be present. Bscore is optional.
///
scored_combo_tree string_to_scored_combo_tree(const std::string& line)
{
    // cout << "line = " << line << "END" << endl;

    static const string uncaptured_float_re = "[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?";
    static const string float_re = string("(") + uncaptured_float_re + ")";
 

    // very crude because handed to another parser
    static const string combo_tree_re = "(.+)";

    static const string cscore_re = string("\\[")
        + "score=" + float_re + ", "
        + "penalized score=" + float_re + ", "
        + "complexity=" + "(\\d+)" + ", "
        + "complexity penalty=" + float_re + ", "
        + "diversity penalty=" + float_re + "\\]";

    static const string demeID_re = string("( demeID: ")
        + "([0-9]+)(\\.([0-9]+))?(\\.SS-([0-9]+))?)?";

    // very crude because handed to another parser
    static const string bscore_re = string("behavioral score: (\\[.+\\])");
    static const string opt_bscore_re = string("( ") + bscore_re + ")?"; // optional

    static const string scored_combo_tree_re = "^" + float_re + " "
        + combo_tree_re + " " + cscore_re + demeID_re + opt_bscore_re + "\n?$";

    static const boost::regex sct_regex(scored_combo_tree_re);

    // Match the regex
    boost::smatch sct_match;
    bool match_result = boost::regex_match(line, sct_match, sct_regex);

    // for (size_t i = 0; i < sct_match.size(); i++)
    //     cout << "sct_match[" << i << "] = \"" << sct_match[i] << "\"" << std::endl;

    OC_ASSERT(match_result, "Line '%s' doesn't match regex '%s'",
              line.c_str(), scored_combo_tree_re.c_str());
    OC_ASSERT(sct_match.size() == 16,
              "Wrong number of sub-matches %u, 10 are expected",
              sct_match.size());

    // Parse score
    score_t sc = std::stof(sct_match[1]);

    // Parse combo tree
    combo::combo_tree tr;
    std::stringstream(sct_match[2].str()) >> tr;

    // Parse composite score
    complexity_t cpx = std::stoi(sct_match[5].str());
    score_t cpx_penalty = std::stof(sct_match[6].str()),
        diversity_penalty = std::stof(sct_match[7].str());
    composite_score cs(sc, cpx, cpx_penalty, diversity_penalty);

    // Parse demeID
    demeID_t demeID;
    if (sct_match[8].length() > 0) {
        unsigned expansion = stoi(sct_match[9].str());
        demeID = demeID_t(expansion);
        if (sct_match[10].length() > 0) {
            unsigned breadth_first = stoi(sct_match[11].str());
            demeID = demeID_t(expansion, breadth_first);
            if (sct_match[12].length() > 0) {
                unsigned ss_deme = stoi(sct_match[13].str());
                demeID = demeID_t(expansion, breadth_first, ss_deme);
            }
        }
    }

    // Parse behavioral score
    behavioral_score bs;
    if (sct_match[15].length() > 0) {
        istringstream iss(sct_match[15].str());
        istreamContainer(iss, back_inserter(bs), "[", "]");
    }

    return scored_combo_tree(tr, demeID, cs, bs);
}

// Fill a vector of scored_combo_tree parsed from stream
std::istream& istream_scored_combo_trees(std::istream& in,
                                         std::vector<scored_combo_tree>& scts) {
    for (std::string line; std::getline(in, line); )
        scts.push_back(string_to_scored_combo_tree(line));
    return in;
}

} // ~namespace moses
} // ~namespace opencog
