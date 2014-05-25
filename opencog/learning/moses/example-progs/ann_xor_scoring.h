/*
 * opencog/learning/moses/example-progs/ann_xor_scoring.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Joel Lehman
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
#ifndef _ANN_XOR_SCORING_H
#define _ANN_XOR_SCORING_H

#include <opencog/util/numeric.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/simple_nn.h>
#include <opencog/comboreduct/reduct/ann_rules.h>

#include "../scoring/scoring.h"

using namespace opencog;
using namespace combo;
using namespace std;
using namespace moses;
#define MIN_FITNESS -1.0e10

// Demo scoring function for solvig the binary XOR function, suing neural nets.
struct AnnXORFitnessFunction : public unary_function<combo_tree, double>
{
    typedef combo_tree::iterator pre_it;
    typedef combo_tree::sibling_iterator sib_it;

    result_type operator()(argument_type tr) const
    {
        if (tr.empty())
            return MIN_FITNESS;

        tree_transform tt;

        // binary xor_problem. The third input is always 1.0, this is
        // "to potentially supply a constant 'bias' to influence the
        // behavior of other neurons" (Joel Lehman)
        double inputs[4][3] = { {0.0, 0.0, 1.0}, 
                                  {0.0, 1.0, 1.0}, 
                                  {1.0, 0.0, 1.0},
                                  {1.0, 1.0, 1.0}};
        double outputs[4] = {0.0, 1.0, 1.0, 0.0};

        ann nn = tt.decodify_tree(tr);
        int depth = nn.feedforward_depth();

        double error = 0.0;
        for (int pattern = 0;pattern < 4;++pattern) {
            nn.load_inputs(inputs[pattern]);
            dorepeat(depth)
                nn.propagate();
            double diff = outputs[pattern] - nn.outputs[0]->activation;
            error += diff * diff;
        }

        return -error;
    }

};

// This is what the original source had this as, but its not
// obviously correct, to me.
#define CPXY_RATIO 1.0

struct ann_xor_cscore  : public cscore_base
{
    composite_score operator()(const combo_tree& tr) const
    {
        complexity_t cpxy = tr.size();
        // Note minus sign!
        return composite_score(-aff.operator()(tr), cpxy, cpxy/CPXY_RATIO);
    }

    AnnXORFitnessFunction aff;
};

struct ann_xor_bscore : public bscore_base
{
    behavioral_score operator()(const combo_tree& tr) const
    {
        composite_score cs(ann_xor_cscore()(tr));

        behavioral_score bs;
        bs.push_back(cs.get_score());
        bs.push_back(cs.get_penalty());
        return bs;
    }
    behavioral_score best_possible_bscore() const
    {
        return {0.0};
    }
};

#endif
