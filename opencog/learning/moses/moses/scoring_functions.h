/*
 * opencog/learning/moses/moses/scoring_functions.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
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
#ifndef _MOSES_SCORING_FUNCTIONS_H
#define _MOSES_SCORING_FUNCTIONS_H

#include <opencog/util/mt19937ar.h>
#include "types.h"
#include "using.h"

namespace opencog { namespace moses {

// ///////////////////// Scoring for truth table data /////////////////////////

struct ConfusionMatrix {
    int TP, FP, TN, FN;
};

struct CaseBasedBoolean : public unary_function<combo_tree, score_t>
{
    CaseBasedBoolean() { }

    CaseBasedBoolean(istream& in) {
        while (in.good()) {
            char foo[10000];
            in.getline(foo, 10000);
            string str(foo);
            if (!in.good() || str.empty())
                break;

            vector<bool> tmp;
            stringstream ss(str);
            bool v;
            ss >> v;
            while (ss.good()) {
                bool x;
                ss >> x;
                tmp.push_back(x);
            }
            tmp.push_back(v); //put the result last;

            if (!_cases.empty())
                assert(_cases.front().size() == tmp.size());

            _cases.push_back(tmp);
        }
        // cout << "#cases: " << _cases.size() << " arity " << arity() << endl;
    }

    bool bool_evaluate(const vector<bool>& bindings, const combo_tree& tr) const
    {
        static MT19937RandGen rng(0); // this is not useful anyway,
                                      // remove once rng has a factory
                                      // and is optional
        for(unsigned int i = 0; i < bindings.size(); ++i) {
            binding(i+1) = bool_to_vertex(bindings[i]);
        }
        return vertex_to_bool(eval_throws(rng, tr));
    }

    ConfusionMatrix ComputeConfusionMatrix(const combo_tree& tr) const {
        ConfusionMatrix cm;
        cm.TP = 0;
        cm.FP = 0;
        cm.TN = 0;
        cm.FN = 0;

        for (CaseSeq::const_iterator c = _cases.begin();c != _cases.end();++c) {
            if (bool_evaluate(*c, tr))
                (*c)[arity()] ? cm.TP++ : cm.FP++;
            else
                (*c)[arity()] ? cm.FN++ : cm.TN++;
        }

        return cm;
    }

    score_t operator()(const combo_tree& tr) const {
        score_t f=0;
        for (CaseSeq::const_iterator c = _cases.begin();c != _cases.end();++c) {
            f -= (bool_evaluate(*c, tr) != (*c)[arity()]);
        }
        return f;
    }

    void compute_behavior(const combo_tree& tr, behavioral_score& bs) const {
        int i = 0;
        for (CaseSeq::const_iterator c = _cases.begin();
             c != _cases.end();++c, ++i)
            bs[i] = (bool_evaluate(*c, tr) != (*c)[arity()]);
    }


    int arity() const {
        return _cases.front().size() - 1;
    }
    int number_of_cases() const {
        return _cases.size();
    }

protected:
    typedef vector<vector<bool> > CaseSeq;
    CaseSeq _cases;
};

struct truth_table_data_score : public unary_function<combo_tree, score_t>
{
    truth_table_data_score(const truth_table_data_score& score) : c(score.c) {}
    truth_table_data_score(struct CaseBasedBoolean& bc) {
        c = &bc;
    }

    score_t operator()(const combo_tree& tr) const {
        return c->operator()(tr);
    }

private:
    struct CaseBasedBoolean *c;
};

struct truth_table_data_bscore : public unary_function<combo_tree, behavioral_score>
{
    truth_table_data_bscore(struct CaseBasedBoolean& bc) {
        c = &bc;
    }

    behavioral_score operator()(const combo_tree& tr) const {
        behavioral_score bs(c->number_of_cases());

        c->compute_behavior(tr, bs);

        return bs;
    }

private:
    typedef vector<vector<bool> > CaseSeq;

    struct CaseBasedBoolean *c;
};

// ////////////// End of scoring for truth table ///////////////

struct interactive_score : public unary_function<combo_tree, score_t>
{
    interactive_score() {  }

    score_t operator()(const combo_tree& tr) const
    {
        cout << "Fitness Function of : " << tr << " enter the score :" << endl;
        score_t score = 0.0;
        cin >> score;
        return score;
    }
};


struct interactive_bscore : public unary_function<combo_tree, behavioral_score>
{
    interactive_bscore() {  }

    behavioral_score  operator()(const combo_tree& tr) const
    {
        behavioral_score bs(0);
        return bs;
    }
};


} // ~namespace moses
} // ~namespace opencog

#endif
