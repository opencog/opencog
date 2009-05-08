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

#include "using.h"

using namespace std;
using namespace boost;


namespace moses
{

struct even_parity {
    template<typename It>
    bool operator()(It from,It to) const {
        bool parity = true;
        while (from != to)
            parity ^= *from++;
        return parity;
    }
};

struct disjunction {
    template<typename It>
    bool operator()(It from,It to) const {
        while (from != to)
            if (*from++)
                return true;
        return false;
    }
};

//simple function : f(x)_o = sum_{i={1,o}} x^i
//that is for instance:
//f(x)_3 = x+x^2+x^3
//f(x)_2 = x+x^2
//f(x)_1 = x
//f(x)_0 = 0
struct simple_symbolic_regression {
    simple_symbolic_regression(int o = 4) : order(o) { }
    int order;
    template<typename It>
    contin_t operator()(It from,It to) const {
        contin_t res = 0;
        dorepeat(order)
        res = (res + contin_t(1)) * (*from);
        cout << "simple_symbolic_regression: "
             << "f(" << *from << ")_" << order << "=" << res << endl;
        return res;
    }
};




// ///////////////////// Scoring for truth table data /////////////////////////


template<typename State, typename iter>
bool bool_evaluate(const State& bindings, iter it)
{
    using namespace id;
    typedef typename iter::sibling_iterator sib_it;

    if (!(*it == id::boolean_if || *it == id::logical_and ||
          *it == id::logical_or || *it == id::logical_not ||
          *it == id::logical_true || *it == id::logical_false))  {
        if (get_argument(*it).idx < 0) // negation
            return !bindings[-get_argument(*it).idx-1];
        else
            return bindings[get_argument(*it).idx-1];
    }

    if (*it == id::boolean_if)
        return (bool_evaluate(bindings, it.begin()) ?
                bool_evaluate(bindings, ++it.begin()) :
                bool_evaluate(bindings, --it.end()));

    if (it.begin() == it.end())
        return (*it == id::logical_or || *it == id::logical_true);

    if (*it == id::logical_or) {
        for (sib_it sib = it.begin();sib != it.end();++sib)
            if (bool_evaluate(bindings, sib))
                return true;
        return false;
    }
    if (*it == id::logical_and) {
        for (sib_it sib = it.begin();sib != it.end();++sib)
            if (!bool_evaluate(bindings, sib))
                return false;
        return true;
    }
    if (*it == id::logical_not) {
        return !bool_evaluate(bindings, it.begin());
    }

    if (*it == id::logical_true)
        return true;
    if (*it == id::logical_false)
        return false;

    std::cout << "can't find " << *it << std::endl;
    assert(false);
    return false;
}


template<typename State, typename iter>
bool bool_evaluate(const State& bindings, const combo_tree& tr)
{
    return bool_evaluate(bindings, tr.begin());
}


struct ConfusionMatrix {
    int TP, FP, TN, FN;
};

struct CaseBasedBoolean : public unary_function<combo_tree, float> {
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

    ConfusionMatrix ComputeConfusionMatrix(const combo_tree& t) const {
        ConfusionMatrix cm;
        cm.TP = 0;
        cm.FP = 0;
        cm.TN = 0;
        cm.FN = 0;

        for (CaseSeq::const_iterator c = _cases.begin();c != _cases.end();++c) {
            if (bool_evaluate(*c, t.begin()))
                (*c)[arity()] ? cm.TP++ : cm.FP++;
            else
                (*c)[arity()] ? cm.FN++ : cm.TN++;
        }

        return cm;
    }


    fitness_t operator()(const combo_tree& t) const {
//    fitness_t penalty=fitness_t(-mycomplexity(t.begin()))/2.0f;
        fitness_t penalty = 0;
//    complexity_t cmin=get_complexity(t.begin());

        return fitness_t(t.empty() ? NEG_INFINITY :
                         1.0f*(fitness_t(operator()(t.begin()))+penalty));
    }


    template<typename iter>
    int operator()(iter src) const {
        //cout << "cbb" << endl;

        int f=0;

        // cout << "case:" << endl;
        // for (int k=1;k<=arity();k++)
        //   cout << "#" << k;
        // cout << endl;

        for (CaseSeq::const_iterator c = _cases.begin();c != _cases.end();++c) {
            f -= (bool_evaluate(*c, src) != (*c)[arity()]);
        }
        return f;
    }


    void compute_behavior(const combo_tree& t, behavioral_score& bs) const {
        int i = 0;
        for (CaseSeq::const_iterator c = _cases.begin();
             c != _cases.end();++c, ++i)
            bs[i] = (bool_evaluate(*c, t.begin()) != (*c)[arity()]);
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

struct truth_table_data_score {
    truth_table_data_score(struct CaseBasedBoolean& bc) {
        c = &bc;
    }

    int operator()(const combo_tree& tr) const {
        return (int)c->operator()(tr);
    }

private:
    struct CaseBasedBoolean *c;
};



struct truth_table_data_bscore {
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




// ////////////// End of scoring for Passenger-style data ///////////////




struct interactive_score {
    interactive_score() {  }

    int operator()(const combo_tree& tr) const {
        cout << "Fitness Function of : " << tr << " enter the score :" << endl;
        fitness_t score = 0.0;
        cin >> score;
        return (int)score;
    }
};


struct interactive_bscore {
    interactive_bscore() {  }

    behavioral_score  operator()(const combo_tree& tr) const {
        behavioral_score bs(0);

        return bs;
    }
};




struct ant_score {
    ant_score() {  }

    int operator()(const combo_tree& tr) const {
        return (int)(-1000 + aff(tr));
    }

    AntFitnessFunction aff;
};



struct ant_bscore {
    ant_bscore( ) { }

    behavioral_score operator()(const combo_tree& tr) const {
        behavioral_score bs(2);
        bs[0] = -moses::ant_score()(tr);
        bs[1] = tr.size();

        return bs;
    }
};




} //~namespace moses

#endif
