#include <opencog/comboreduct/combo/eval.h>

#include <iostream>
#include <opencog/learning/moses/moses/moses.h>
#include <opencog/learning/moses/moses/optimization.h>
#include <opencog/learning/moses/moses/scoring_functions.h>
#include <opencog/learning/moses/moses/scoring.h>
#include <opencog/learning/moses/moses/ann_scoring.h>

#include <opencog/util/mt19937ar.h>

#include <opencog/util/Logger.h>

#include <opencog/learning/moses/moses/representation.h>

using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;
using namespace opencog;


int main(int argc, char** argv)
{

    //set flag to print only cassert and other ERROR level logs on stdout
    opencog::logger().setPrintErrorLevelStdout();

    //read in maximum evaluations and RNG seed from command line
    int max_evals;
    int seed;
    bool reduce=true;
    try {
        //if (argc!=3)
        //    throw "foo";
        max_evals=lexical_cast<int>(argv[1]);
        seed=lexical_cast<int>(argv[2]);
        
        
        set_stepsize(lexical_cast<double>(argv[3]));
        set_expansion(lexical_cast<double>(argv[4]));
        set_depth(lexical_cast<int>(argv[5]));

        reduce = lexical_cast<int>(argv[6]);
    } catch (...) {
        cerr << "usage: " << argv[0] << " maxevals seed" << endl;
        exit(1);
    }
    
    //read seed tree in from stdin
    combo_tree tr;
    cin >> tr; 

    opencog::MT19937RandGen rng(seed);

    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(), id::ann_type, 1);
    
    const reduct::rule* si = &(ann_reduction());
    if(!reduce)
        si = &(clean_reduction());

    //SINGLE MARKOVIAN POLE TASK
    ann_pole_score p_score;
    ann_pole_bscore p_bscore; 
    metapopulation<ann_pole_score, ann_pole_bscore, univariate_optimization>
    metapop_pole(rng, tr,
            tt, *si,
            p_score,
            p_bscore,
            univariate_optimization(rng));

    moses::moses(metapop_pole, max_evals, 100000);

    //change best tree into ANN
    tree_transform trans; 
    combo_tree best = metapop_pole.best_trees().front();
    ann bestnet = trans.decodify_tree(best);
   
    //write out the best network
    cout << "Best network: " << endl; 
    cout << &bestnet << endl;

    //write out in dot format
    bestnet.write_dot("best_nn.dot");
    
    //for parameter sweep
    cout << metapop_pole.best_score().first << endl;
}




