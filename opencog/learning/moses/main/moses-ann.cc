#include <opencog/comboreduct/combo/eval.h>

#include <iostream>
#include <opencog/learning/moses/moses/moses.h>
#include <opencog/learning/moses/moses/optimization.h>
#include <opencog/learning/moses/moses/scoring_functions.h>
#include <opencog/learning/moses/moses/scoring.h>
#include <opencog/learning/moses/moses/ann_scoring.h>

#include <opencog/util/mt19937ar.h>

#include <opencog/util/Logger.h>

using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;
using namespace opencog;


int main(int argc, char** argv)
{

    //set flag to print only cassert and other ERROR level logs on stdout
    opencog::logger().setPrintErrorLevelStdout();

    combo_tree tr;
    cin >> tr; //"ann(ann_node(ann_input rand))";

    tree_transform trans;
    ann nn = trans.decodify_tree(tr);
    cout << tr << endl;
    cout << "Network depth: " << nn.feedforward_depth() << endl;
    cout << &nn << endl;
    combo_tree blah = trans.encode_ann(nn);
    cout << blah << endl;

    opencog::MT19937RandGen rng(0);

    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(), id::ann_type, 1);

    ann_score score;
    ann_bscore bscore;

    metapopulation<ann_score, ann_bscore, univariate_optimization>
    metapop(rng, tr,
            tt, clean_reduction(),
            score,
            bscore,
            univariate_optimization(rng));

    moses::moses(metapop, 10000, 0);

//TEST THE BEST NET ON XOR
    combo_tree best = metapop.best_trees().front();
    ann bestnet = trans.decodify_tree(best);
    
    double inputs[4][3] = { {0.0, 0.0,1.0}, 
                                {0.0, 1.0,1.0}, 
                                {1.0, 0.0,1.0},
                                {1.0, 1.0,1.0}};
    
    int depth = bestnet.feedforward_depth();
    for (int pattern = 0;pattern < 4;pattern++) {
        bestnet.load_inputs(inputs[pattern]);
        for (int x = 0;x < depth;x++)
            bestnet.propagate();
        cout << "Input [ " << inputs[pattern][0] << " " << inputs[pattern][1] << " ] : Output " << bestnet.outputs[0]->activation << endl;
    
    }
    cout << &bestnet << endl;
}




