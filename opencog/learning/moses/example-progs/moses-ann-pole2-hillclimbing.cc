#include <iostream>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>
#include <opencog/comboreduct/interpreter/eval.h>

#include "../moses/moses_main.h"
#include "../optimization/optimization.h"
#include "../scoring/scoring.h"

#include "pole_scoring.h"


using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;
using namespace opencog;


int main(int argc, char** argv)
{

    //set flag to print only cassert and other ERROR level logs on stdout
    logger().setPrintErrorLevelStdout();

    combo_tree tr;
    cin >> tr; 

    tree_transform trans;
    ann nn = trans.decodify_tree(tr);
    cout << tr << endl;
    cout << "Network depth: " << nn.feedforward_depth() << endl;
    cout << &nn << endl;

    randGen().seed(0);

    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(), id::ann_type, 1);


    //DOUBLE MARKOVIAN POLE TASK`
    ann_pole2_cscore p2_cscore;
    ann_pole2_bscore p2_bscore;

    hill_climbing hc;
    metapopulation metapop_pole2(tr, tt, clean_reduction(),
                      p2_cscore, p2_bscore, hc);

    moses_parameters pa;
    moses_statistics st;
    run_moses(metapop_pole2, pa, st);

    combo_tree best = metapop_pole2.best_tree();
    ann bestnet = trans.decodify_tree(best);
    
    cout << "Best network: " << endl;
    cout << &bestnet << endl;
    bestnet.write_dot("best_nn.dot");    

    //for parameter sweet
    cout << metapop_pole2.best_score() << endl;
}




