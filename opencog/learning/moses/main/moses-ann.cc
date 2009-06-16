#include <opencog/comboreduct/combo/eval.h>

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

#include <iostream>
#include <opencog/learning/moses/moses/ann_scoring.h>
using namespace std;

int main(int argc,char** argv) { 

    //set flag to print only cassert and other ERROR level logs on stdout
    opencog::logger().setPrintErrorLevelStdout();

    combo_tree input_tree("ann(ann_node(ann_input rand))");
    tree_transform tt;
    ann* nn = tt.decodify_tree(input_tree);
    cout << input_tree << endl; 

}




