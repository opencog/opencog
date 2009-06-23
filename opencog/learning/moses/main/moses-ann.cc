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

int main(int argc, char** argv)
{

    //set flag to print only cassert and other ERROR level logs on stdout
    opencog::logger().setPrintErrorLevelStdout();

    combo_tree tr;
    combo_tree::iterator it, top, n1, n2, n3, n4;

    it = tr.begin();
    top = tr.insert(it, id::ann);
    n1 = tr.append_child(top, id::ann_node);
    n2 = tr.append_child(n1, id::ann_node);
    n3 = tr.append_child(n1, id::ann_node);
    tr.append_child(n1, 0.3);
    tr.append_child(n1, 0.4);

    combo_tree subtree;
    n4 = tr.insert(it, id::ann_input);

    tr.insert_subtree(n2.begin(), n4);
    tr.insert(n2.end(), 0.9);

    tr.insert_subtree(n3.begin(), n4);
    tr.insert(n3.end(), 0.75);

    //cin >> input_tree; //"ann(ann_node(ann_input rand))";

    tree_transform tt;
    ann nn = tt.decodify_tree(tr);
    cout << tr << endl;
    cout << "Network depth: " << nn.feedforward_depth() << endl;
    cout << &nn << endl;
}




