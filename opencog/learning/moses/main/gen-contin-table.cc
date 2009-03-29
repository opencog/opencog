#include <iostream>

#include <boost/lexical_cast.hpp>

#include "comboreduct/combo/eval.h"
#include "comboreduct/combo/type_tree.h"
#include "comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

#include "util/mt19937ar.h"

using namespace std;
using namespace boost;
using namespace combo;
using namespace opencog;
using namespace ant_combo;


int main(int argc, char ** argv)
{
    int rand_seed, nsamples;
    double  max_randvalue, min_randvalue;
    combo_tree combo_tr;
    type_tree tr;
    argument_type_list arg_type_list;
    try {
        if (argc != 5)
            throw "foo";
        rand_seed = lexical_cast<int>(argv[1]);
        max_randvalue = lexical_cast<double>(argv[2]);
        min_randvalue = lexical_cast<double>(argv[3]);
        nsamples = lexical_cast<int>(argv[4]);
    } catch (...) {
        cerr << "Usage:" << argv[0] << " rand_seed max_randvalue min_randvalue nsamples" << endl;
        exit(1);
    }

    while (cin.good()) {
        // get the combo_tree from the stream
        cin >> combo_tr;   // compile error , is the operator from the  ant_combo_vocabulary? Should I write it for myself
        if (!cin.good())
            break;

        // infer the tree type and get the argument_type_list
        tr = infer_type_tree(combo_tr);
        arg_type_list = type_tree_input_arg_types(tr);

        // generate the RandNumber for the given the max_randvalue and min_randvalue
        MT19937RandGen rng(rand_seed);
        size_t arg_number = arg_type_list.size();
        RndNumTable rands(nsamples, arg_number, rng, max_randvalue, min_randvalue);

        contin_table contintable(combo_tr, rands, rng);

        // output the contin-table
        int k = 0;
        for (const_cm_it i = rands.begin(); i != rands.end(); ++i, ++ k) {
            for (const_cv_it j = (*i).begin(); j != (*i).end(); ++j) {
                cout << (*j) << " ";
            }
            cout << contintable[k] << endl;
        }
    }
    return 0;
}
