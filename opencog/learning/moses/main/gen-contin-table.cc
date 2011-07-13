#include <iostream>

#include <boost/lexical_cast.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/random.h>

#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>
#include <opencog/comboreduct/combo/type_tree.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>

using namespace std;
using namespace boost;
using namespace opencog;
using namespace combo;
using namespace ant_combo;

int main(int argc, char ** argv)
{
    int rand_seed, nsamples;
    double  max_randvalue, min_randvalue, variance;
    combo_tree combo_tr;
    type_tree tr;
    argument_type_list arg_type_list;
    try {
        if (argc != 6)
            throw "foo";
        rand_seed = lexical_cast<int>(argv[1]);
        max_randvalue = lexical_cast<double>(argv[2]);
        min_randvalue = lexical_cast<double>(argv[3]);
        nsamples = lexical_cast<int>(argv[4]);
        variance = lexical_cast<double>(argv[5]);
    } catch (...) {
        cerr << "stdin a combo program and it will stdout the contin-table" << endl;
        cerr << "Usage:" << argv[0] << " rand_seed max_randvalue min_randvalue nsamples variance" << endl;
        cerr << "variance corresponds to a noise of the output" << endl;
        exit(1);
    }

    while (cin.good()) {
        // get the combo_tree from the stream
        cin >> combo_tr;
        if (!cin.good())
            break;

        // infer the tree type and get the argument_type_list
        tr = infer_type_tree(combo_tr);
        arg_type_list = type_tree_input_arg_types(tr);

        // generate the RandNumber for the given the max_randvalue and min_randvalue
        MT19937RandGen rng(rand_seed);
        size_t arg_number = arg_type_list.size();
        contin_input_table cti(nsamples, arg_number, rng, max_randvalue, min_randvalue);

        try {
            contin_output_table contintable(combo_tr, cti, rng);

            // output the contin-table
            int k = 0;
            for (const_cm_it i = cti.begin(); i != cti.end(); ++i, ++ k) {
                for (const_cv_it j = (*i).begin(); j != (*i).end(); ++j) {
                    cout << (*j) << " ";
                }
                cout << gaussian_rand(std::sqrt(variance), contintable[k], rng)
                     << endl;
            }
        } catch (...) {
            cout << "an exception has been raised perhaps you should try"
                " again with a different min_randvalue or max_rand value" << endl;
            exit(1);
        }
    }
    return 0;
}
