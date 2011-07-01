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
    int rand_seed;
    float partial_probility = 1.0;

    combo_tree combo_tr;
    type_tree tr;
    argument_type_list arg_type_list;

    try {
        if ( (argc != 2) && (argc != 3 ))
            throw "foo";
        rand_seed = lexical_cast<int>(argv[1]);
        if (argc == 3)
            partial_probility = lexical_cast<float>(argv[2]);
    } catch (...) {
        cout << "Description:" << endl;
        cout << "\tgen-truth-table can generate the complete or partial table with"
             << " the different options" << endl;
        cout << "Usage:" << endl;
        cout << "\t\t" << "1. complete table: " << argv[0] << " rand_seed " << endl;
        cout << "\t\t" << "2. partial  table: " << argv[0] << " rand_seed  partial_probability" << endl;
        exit(1);
    }

    while (cin.good()) {
        cout << "Please input the combo program :" << endl;
        // get the combo_tree from the stream
        cin >> combo_tr;
        if (!cin.good())
            break;

        // infer the tree type and get the argument_type_list
        tr = infer_type_tree(combo_tr);
        arg_type_list = type_tree_input_arg_types(tr);

        // generate the RandNumber for the given the max_randvalue and min_randvalue
        MT19937RandGen rng(rand_seed);
        MT19937RandGen output_rng(rand_seed);

        size_t arg_number = arg_type_list.size();
        size_t sample_number = pow(2.0f, (int)arg_number);

        cout << "The number of args:" << arg_number << endl;
        try {
            //  generate the entire truth-table
            truth_table truthtable(combo_tr);

            // output the truth-table depend on the partial_probility
            for ( size_t i = 0 ; i < sample_number; ++i) {

                bool is_output = biased_randbool(partial_probility, rng);

                if (is_output) {
                    for ( size_t j = 0; j < arg_number; ++j) {
                        cout << ( (i >> j) % 2 ) << "\t";
                    }
                    std::cout << static_cast<int>(truthtable[i]) << endl;
                }
            }
        } catch (...) {
            cout << "an exception has been raised perhaps you should try again " << endl;
            exit(1);
        }
    }

    return 0;
}
