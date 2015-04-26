#include "opencog/comboreduct/combo/procedure_repository.h"
#include "opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

#include "../pleasure/population.hpp"
#include "../pleasure/generation.hpp"
#include "../pleasure/node_list.hpp"
#include "../pleasure/generation_table.hpp"

#include <fstream>
#include <iostream>
#include <ctime>

using namespace std;
using namespace pleasure;
using namespace opencog;
using namespace opencog::combo;

int main(int argc, char* argv[])
{
    procedure_repository repo;
    std::ifstream file("procedures.combo");
    int n = load_procedure_repository<ant_builtin_action, ant_perception, ant_action_symbol, ant_indefinite_object>
        (file, repo, false);
    cout << n << " procedures loaded" << endl;
    repo.infer_types_repo();

    time_t start_time = time(NULL);
    population pop;
    node_list nlist;
    std::ifstream in("nodelist");

    if (!in) {
        std::cout << "Error in nodelist file (empty or missing)" << std::endl;
    }

    while (!in.eof())
    {
        //stream_to_node_list(in, nlist);
        std::string str;
        std::getline(in, str);
        if (str.size()) {
            procedure_call proc_p = repo.instance(str);
            if (proc_p) {
                cout << "function " << proc_p->get_name() << endl;
                nlist.insert(vertex(proc_p));
            }
            else {
                cout << "builtin " << str << endl;
                nlist.insert(combo::str_to_vertex<combo::ant_builtin_action, combo::ant_perception, combo::ant_action_symbol, combo::ant_indefinite_object>(str));
            }
        }
        else
            // the eof() method doesn't work with getline
            break;
    }
    
    generation_table gtable;
    combo::type_tree ttree(combo::id::lambda_type);
    combo::type_tree::post_order_iterator it = ttree.begin_post();
    //ttree.append_child(it, combo::id::contin_type);
    //ttree.append_child(list_it, combo::id::contin_type);
    ttree.append_child(it, combo::id::contin_type);
    //ttree.append_child(it, combo::id::contin_type);
    //ttree.append_child(it, combo::id::boolean_type);
    //ttree.append_child(it, combo::id::boolean_type);
    ttree.append_child(it, combo::id::contin_type);
    ttree.append_child(it, combo::id::contin_type);
    ttree.append_child(it, combo::id::contin_type);
    //ttree.append_child(it, combo::id::list_type);
    std::cout << ttree;
    generate_generation_table(nlist, ttree, gtable);

    //const combo::vertex_set ignore_ops; // empty
    //reduct::logical_reduction logic_red(ignore_ops);
    // logic_red(1) gets you the actual reduct::rule& instance

    enumerate_program_trees(gtable, 3, ttree, pop, reduct::mixed_reduction());

    pleasure::reduce(pop, reduct::mixed_reduction());
    
    for (population::iterator it = pop.begin(); it != pop.end(); ++it) {
        std::cout << *it << std::endl;
    }
    std::cout << "Running time: " << difftime(time(NULL), start_time) << std::endl;
    return 0;
};
