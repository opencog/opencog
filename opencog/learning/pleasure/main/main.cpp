#include "pleasure/population.hpp"
#include "pleasure/generation.hpp"
#include "pleasure/node_list.hpp"
#include "pleasure/generation_table.hpp"

#include <fstream>
#include <iostream>
#include <ctime>

using namespace pleasure;
using namespace opencog;

int main(int argc, char* argv[])
{
    time_t start_time = time(NULL);
    population pop;
    node_list nlist;
    std::ifstream in("nodelist");
    
    while (!in.eof())
    {
        stream_to_node_list(in, nlist);
    }
    
    generation_table gtable;
    combo::type_tree ttree(combo::id::lambda_type);
    combo::type_tree::post_order_iterator it = ttree.begin_post();
    ttree.append_child(it, combo::id::boolean_type);
    ttree.append_child(it, combo::id::boolean_type);
    ttree.append_child(it, combo::id::boolean_type);
    ttree.append_child(it, combo::id::boolean_type);
    ttree.append_child(it, combo::id::boolean_type);
    generate_generation_table(nlist, ttree, gtable);
    enumerate_program_trees(gtable, 3, ttree, pop, reduct::logical_reduction());
    pleasure::reduce(pop, reduct::logical_reduction());
    
    for (population::iterator it = pop.begin(); it != pop.end(); it++) {
        std::cout << *it << std::endl;
    }
    std::cout << "Running time: " << difftime(time(NULL), start_time) << std::endl;
    return 0;
};
