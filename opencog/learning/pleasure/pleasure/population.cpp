#include "population.hpp"

namespace pleasure
{
void reduce(population& pop, combo::arity_t arg_num, const reduct::rule& reduction_rule)
{
    for (population::iterator it = pop.begin(); it != pop.end();) {
        reduction_rule(*it);
        if (!combo::does_contain_all_arg_up_to(*it, arg_num))
            it = pop.erase(it);
        else
            it++;
    }
    erase_duplicate(pop);
}

void erase_duplicate(population& pop)
{
    pop.sort(opencog::size_tree_order<combo::vertex>());
    pop.unique();
}
}
