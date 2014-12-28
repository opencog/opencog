#include "population.hpp"

namespace pleasure {
using namespace opencog;

    static int period = 0;

    void erase_duplicate(population& pop);

    void reduce(population& pop, const reduct::rule& reduction_rule) {
        for (population::iterator it = pop.begin(); it != pop.end(); ++it) {
            reduction_rule(*it);
        }
        erase_duplicate(pop);
    }
    void reduced_insertion(population& pop, combo::combo_tree new_tree, const reduct::rule& reduction_rule) {
        period++;
        reduction_rule(new_tree);
        pop.push_back(new_tree);
        if (period > 1000) {
            erase_duplicate(pop);
            period = 0;
        }
    }

    void erase_duplicate(population& pop) {
        pop.sort(opencog::size_tree_order<combo::vertex>());
        pop.unique();
    }
}
