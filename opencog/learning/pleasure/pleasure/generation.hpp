#include "generation_table.hpp"
#include "population.hpp"

namespace pleasure {
    void enumerate_program_trees(generation_table& gtable, int depth, opencog::combo::type_tree& ttree, population& pop, const reduct::rule& reduction_rule);
}
