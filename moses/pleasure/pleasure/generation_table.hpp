#ifndef GENERATION_TABLE_
#define GENERATION_TABLE_

#include <vector>

#include <opencog/comboreduct/reduct/reduct.h>

#include "node_list.hpp"

namespace pleasure {
    struct generation_node {
        opencog::combo::type_tree node;
        node_list glist;
    };
    typedef std::vector<generation_node> generation_table;
    void generate_generation_table(node_list& nlist, opencog::combo::type_tree& ttree, generation_table& gtable);
}
#endif

