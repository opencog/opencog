#ifndef GENERATION_TABLE_
#define GENERATION_TABLE_

#include <vector>

//#include <ComboReduct/combo/type_tree.h>
#include "comboreduct/reduct/reduct.h"

#include "node_list.hpp"

namespace pleasure
{
struct generation_node {
    combo::vertex node;
    std::vector<node_list> glist;
};
typedef std::vector<generation_node> generation_table;
void generate_generation_table(node_list& nlist, combo::type_tree& ttree, generation_table& gtable);
}
#endif

