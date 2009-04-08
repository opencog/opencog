#include <list>

#include "comboreduct/combo/vertex.h"
#include "comboreduct/combo/type_tree.h"
#include "comboreduct/reduct/reduct.h"

namespace pleasure
{
typedef std::list<combo::combo_tree> population;

void reduce(population& pop, combo::arity_t arg_num, const reduct::rule& reduction_rule);

void erase_duplicate(population& pop);
}
