#include <list>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/type_tree.h>
#include <opencog/comboreduct/reduct/reduct.h>


namespace pleasure {
    typedef std::list<opencog::combo::combo_tree> population;
    
    //void reduce(population& pop, combo::arity_t arg_num, const reduct::rule& reduction_rule);
    void reduce(population& pop, const reduct::rule& reduction_rule);
    
    void reduced_insertion(population& pop, opencog::combo::combo_tree new_tree, const reduct::rule& reduction_rule);
}
