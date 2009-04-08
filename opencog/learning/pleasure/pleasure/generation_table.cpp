#include "generation_table.hpp"

namespace pleasure
{
void generate_generation_table(node_list& nlist, combo::type_tree& ttree, generation_table& gtable)
{
    std::vector<combo::type_tree> type_node_list;
    std::vector<node_list> type_lists;
    combo::argument_type_list atlist = combo::type_tree_input_arg_types(ttree);
    for (std::set<combo::vertex>::iterator it = nlist.begin(); it != nlist.end(); it++) {
        generation_node gnode;
        gnode.node = (*it);
        gtable.push_back(gnode);
        combo::type_tree outttree = combo::get_output_type_tree(*it);
        int temp;
        for (temp = 0; (temp != (int)type_node_list.size()) and (!combo::equal_type_tree(type_node_list[temp], outttree)); temp++);
        if (temp == (int)type_node_list.size()) {
            type_node_list.push_back(outttree);
            node_list tempnlist;
            tempnlist.insert(*it);
            type_lists.push_back(tempnlist);
        } else {
            type_lists[temp].insert(*it);
        }
    }
    for (int i = 0; i < (int)atlist.size(); i++) {
        combo::argument arg(i + 1);
        generation_node gnode;
        gnode.node = arg;
        gtable.push_back(gnode);
        int temp;
        for (temp = 0; (temp != (int)type_node_list.size()) and (!combo::equal_type_tree(atlist[i], type_node_list[temp])); temp++);
        if (temp == (int)type_node_list.size()) {
            printf("Error. No candidates possible for given signature");
            exit(1);
        } else {
            type_lists[temp].insert(arg);
        }
    }
    for (std::vector<generation_node>::iterator it = gtable.begin(); it != gtable.end(); it++) {
        combo::arity_t temp = combo::get_arity(it->node);
        bool has_arg_list = false;
        if (temp < 0) {
            has_arg_list = true;
            temp *= -1;
        }
        for (combo::arity_t i = 0; i < temp; i++) {
            combo::type_tree inttree = combo::get_input_type_tree(it->node, i);
            for (int it2 = 0; it2 != (int)type_node_list.size(); it2++) {
                if (combo::equal_type_tree(type_node_list[it2], inttree))
                    it->glist.push_back(type_lists[it2]);
            }
        }
        if (has_arg_list) {
            it->glist.push_back(it->glist.back());
        }
    }
}
}
