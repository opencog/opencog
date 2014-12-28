#include "generation_table.hpp"

namespace pleasure {

using namespace opencog;

    bool type_registered(combo::type_tree ttree, generation_table& gtable);

    void generate_generation_table(node_list& nlist, combo::type_tree& ttree, generation_table& gtable) {
        std::vector<combo::type_tree> type_node_list;
        std::vector<node_list> type_lists;
        combo::type_tree_seq atlist = combo::get_signature_inputs(ttree);
        for (std::set<combo::vertex>::iterator it = nlist.begin(); it != nlist.end(); ++it) {
            generation_node gnode;
            combo::type_tree outttree = combo::get_output_type_tree(*it);
            gnode.node = outttree;
            if (!type_registered(outttree, gtable))
                gtable.push_back(gnode);
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
            int temp;
            for (temp = 0; (temp != (int)type_node_list.size()) and (!combo::equal_type_tree(atlist[i], type_node_list[temp])); temp++);
            if (temp == (int)type_node_list.size()) {
                printf("Error. No candidates possible for given signature");
                exit(1);
            } else {
                type_lists[temp].insert(arg);
            }
        }
        for (std::vector<generation_node>::iterator it = gtable.begin(); it != gtable.end(); ++it) {
            for (int it2 = 0; it2 != (int)type_node_list.size(); it2++) {
                if (combo::equal_type_tree(type_node_list[it2], it->node))
                    it->glist = type_lists[it2];
            }
        }
    }
    
    bool type_registered(combo::type_tree ttree, generation_table& gtable) {
        for (generation_table::iterator it = gtable.begin(); it != gtable.end(); ++it) {
            if (combo::equal_type_tree(it->node, ttree))
                return true;
        }
        return false;
    }
}
