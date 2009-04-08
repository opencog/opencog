#include "generation.hpp"

#include <iostream>
using namespace std;

namespace pleasure
{
void increase_tree_depth(generation_table& gtable, population& pop, int& from_depth, combo::arity_t& needed_arg_count);

void enumerate_program_trees(generation_table& gtable, int depth, combo::type_tree& ttree, population& pop)
{
    pop.clear();
    for (std::vector<generation_node>::iterator it = gtable.begin(); it != gtable.end(); it++) {
        if (combo::equal_type_tree(combo::get_output_type_tree(it->node), combo::type_tree_output_type_tree(ttree)))
            pop.push_back(combo::combo_tree(it->node));
    }
    combo::arity_t needed_arg_count = combo::type_tree_arity(ttree);
    for (int i = 1; i < depth; i++) {
        increase_tree_depth(gtable, pop, i, needed_arg_count);
    }
    for (population::iterator it = pop.begin(); it != pop.end();) {
        bool erased = false;
        for (combo::combo_tree::leaf_iterator lit = it->begin_leaf(); lit != it->end_leaf(); lit++) {
            if (get_arity(*lit) != 0 && !combo::is_argument(*lit)) {
                erased = true;
                break;
            }
        }
        if (!combo::does_contain_all_arg_up_to(*it, needed_arg_count)) {
            erased = true;
        }
        if (erased)
            it = pop.erase(it);
        else
            it++;
    }
}

void increase_tree_depth(generation_table& gtable, population& pop, int& from_depth, combo::arity_t& needed_arg_count)
{
    for (population::iterator it = pop.begin(); it != pop.end(); it++) {
        int mdepth = 0;
        for (combo::combo_tree::leaf_iterator lit = it->begin_leaf(); lit != it->end_leaf(); lit++) {
            mdepth = std::max(mdepth, it->depth(lit));
        }
        if (mdepth < (from_depth - 1))
            continue;
        int number_of_combinations = 1;
        bool can_have_leaves = false;
        std::vector< std::vector<std::vector<node_list>::iterator> > assignment;
        std::vector<combo::combo_tree::leaf_iterator> leaves;
        for (combo::combo_tree::leaf_iterator lit = it->begin_leaf(); lit != it->end_leaf(); lit++) {
            assignment.push_back(std::vector<std::vector<node_list>::iterator>());
            for (std::vector<generation_node>::iterator it2 = gtable.begin(); it2 != gtable.end(); it2++)
                if (it2->node == *lit) {
                    if (it2->glist.size() != 0)
                        can_have_leaves = true;
                    for (std::vector<node_list>::iterator it3 = it2->glist.begin(); it3 != it2->glist.end(); it3++) {
                        assignment.back().push_back(it3);
                        number_of_combinations *= it3->size();
                    }
                    break;
                }
        }
        if (!can_have_leaves)
            number_of_combinations = 0;
        for (int i = 0; i < number_of_combinations; i++) {
            combo::combo_tree temp_tree(*it);
            int ongoing_count = 1;
            leaves.clear();
            for (combo::combo_tree::leaf_iterator lit = temp_tree.begin_leaf(); lit != temp_tree.end_leaf(); lit++)
                leaves.push_back(lit);
            std::vector< std::vector<std::vector<node_list>::iterator> >::iterator it2 = assignment.begin();
            for (std::vector<combo::combo_tree::leaf_iterator>::iterator lit = leaves.begin(); lit != leaves.end(); lit++) {
                if (it2->size() != 0) {
                    for (std::vector<std::vector<node_list>::iterator>::iterator it3 = it2->begin(); it3 != it2->end(); it3++) {
                        node_list::iterator nlit = (**it3).begin();
                        for (int n = 0; n < (i / (number_of_combinations / (ongoing_count * (int)(**it3).size())) % (int)(**it3).size()); n++)
                            nlit++;
                        temp_tree.append_child(*lit, *nlit);
                        ongoing_count *= (**it3).size();
                    }
                }
                it2++;
            }
            bool erased = true;
            for (combo::combo_tree::leaf_iterator lit = temp_tree.begin_leaf(); lit != temp_tree.end_leaf(); lit++) {
                if (get_arity(*lit) != 0 && !combo::is_argument(*lit)) {
                    erased = false;
                    break;
                }
            }
            if (combo::does_contain_all_arg_up_to(temp_tree, needed_arg_count)) {
                erased = false;
            }
            if (!erased)
                pop.push_front(temp_tree);
        }
    }
}
}
