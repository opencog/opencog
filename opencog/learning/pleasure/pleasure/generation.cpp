#include "generation.hpp"

namespace pleasure {

using namespace opencog;

    void increase_tree_depth(generation_table& gtable, population& pop, int& from_depth, combo::arity_t& needed_arg_count, int fill_from_arg, const reduct::rule& reduction_rule);
    
    void fill_leaves(population& pop, int& from_arg);
    
    void fill_leaves_single(combo::combo_tree& new_tree, int from_arg);

    void enumerate_program_trees(generation_table& gtable, int depth, combo::type_tree& ttree, population& pop, const reduct::rule& reduction_rule) {
        pop.clear();
        // For each generation node with the right return-type, add it to the pop
        for (std::vector<generation_node>::iterator it = gtable.begin(); it != gtable.end(); ++it) {
            if (combo::equal_type_tree(it->node, combo::get_signature_output(ttree))) {
                for (node_list::iterator it2 = it->glist.begin(); it2 != it->glist.end(); it2++)
                    pop.push_back(combo::combo_tree(*it2));
                break;
            }
        }
        // add the right number of arguments
        int from_arg = combo::get_signature_inputs(ttree).size();
        combo::arity_t needed_arg_count = combo::type_tree_arity(ttree);
        std::cout << ttree << " " << needed_arg_count << std::endl;
        for (int i = 1; i < depth; i++) {
            fill_leaves(pop, from_arg);
            reduce(pop, reduction_rule);
            increase_tree_depth(gtable, pop, i, needed_arg_count, from_arg, reduction_rule);
        }
        for (population::iterator it = pop.begin(); it != pop.end();) {
            bool erased = false;
            for (combo::combo_tree::leaf_iterator lit = it->begin_leaf(); lit != it->end_leaf(); ++lit) {
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
                ++it;
        }
    }

    void increase_tree_depth(generation_table& gtable, population& pop, int& from_depth, combo::arity_t& needed_arg_count, int fill_from_arg, const reduct::rule& reduction_rule) {
        for (population::iterator it = pop.begin(); it != pop.end(); ++it) {
            int number_of_combinations = 1;
            bool can_have_leaves = false;
            std::vector<generation_table::iterator> assignment;
            std::vector<combo::combo_tree::leaf_iterator> leaves;
            for (combo::combo_tree::leaf_iterator lit = it->begin_leaf(); lit != it->end_leaf(); ++lit) {
                if (combo::is_argument(*lit))
                    if (combo::get_argument(*lit).abs_idx() <= needed_arg_count)
                        continue;
                for (std::vector<generation_node>::iterator it2 = gtable.begin(); it2 != gtable.end(); it2++)
                    if (combo::equal_type_tree(it2->node, combo::infer_vertex_type(*it, lit))) {
                        if (it2->glist.size() != 0)
                            can_have_leaves = true;
                        assignment.push_back(it2);
                        number_of_combinations *= it2->glist.size();                        
                        break;
                    }
            }
            if (!can_have_leaves)
                number_of_combinations = 0;
            for (int i = 0; i < number_of_combinations; i++) {
                combo::combo_tree temp_tree(*it);
                int ongoing_count = 1;
                leaves.clear();
                for (combo::combo_tree::leaf_iterator lit = temp_tree.begin_leaf(); lit != temp_tree.end_leaf(); ++lit)
                    leaves.push_back(lit);
                std::vector<generation_table::iterator>::iterator it2 = assignment.begin();
                for (std::vector<combo::combo_tree::leaf_iterator>::iterator lit = leaves.begin(); lit != leaves.end(); ++lit) {
                    if (combo::is_argument(**lit))
                        if (combo::get_argument(**lit).abs_idx() <= needed_arg_count)
                           continue;
                    if ((*it2)->glist.size() != 0) {
                        node_list::iterator it3 = (*it2)->glist.begin();
                        for (int n = 0; n < (int)(i/(number_of_combinations/(ongoing_count * (*it2)->glist.size())) % (*it2)->glist.size()); n++)
                            it3++;
                        temp_tree.replace(*lit, *it3);
                        ongoing_count *= (*it2)->glist.size();
                    }
                    it2++;
                }
                bool erased = true;
                for (combo::combo_tree::leaf_iterator lit = temp_tree.begin_leaf(); lit != temp_tree.end_leaf(); ++lit) {
                    if (combo::get_arity(*lit) != 0 && !combo::is_argument(*lit)) {
                        erased = false;
                        break;
                    }
                }
                if (combo::does_contain_all_arg_up_to(temp_tree, needed_arg_count)) {
                    erased = false;
                }
                if (!erased) {
                    //Uses less memory but is very slow
                    /*fill_leaves_single(temp_tree, fill_from_arg);
                    reduced_insertion(new_pop, temp_tree, reduction_rule);*/
                    pop.push_front(temp_tree);
                }
            }
        }
    }
    
    void fill_leaves(population& pop, int& from_arg) {
        for (population::iterator it = pop.begin(); it != pop.end(); ++it) {
            int local_from_arg = from_arg;
            for (combo::combo_tree::leaf_iterator lit = it->begin_leaf(); lit != it->end_leaf(); ++lit) {
                if (!combo::is_argument(*lit)) {
                    combo::arity_t number_of_leaves = combo::get_arity(*lit);
                    if (number_of_leaves < 0) number_of_leaves = -1 * number_of_leaves + 1;
                    for (combo::arity_t count = 0; count < number_of_leaves; count++) {
                        (*it).append_child(lit, combo::argument(++local_from_arg));
                    }
                }
            }
        }
    }
    
    void fill_leaves_single(combo::combo_tree& new_tree, int from_arg) {
       for (combo::combo_tree::leaf_iterator lit = new_tree.begin_leaf(); lit != new_tree.end_leaf(); ++lit) {
            if (!combo::is_argument(*lit)) {
                combo::arity_t number_of_leaves = combo::get_arity(*lit);
                if (number_of_leaves < 0) number_of_leaves = -1 * number_of_leaves + 1;
                for (combo::arity_t count = 0; count < number_of_leaves; count++) {
                    new_tree.append_child(lit, combo::argument(++from_arg));
                }
            }
        }
    }
    
}
