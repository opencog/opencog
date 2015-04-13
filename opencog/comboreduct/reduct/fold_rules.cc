/*
 *opencog/comboreduct/reduct/fold_rules.cc
 */

#include "fold_rules.h"

namespace opencog { namespace reduct {

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::pre_order_iterator pre_it;

//fold unrolling:
//    foldl(f v list(a b c)) = f(f(f(v a) b) c)
//    foldr(f v list(a b c)) = f(a f(b f(c v)))
void fold_unrolling::operator()(combo_tree& tr,combo_tree::iterator it) const {

    if(*it==id::foldl) {
        combo_tree cb_tr(it);
        tr.erase_children(it);
        sib_it sib = cb_tr.begin().begin();    // *sib = f
        vertex f = *sib;  
        ++sib;    // *sib = v
        vertex v = *sib;
        ++sib;    // *sib = list(a b c)
        combo_tree lst_tr(sib);
        sib_it lst = lst_tr.begin();
        sib_it lst_it = lst.end();

        do{
            *it = f;
            tr.append_child(it, id::null_vertex);
            --lst_it;
            tr.append_child(it, *lst_it);
            it = it.begin();
        } while(lst_it != lst.begin());
        tr.replace(it, v);    //replace null_vertex with v
    }

    if(*it==id::foldr) {
        combo_tree cb_tr(it);
        tr.erase_children(it);
        sib_it sib = cb_tr.begin().begin();    // *sib = f
        vertex f = *sib; 
        ++sib;    // *sib = v
        vertex v = *sib;
        ++sib;    // *sib = list(a b c)
        combo_tree lst_tr(sib);
        sib_it lst = lst_tr.begin();

        for(sib_it lst_it = lst.begin(); lst_it != lst.end(); ++lst_it){
            *it = f;
            tr.append_child(it, *lst_it);
            tr.append_child(it, id::null_vertex);
            it = ++it.begin();
        }
        tr.replace(it, v);    //replace null_vertex with v
    }
}

} // ~namespace reduct
} // ~namespace opencog
