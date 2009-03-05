#include "ComboReduct/reduct/general_rules.h"
#include "ComboReduct/combo/eval.h"
#include "ComboReduct/combo/assumption.h"

namespace reduct {
  typedef combo_tree::sibling_iterator sib_it;

  //flattens all associative functions: f(a,f(b,c)) -> f(a,b,c)
  //note that level is recursive that is f(a,f(b,f(c,d))) -> f(a,b,c,d)
  void level::operator()(combo_tree& tr,combo_tree::iterator it) const {
    if (is_associative(*it))
      for (sib_it sib=it.begin();sib!=it.end();)
	if (*sib==*it)
	  sib=tr.erase(tr.flatten(sib));
	else
	  ++sib;
  }

  //evaluates sub-expressions when possible
  //if an operator is commutative, op(const,var,const) will become
  //op(op(const,const),var), e.g., +(2,x,1)->+(3,x)
  void eval_constants::operator()(combo_tree& tr,combo_tree::iterator it) const {
    if (it.is_childless()) {
      if (is_indefinite_object(*it)) //not sure we want that when indefinite_object is random
	*it = eval_throws(rng, it, evaluator);
      return;
    }
    sib_it to;
    if(is_associative(*it)) {
      if(is_commutative(*it)) {
	to=tr.partition(it.begin(),it.end(),is_constant<vertex>);
	int n_consts=distance(it.begin(),to);
	if (n_consts<2 && (!(n_consts==1 && it.has_one_child())))
	  return;
	if (to!=it.end()) {
	  tr.reparent(tr.append_child(it,*it),it.begin(),to);
	  it=it.last_child();
	}
      }
      else {
        LADSUtil::cassert(TRACE_INFO, false, "Not implemented yet");
      }
    }
    else {
      for (sib_it sib=it.begin();sib!=it.end();++sib)
	if (!is_constant(*sib))
	  return;	
    }
    *it=eval_throws(rng, it, evaluator);
    tr.erase_children(it);
  }

  //Reorder children of commutative operators (should be applied upwards)
  void reorder_commutative::operator()(combo_tree& tr,combo_tree::iterator it) const {
    if(is_commutative(*it))
      tr.sort_on_subtrees(it.begin(),it.end(),
			  LADSUtil::lexicographic_subtree_order<vertex>(),false);
  }

  //Get rid of subtrees marked with a null_vertex in their roots
  void remove_null_vertices::operator()(combo_tree& tr,combo_tree::iterator it) const {
    for (sib_it sib=it.begin();sib!=it.end();)
      if (*sib==id::null_vertex)
	sib=tr.erase(sib);
      else
	++sib;
  }

  void remove_all_assumptions::operator()(combo_tree& tr,combo_tree::iterator it) const {
    delete_all_assumptions(tr);
  }

} //~namespace reduct

