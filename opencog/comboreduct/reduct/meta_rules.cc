#include "ComboReduct/reduct/meta_rules.h"
#include <LADSUtil/foreach.h>
#include "ComboReduct/combo/assumption.h"

namespace reduct {

  void downwards::operator()(combo_tree& tr,combo_tree::iterator it) const {
    combo_tree::iterator end=it;  
    end.skip_children();
    ++end;

    static const type_tree unknown_type_tree = 
      type_tree(combo::id::unknown_type);

    if (input==unknown_type_tree)
      for(;it!=end;++it)
	(*r)(tr,it);
    else
      for(;it!=end;++it)
	if(//combo::get_argument_type_tree(*it, tr.sibling_index(it))==input
	   //&& 
	   combo::get_output_type_tree(*it)==type_tree(output))
	  (*r)(tr,it);
  }

  //apply rule from the leaves of the subtree rooted by it to it
  void upwards::operator()(combo_tree& tr,combo_tree::iterator it) const {
    combo_tree::post_order_iterator at=it,end=it;
    ++end;
    at.descend_all();

    for (;at!=end;++at)
      (*r)(tr,at);
  }

  void sequential::operator()(combo_tree& tr,combo_tree::iterator it) const {
    foreach (const rule& r,rules) {
      //std::cout << "SEQUENTIAL RULE : " << tr << "IT : " << *it << std::endl;
      r(tr,it);
    }
  }

  void iterative::operator()(combo_tree& tr,combo_tree::iterator it) const {
    combo_tree tmp;
    do {
      tmp=combo_tree(it);
      (*r)(tr,it);
    } while (!tr.equal_subtree(it,tmp.begin()));
  }

  void assum_iterative::operator()(combo_tree& tr,combo_tree::iterator it) const {
    combo_tree tmp;
    do {
      tmp = tr;
      (*r)(tr,it);
    } while(tr!=tmp || !equal_assumptions(tmp, tr));
  }

} //~namespace reduct

