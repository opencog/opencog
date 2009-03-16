#include "post_learning_rules.h"
#include "PetComboVocabulary.h"

namespace reduct {

  using namespace PetCombo;

  typedef combo_tree::sibling_iterator sib_it;
  typedef combo_tree::iterator pre_it;

  //add a drop action in front of a grab action
  void post_learning_drop_before_grab::operator()(combo_tree& tr,
						  combo_tree::iterator it) const {
    if(*it==id::sequential_exec || *it==id::sequential_or
       || *it==id::sequential_and) {
      for(sib_it sib = it.begin(); sib != it.end(); ++sib) {
	if(*sib==instance(id::grab)) {
	  sib_it pre_sib = tr.previous_sibling(sib);
	  if(tr.is_valid(pre_sib)) {
	    if(*pre_sib!=instance(id::drop))
	      sib = tr.insert(sib, instance(id::drop));
	  }
	  else sib = tr.insert(sib, instance(id::drop));
	}
      }
    }
  }

}//~namespace reduct
