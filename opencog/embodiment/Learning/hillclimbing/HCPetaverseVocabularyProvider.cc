/** 
 * HCPetaverseVocabularyProvider.cc
 * 
 * Author(s):
 *    Nil Geisweiller
 * Created : Wed 19 Mar 2008
 */
#include "HCPetaverseVocabularyProvider.h"
#include "PetComboVocabulary.h"

namespace hillclimbing {

  using namespace PetCombo;

  HCPetaverseVocabularyProvider::HCPetaverseVocabularyProvider() {
    //elementary operators
    if(eo.empty()) {
      for(unsigned int i = 0; i < _elementary_operators_size; i++)
	eo.insert(_elementary_operators[i]);
    }
    //elementary builtin actions
    if(ea.empty()) {
      for(unsigned int i = 0; i < _elementary_actions_size; i++)
	ea.insert(instance(_elementary_actions[i]));
    }
    //elementary perceptions
    if(ep.empty()) {
      for(unsigned int i = 0; i < _elementary_perceptions_size; i++)
	ep.insert(instance(_elementary_perceptions[i]));
    }
    //indefinite objects
    if(is.empty()) {
      for(unsigned int i = 0; i < _indefinite_objects_size; i++)
	is.insert(instance(_indefinite_objects[i]));
    }
  }
  
  //return a reference of the set of operators
  const PetaverseVocabularyProviderBase::operator_set& 
  HCPetaverseVocabularyProvider::get_elementary_operators() const {
    return eo;
  }

  //return a reference of the set of actions
  const builtin_action_set& HCPetaverseVocabularyProvider::get_elementary_actions() const {
    return ea;
  }

  //return a reference of the set of perceptions
  const perception_set& HCPetaverseVocabularyProvider::get_elementary_perceptions() const {
    return ep;
  }
    
  //return a reference of the set of indefinite objects
  const PetaverseVocabularyProviderBase::indefinite_object_set&
  HCPetaverseVocabularyProvider::get_indefinite_objects() const {
    return is;
  }

}//~namespace hillclimbing
