/** 
 * PetaverseVocabularyProviderBase.h
 * 
 * Abstract class to be inherited to use a learning algo in the context
 * of petaverse imitation learning, the inherited class will be used by
 * ImitationLearningTask and provides the methods needed by the EntropyFilter
 * and ActionFilter to determine the set of atomic actions and perceptions
 * involved in imitation learning. The operator set is still requiered to
 * compute sizePenalty function in the fitness estimator. It should contain
 * only non action, non perception, non indefinite_object operators.
 *
 * Author(s):
 *    Nil Geisweiller
 * Created : Wed 19 Mar 2008
 */
#ifndef _PETAVERSE_VOCABULARY_PROVIDER_BASE_H
#define _PETAVERSE_VOCABULARY_PROVIDER_BASE_H

#include "comboreduct/combo/vertex.h"
#include "PetComboVocabulary.h"

class PetaverseVocabularyProviderBase {

public:

  typedef std::set<combo::vertex> operator_set;
  typedef std::set<combo::indefinite_object> indefinite_object_set;

  //ctor, dtor

  PetaverseVocabularyProviderBase() {}
  virtual ~PetaverseVocabularyProviderBase() {}

  //access methods

  //return a reference of the set of operators
  virtual const operator_set& get_elementary_operators() const = 0;

  //return a reference of the set of actions
  virtual const combo::builtin_action_set& get_elementary_actions() const = 0;

  //return a reference of the set of perceptions
  virtual const combo::perception_set& get_elementary_perceptions() const = 0;

  //return a reference of the set of indefinite objects
  virtual const indefinite_object_set& get_indefinite_objects() const = 0;
};

#endif
