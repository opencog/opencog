/** 
 * ShellWorldWrapper.cc
 * 
 * Author(s):
 *    Moshe Looks
 *    Nil Geisweiller
 * Created : Fri Nov 30 2007
 */

#include "ShellWorldWrapper.h"
#include "PetComboVocabulary.h"

namespace WorldWrapper {

  using namespace PetCombo;

  /**
   * Constructor, destructor
   */
  ShellWorldWrapper::ShellWorldWrapper() : _isFailed(false),
					   _isFinished(true) {}

  ShellWorldWrapper::~ShellWorldWrapper() {}

  /**
   * public methods
   */

  bool ShellWorldWrapper::isPlanFinished() const {
    return _isFinished;
  }

  bool ShellWorldWrapper::isPlanFailed() const {
    return _isFailed;
  }

  bool ShellWorldWrapper::sendSequential_and(sib_it from, sib_it to) {
    using namespace combo;
    combo_tree tr(id::sequential_and);
    pre_it head = tr.begin();
    for(sib_it sib = from; sib != to; ++sib) {
      tr.replace(tr.append_child(head), sib);
    }
    std::cout << "What should the result of sending plan "
	      << tr << "be (action_success or action_failure)?"
	      << std::endl;
    std::cout << "> ";
    vertex v;
    std::cin >> v;
    _isFailed = v!=id::action_success;
    _isFinished = true;
    return true;
  }

  combo::vertex ShellWorldWrapper::evalPerception(pre_it per, combo::variable_unifier& vu) {
    using namespace combo;
    std::cout << "What should the result of " 
	      << combo::combo_tree(per) << "be (true or false)?" << std::endl;
    std::cout << "> ";
    vertex v;
    std::cin >> v;
    return v;
  }

  combo::vertex ShellWorldWrapper::evalIndefiniteObject(combo::indefinite_object io, combo::variable_unifier& vu) {
    using namespace combo;
    std::cout << "What should " << io << " return?" << std::endl;
    std::cout << "> ";
    vertex v;
    std::cin >> v;
    return v;
  }

}//~namespace WorldWrapper

