/** 
 * ShellWorldWrapper.h
 * 
 * Author(s):
 *    Nil Geisweiller
 * Created : Fri Nov 30 2007
 */
#ifndef _SHELLWORLDWRAPPER_H
#define _SHELLWORLDWRAPPER_H

#include "WorldWrapper.h"

namespace WorldWrapper {

  /**
   * interactive shell world wrapper
   */

  class ShellWorldWrapper : public WorldWrapperBase {
    
  public:

    /**
     * Constructor, destructor
     */
    ShellWorldWrapper();
    ~ShellWorldWrapper();
    

    /**
     * return true is the current action plan is finished
     * false otherwise or if there is no action plan
     */
    bool isPlanFinished() const;
 
    /**
     * return true if the plan has failed
     * false otherwise
     * pre-condition : the plan is finished
     */
    bool isPlanFailed() const;
    
    /**
     * Send a sequence of sequential_and actions [from, to)
     * stdio the user to decide whether the plan has failed or not
     * return true iff the plan is actually executed
     */
    bool sendSequential_and(sib_it from, sib_it to);

    /**
     * evaluate a perception
     */
    combo::vertex evalPerception(pre_it per, 
    							 combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());
    
    /**
     * evaluate an indefinite object
     */
    combo::vertex evalIndefiniteObject(combo::indefinite_object io, 
    								   combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

  private:
    bool _isFailed;
    bool _isFinished;
  };

}//~namespace WorldWrapper

#endif
