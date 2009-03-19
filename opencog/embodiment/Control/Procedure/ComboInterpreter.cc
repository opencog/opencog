#include "util/Logger.h"
#include "util/exceptions.h"
#include "util/lazy_random_selector.h"
#include "util/lazy_normal_selector.h"

#include "ComboInterpreter.h"
#include "NetworkElement.h"
#include "StringMessage.h"
#include <boost/lexical_cast.hpp>

namespace Procedure {
  using namespace MessagingSystem;
  using namespace boost;
  using namespace std;
  
  ComboInterpreter::ComboInterpreter(PerceptionActionInterface::PAI& p, opencog::RandGen& _rng) : rng(_rng), _ww(new WorldWrapper::PAIWorldWrapper(p,_rng)), _next(0) { 
  }

  ComboInterpreter::ComboInterpreter(VirtualWorldData::VirtualWorldState& v, opencog::RandGen& _rng) : rng(_rng), _ww(new WorldWrapper::RuleValidationWorldWrapper(v)), _next(0) {
  }

  ComboInterpreter::~ComboInterpreter() { 
  }

  void ComboInterpreter::run(NetworkElement *ne) { 
      if (_vec.empty())
          return;

      if (_vec.size() > 1) {
          logger().log(opencog::Logger::WARN, "Got multiple (%d) running procedures in ComboInterpreter!", _vec.size());
      }
      
      std::set<RunningProcedureId> done;

      opencog::lazy_selector* sel; 
      if (NetworkElement::parameters.get("AUTOMATED_SYSTEM_TESTS") != "1") {
          //loop in random order until we find a running procedure that's ready 
          //along the way, get rid of done procedures
          sel = new opencog::lazy_random_selector(_vec.size(), rng); 
      } else {
          sel = new opencog::lazy_normal_selector(_vec.size()); 
      }

      while (!sel->empty()) {
          
          Vec::iterator it=_vec.begin()+(*sel)();
          RunningComboProcedure& rp=(*it)->second;
          
          if (rp.isReady()) {
              logger().log(opencog::Logger::DEBUG, "Running procedure id '%d'.",  ((RunningProcedureId&) (*it)->first).getId());
              rp.cycle();

          } else if (rp.isFinished()) {
              logger().log(opencog::Logger::DEBUG, "Done procedure id '%d'.", ((RunningProcedureId&) (*it)->first).getId());
              done.insert((*it)->first);
          }
      }
      delete sel;

      Vec::iterator last =_vec.end();
      if(!done.empty()){
          last = std::partition(_vec.begin(), _vec.end(), DonePred(done));
      }

      for (Vec::iterator it=last;it!=_vec.end();++it) {
          RunningComboProcedure& rp=(*it)->second;
          if (!rp.isFinished()) {
              // Should be finished here. If not, record failure for querying and 
              // send it to combo shell, if any
              _failed.insert((*it)->first);
             
              logger().log(opencog::Logger::ERROR, "Not finished '%d' - adding to failed list",  ((RunningProcedureId&) (*it)->first).getId());
              if (ne) {
                  StringMessage msg(ne->getID(),
                            NetworkElement::parameters.get("COMBO_SHELL_ID"),"action_failure");
                  ne->sendMessage(msg);
              }

          } else if (rp.getResult()!=combo::id::null_vertex) { //stopped?
              if (!ne) {
                  if (rp.isFailed()) {
                     
                      _failed.insert((*it)->first); //record failures for querying
                  } else {
                      _resultMap.insert(make_pair((*it)->first, rp.getResult()));
                      _unifierResultMap.insert(make_pair((*it)->first, rp.getUnifierResult()));
                  }                  

              } else { //do the same and also send a message recording the result

                  if (rp.isFailed()) {
                      _failed.insert((*it)->first);
                      StringMessage msg(ne->getID(),
                                NetworkElement::parameters.get("COMBO_SHELL_ID"),"action_failure");
                      ne->sendMessage(msg);

                  } else {
                      _resultMap.insert(make_pair((*it)->first, rp.getResult()));
                      _unifierResultMap.insert(make_pair((*it)->first, rp.getUnifierResult()));
                      stringstream ss;
                      ss << rp.getResult();
                      StringMessage msg(ne->getID(),
                                    NetworkElement::parameters.get("COMBO_SHELL_ID"),ss.str());
                      ne->sendMessage(msg);
                  }
              }
          }
          _map.erase(*it);
      }
      _vec.erase(last,_vec.end());
  }

  //add a procedure to be run by the interpreter
  RunningProcedureId ComboInterpreter::runProcedure(const combo::combo_tree& tr, const std::vector<combo::vertex>& arguments) {
    RunningProcedureId id(++_next, COMBO);  
    _vec.push_back(_map.insert(make_pair(id, RunningComboProcedure(*_ww,tr,rng,arguments))).first);
    return id;
  }
  
  //add a procedure to be run by the interpreter
  RunningProcedureId ComboInterpreter::runProcedure(const combo::combo_tree& tr, const std::vector<combo::vertex>& arguments, combo::variable_unifier& vu) {
    RunningProcedureId id(++_next, COMBO);  
    _vec.push_back(_map.insert(make_pair(id, RunningComboProcedure(*_ww,tr,rng,arguments,true,vu))).first);
    return id;
  }
  
  bool ComboInterpreter::isFinished(RunningProcedureId id) { 
    Map::iterator it=_map.find(id);
    return (it==_map.end() || it->second.isFinished());
  }

  // Note: this will return false if the stopProcedure() method was previously called for this same procedure id, 
  // even if the procedure execution has failed before
  bool ComboInterpreter::isFailed(RunningProcedureId id) {
    if (_failed.find(id)!=_failed.end()){
      return true;
    }
    Map::iterator it=_map.find(id);

//    logger().log(opencog::Logger::WARN, "_map!end '%s', finished '%s', failed '%s'.",
//                    (it!=_map.end())?"true":"false", it->second.isFinished()?"true":"false", it->second.isFailed()?"true":"false");
    return (it!=_map.end() && it->second.isFinished() && it->second.isFailed());
  }

  // Get the result of the procedure with the given id
  // Can be called only if the following conditions are true:
  // - procedure execution is finished (checked by isFinished() method)
  // - procedure execution has not failed (checked by isFailed() method)
  // - procedure execution was not stopped (by calling stopProcedure() method) 
  combo::vertex ComboInterpreter::getResult(RunningProcedureId id) {
    opencog::cassert(TRACE_INFO, isFinished(id), "ComboInterpreter - Procedure '%d' not finished.", id.getId());
    opencog::cassert(TRACE_INFO, !isFailed(id), "ComboInterpreter - Procedure '%d' failed.", id.getId());
    
    ResultMap::iterator it = _resultMap.find(id);

    if (it==_resultMap.end()) {
      Map::iterator mi=_map.find(id);
      opencog::cassert(TRACE_INFO, mi!=_map.end(),"ComboInterpreter - Unable to find procedure '%d' in _map.", id.getId());
      return mi->second.getResult();
    }
    return it->second; 
  }
  
  combo::variable_unifier& ComboInterpreter::getUnifierResult(RunningProcedureId id){
	  opencog::cassert(TRACE_INFO, isFinished(id), "ComboInterpreter - Procedure '%d' not finished.", id.getId());
	  opencog::cassert(TRACE_INFO, !isFailed(id), "ComboInterpreter - Procedure '%d' failed.", id.getId());
	  
	  UnifierResultMap::iterator it = _unifierResultMap.find(id);
	  if(it == _unifierResultMap.end()){
		  Map::iterator m_it = _map.find(id);
		  opencog::cassert(TRACE_INFO, m_it!=_map.end(),"ComboInterpreter - Unable to find procedure '%d' in _map.", id.getId());
		  return m_it->second.getUnifierResult();
	  }
	  return it->second;
  }
    
  // makes the procedure with the given id to stop and remove it from the interpreter
  void ComboInterpreter::stopProcedure(RunningProcedureId id) {
    Map::iterator it=_map.find(id);
    if (it!=_map.end()) {
      it->second.stop(); //stop in the middle
    } else {
      Set::iterator failedIt = _failed.find(id);
      if (failedIt != _failed.end()) {
	_failed.erase(failedIt);
      }
      ResultMap::iterator result_it = _resultMap.find(id);
      if (result_it != _resultMap.end()) {
	_resultMap.erase(result_it);
      }      
      UnifierResultMap::iterator uresult_it = _unifierResultMap.find(id);
      if (uresult_it != _unifierResultMap.end()) {
	_unifierResultMap.erase(uresult_it);
      }
    }
  }

} //~namespace Procedure
