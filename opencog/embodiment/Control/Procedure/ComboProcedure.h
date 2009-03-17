#ifndef COMBOPROCEDURE_H_
#define COMBOPROCEDURE_H_
/**
 * Base class for Combo Procedures
 * @author Welter Luigi
 * 
 */
#include "GeneralProcedure.h" 
#include "comboreduct/combo/procedure_call.h"

#include <iostream>
#include <exception>

namespace Procedure {

class ComboProcedure : public GeneralProcedure, public combo::procedure_call_base {
  
 public: 
  ComboProcedure(); // just to be used in internal maps of ComboProcedureRepository
  ComboProcedure(const procedure_call_base& pc); //copy a procedure_call into a ComboProcedure
  ComboProcedure(const std::string& name, unsigned int arity, const combo::combo_tree&, bool infer_type = false); 
  
  virtual ~ComboProcedure();
  
  ProcedureType getType() const; 
  const std::string& getName() const;
  const combo::combo_tree& getComboTree() const;
  unsigned int getArity() const { return arity(); }
  
//  combo::combo_tree& getComboTree() { return _body; }
};

} 

std::istream& operator>>(std::istream&,Procedure::ComboProcedure&) throw (opencog::ComboException, std::bad_exception);
std::ostream& operator<<(std::ostream&,const Procedure::ComboProcedure&);


#endif /*COMBOPROCEDURE_H_*/
