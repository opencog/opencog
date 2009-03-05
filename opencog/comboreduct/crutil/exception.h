#ifndef _COMBOREDUCT_EXCEPTION_H
#define _COMBOREDUCT_EXCEPTION_H

#include <string>
#include "ComboReduct/combo/vertex.h"

namespace combo {

class Exception {
protected:
  std::string _message;
public:
  Exception();
  Exception(std::string m);

  std::string get_message();
};

class EvalException : public Exception {
  combo::vertex _vertex;
public:
  EvalException();
  EvalException(combo::vertex v);
  
  combo::vertex get_vertex();
};

class TypeCheckException : public Exception {
  int _arg;
 public:
  TypeCheckException();
  TypeCheckException(int arg);
};

}
#endif
