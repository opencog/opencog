#ifndef __EXCEPTION_H
#define __EXCEPTION_H

#include <string>
#include "vertex.h"

class Exception {
protected:
  std::string _message;
public:
  Exception() {}
  Exception(std::string m) : _message(m) {}

  std::string get_message() { return _message; }
};

class EvalException : public Exception {
  combo::vertex _vertex;
public:
  EvalException() {}
  EvalException(combo::vertex v) : _vertex(v) {
    _message = "Eval Exception";
  }

  combo::vertex get_vertex() { return _vertex; }
};

class TypeCheckException : public Exception {
  int _arg;
public:
  TypeCheckException() {
    _message = "Type check Exception";
  }
  TypeCheckException(int arg) : _arg(arg) {
    TypeCheckException::TypeCheckException();
  }
};

#endif
