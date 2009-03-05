#include "ComboReduct/crutil/exception.h"

using namespace combo;

Exception::Exception() {}
Exception::Exception(std::string m) : _message(m) {}
std::string Exception::get_message() { return _message; }

EvalException::EvalException() {}
EvalException::EvalException(combo::vertex v) : _vertex(v) {
  _message = "Eval Exception";
}
combo::vertex EvalException::get_vertex() { return _vertex; }

TypeCheckException::TypeCheckException() {
  _message = "Type check Exception";
}
TypeCheckException::TypeCheckException(int arg) : _arg(arg) {
  TypeCheckException::TypeCheckException();
}

