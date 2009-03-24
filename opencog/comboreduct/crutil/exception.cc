#include "comboreduct/crutil/exception.h"

using namespace combo;

ComboReductException::ComboReductException() {}
ComboReductException::ComboReductException(std::string m) : _message(m) {}
std::string ComboReductException::get_message() {
    return _message;
}

EvalException::EvalException() {}
EvalException::EvalException(combo::vertex v) : _vertex(v) {
    _message = "Eval Exception";
}
combo::vertex EvalException::get_vertex() {
    return _vertex;
}

TypeCheckException::TypeCheckException() {
    _message = "Type check Exception";
}
TypeCheckException::TypeCheckException(int arg) : _arg(arg) {
    TypeCheckException::TypeCheckException();
}

