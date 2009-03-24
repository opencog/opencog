#ifndef _COMBOREDUCT_EXCEPTION_H
#define _COMBOREDUCT_EXCEPTION_H

#include <string>
#include "comboreduct/combo/vertex.h"

namespace combo {

class ComboReductException {
protected:
    std::string _message;
public:
    ComboReductException();
    ComboReductException(std::string m);

    std::string get_message();
};

class EvalException : public ComboReductException {
    combo::vertex _vertex;
public:
    EvalException();
    EvalException(combo::vertex v);

    combo::vertex get_vertex();
};

class TypeCheckException : public ComboReductException {
    int _arg;
public:
    TypeCheckException();
    TypeCheckException(int arg);
};

}
#endif
