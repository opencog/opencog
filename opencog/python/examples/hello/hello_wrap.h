#ifndef HELLO_WRAP_H
#define HELLO_WRAP_H

//  Copyright Joel de Guzman 2002-2004. Distributed under the Boost
//  Software License, Version 1.0. (See accompanying file LICENSE_1_0.txt 
//  or copy at http://www.boost.org/LICENSE_1_0.txt)
//  Hello World Example from the tutorial
//  [Joel de Guzman 10/9/2002]

#include "hello.h"
#include <boost/python.hpp>
using namespace boost::python;

struct BaseWrap : Base, wrapper<Base>
{
    bool operator==(const Base& rhs) const;
    // For wrapping a pure virtual function.
    int pure();
    // For wrapping a virtual function that has a default implementation.
    int notpure();
    int default_notpure();
    /*void tilda_Base();
    void default_tilda_Base();*/
};

bool (X::*fx1)(int) = &X::f;

#endif
