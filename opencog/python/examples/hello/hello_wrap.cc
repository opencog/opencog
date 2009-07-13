//  Copyright Joel de Guzman 2002-2004. Distributed under the Boost
//  Software License, Version 1.0. (See accompanying file LICENSE_1_0.txt 
//  or copy at http://www.boost.org/LICENSE_1_0.txt)
//  Hello World Example from the tutorial
//  [Joel de Guzman 10/9/2002]

#include "hello_wrap.h"
#include <boost/python/module.hpp>
#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/object.hpp>
#include <boost/python/def.hpp>

int BaseWrap::notpure()
{
    if (override notpure = this->get_override("notpure"))
        return notpure();

    return Base::notpure();
}

int BaseWrap::default_notpure()
{
    return this->Base::notpure();
}

int BaseWrap::pure()
{
    return this->get_override("pure")();
}

bool BaseWrap::operator==(const Base& rhs) const
{
    return this->get_override("operator==")();
}

BOOST_PYTHON_MODULE(hello)
{
    def("greet", greet);
    def("foo", foo, args("x", "y"), "foo's docstring");

    //class_<World>("World")
    //class_<World>("World", no_init) // To expose no constructors
    class_<World>("World", init<std::string>())
        //.def(init<double, double>())
        .def("greet", &World::greet)
        .def("set", &World::set)
    ;

    class_<Var>("Var", init<std::string>())
        .def_readonly("name", &Var::name)
        .def_readwrite("value", &Var::value)
    ;

    class_<Num>("Num")
        .add_property("rovalue", &Num::get)
        .add_property("value", &Num::get, &Num::set)
    ;

    def("b", b);
    def("d", d);
    /*def("factory", factory)
    ;*/
    // Tell Python to take ownership of factory's result
    def("factory", factory,
        return_value_policy<manage_new_object>())
    ;

    class_<BaseWrap, boost::noncopyable>("Base")
        // For pure virtual functions.
        .def("pure", pure_virtual(&Base::pure))
        .def(self == self)
        // For virtual functions with default implementations.
        .def("notpure", &Base::notpure, &BaseWrap::default_notpure)
        //.def("~Base", &Base::~Base, &BaseWrap::default_tilda_Base)
    ;

    class_<Derived, bases<Base> >("Derived")
        .def("pure", &Derived::pure)
    ;

    class_<FilePos>("FilePos")
        .def(self + self)
        /*.def(self + int())          // __add__
        .def(int() + self)          // __radd__
        .def(self - self)           // __sub__
        .def(self - int())          // __sub__
        .def(self += int())         // __iadd__
        .def(self -= other<int>())
        .def(self < self)           // __lt__*/
    ;

    // Function overloads.
    class_<X>("X")
        .def("f", fx1)
    ;

    // Python scopes
    scope().attr("yes") = 1;
    scope().attr("no") = 0;

    // Change the current scope 
    object ass =
        class_<A>("A")
            .def("f", &A::f)
            ;
    scope myscope(ass);

    // Define a class Y in the current scope, A
    class_<A::Y>("Y")
        .def("g", &A::Y::g)
    ;
}
