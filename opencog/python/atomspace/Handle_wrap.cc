#include "Handle_wrap.h"
#include "Handle.h"
#include <boost/python/class.hpp>
#include <boost/python/iterator.hpp>
#include <boost/lexical_cast.hpp>

using namespace opencog;
using namespace boost::python;

void push_back(HandleSeq &hs, Handle h)
{
    hs.push_back(h);
}

void pop_back(HandleSeq &hs)
{
    hs.pop_back();
}

void assign(HandleSeq &hs, HandleSeq::iterator first, HandleSeq::iterator last)
{
    hs.assign(first, last);
}

void assign(HandleSeq &hs, unsigned int n, const Handle& u)
{
    hs.assign(n, u);
}

void clear(HandleSeq &hs)
{
    hs.clear();
}

/*HandleSeq::iterator insert(HandleSeq &hs, HandleSeq::iterator pos, const 
Handle& h)
{
    return hs.insert(pos, h);
}*/

/*void insert(HandleSeq &hs, HandleSeq::iterator pos, unsigned int n, const Handle& h)
{
    hs.insert(pos, n, h);
}*/

void insert(HandleSeq &hs, HandleSeq::iterator pos, HandleSeq::iterator first, 
HandleSeq::iterator last)
{
    hs.insert(pos, first, last);
}

void init_Handle_py()
{
    class_<HandleSeq, boost::noncopyable>("HandleSeq")
        .def("assign",
            (void (*)(HandleSeq&, HandleSeq::iterator, HandleSeq::iterator))
            assign)
        .def("assign",
            (void (*)(HandleSeq&, unsigned int, const Handle&))
            assign)
        .def("clear", clear)
        /*.def("insert",
            (HandleSeq::iterator (*)(HandleSeq::iterator, const Handle&))
            insert)*/
        /*.def("insert",
            (void (*)(HandleSeq::iterator, unsigned int, const Handle&))
            insert)*/
        .def("insert",
            (void (*)(HandleSeq::iterator, HandleSeq::iterator,
                HandleSeq::iterator))
            insert)
        .def("push_back", push_back)
        .def("pop_back", pop_back)
        .def("__iter__", iterator<HandleSeq>())
    ;

    class_<Handle>("Handle", no_init)
        .def(init<>())
        .def(init<const Handle&>())
        .def_readonly("UNDEFINED", &Handle::UNDEFINED)
        //Nil: I commented that line because it produces a compile error
        //.def("__str__", boost::lexical_cast<std::string, const Handle&>)
    ;
}
