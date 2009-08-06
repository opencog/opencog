#include "SavableRepository_wrap.h"
#include <opencog/persist/file/SavableRepository.h>
#include <boost/python.hpp>

using namespace opencog;
using namespace boost::python;

void init_SavableRepository_py()
{
    class_<SavableRepositoryWrap, 
    boost::noncopyable>("SavableRepository",
        no_init)
        .def("getId", pure_virtual(&SavableRepository::getId))
        .def("saveRepository",
            pure_virtual(&SavableRepository::saveRepository))
        .def("loadRepository",
            pure_virtual(&SavableRepository::loadRepository))
        .def("clear", pure_virtual(&SavableRepository::clear))
    ;
}

// For the pure virtual functions.

const char* SavableRepositoryWrap::getId() const
{
    return this->get_override("getId")();
}
void SavableRepositoryWrap::saveRepository(FILE *f) const
{
    this->get_override("saveRepository")();
}
void SavableRepositoryWrap::loadRepository(FILE *f, HandleMap<Atom*>*)
{
    this->get_override("loadRepository")();
}
void SavableRepositoryWrap::clear()
{
    this->get_override("clear")();
}

// For the non-pure virtual functions.
