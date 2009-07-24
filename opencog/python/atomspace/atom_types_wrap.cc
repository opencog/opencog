#include "atom_types_wrap.h"
#include <opencog/atomspace/atom_types.h>
#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>

using namespace boost::python;
using namespace opencog;

class atom_types_scope_t {};
object atom_types_scope;

void init_atom_types_py()
{
    // Setup scope.
    class_<atom_types_scope_t> a("types");
    atom_types_scope = a;
    scope within(atom_types_scope);

    a.def_readonly("ATOM", &ATOM);
    a.def_readonly("NODE", &NODE);
}
