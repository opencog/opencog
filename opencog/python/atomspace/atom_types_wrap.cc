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
    a.def_readonly("LINK", &LINK);
    a.def_readonly("CONCEPT_NODE", &CONCEPT_NODE);
    a.def_readonly("NUMBER_NODE", &NUMBER_NODE);
    a.def_readonly("ORDERED_LINK", &ORDERED_LINK);
    a.def_readonly("UNORDERED_LINK", &UNORDERED_LINK);
    a.def_readonly("SET_LINK", &SET_LINK);
    a.def_readonly("SUBSET_LINK", &SUBSET_LINK);
}
