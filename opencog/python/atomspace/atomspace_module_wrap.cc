#include <boost/python/module.hpp>

#include "atomspace_module_wrap.h"
#include "atom_types_wrap.h"
#include "AttentionValue_wrap.h"
#include "Atom_wrap.h"
#include "AtomSpace_wrap.h"
#include "CompositeTruthValue_wrap.h"
#include "CountTruthValue_wrap.h"
#include "ClassServer_wrap.h"
#include "Handle_wrap.h"
#include "IndefiniteTruthValue_wrap.h"
#include "Node_wrap.h"
#include "Link_wrap.h"
#include "SavableRepository_wrap.h"
#include "SimpleTruthValue_wrap.h"
#include "SpaceServer_wrap.h"
#include "SpaceServerContainer_wrap.h"
#include "TruthValue_wrap.h"

using namespace boost::python;

class atomspace_scope_t {};
object atomspace_scope;

void init_atomspace_module_py()
{
    // Setup scope.
    atomspace_scope = class_<atomspace_scope_t>("atomspace");
    scope within(atomspace_scope);

    // Order matters!..
    init_Handle_py();
    init_SavableRepository_py();
    init_SpaceServerContainer_py();
    init_SpaceServer_py();
    init_TruthValue_py();
    init_IndefiniteTruthValue_py();
    init_SimpleTruthValue_py();
    init_CompositeTruthValue_py();
    init_CountTruthValue_py();
    init_AttentionValue_py();
    init_Atom_py();
    init_AtomSpace_py();
    init_Node_py();
    init_Link_py();
    init_ClassServer_py();
    init_atom_types_py();
}

/*BOOST_PYTHON_MODULE(atomspace)
{
    // Setup the atomspace module info.
    scope().attr("__doc__") = "Wrapper for the OpenCog AtomSpace library";
    scope().attr("__name__") = "opencog.atomspace";
#ifndef DEBUG
    scope().attr("__version__") = "some version (debug)";
#else
    scope().attr("__version__") = "some version (release)";
#endif
    scope().attr("__author__") = "David Kilgore <davidpkilgore@gmail.com>";
    scope().attr("__credits__") =
        "Based on the following libraries:\n"
        "Boost Python v2 (http://www.boost.org)";

    init_atomspace_py();
}*/
