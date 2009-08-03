#include <boost/python/module.hpp>

#include "atomspace_module_wrap.h"
#include "atom_types_wrap.h"
#include "AttentionValue_wrap.h"
#include "Atom_wrap.h"
#include "AtomSpace_wrap.h"
#include "AtomTable_wrap.h"
#include "CompositeTruthValue_wrap.h"
#include "CountTruthValue_wrap.h"
#include "ClassServer_wrap.h"
#include "Handle_wrap.h"
#include "HandleEntry_wrap.h"
#include "IndefiniteTruthValue_wrap.h"
#include "Node_wrap.h"
#include "Link_wrap.h"
#include "SavableRepository_wrap.h"
#include "SimpleTruthValue_wrap.h"
#include "SpaceServer_wrap.h"
#include "SpaceServerContainer_wrap.h"
#include "TruthValue_wrap.h"
#include "TLB_wrap.h"

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
    init_HandleEntry_py();
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
    init_AtomTable_py();
    init_Node_py();
    init_Link_py();
    init_ClassServer_py();
    init_atom_types_py();
    init_TLB_py();
}
