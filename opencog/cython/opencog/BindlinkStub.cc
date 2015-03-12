#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomSpace.h>

#include "BindlinkStub.h"

using namespace opencog;

Handle opencog::stub_bindlink(AtomSpace* atomspace, Handle handle)
{
    return handle;
}
