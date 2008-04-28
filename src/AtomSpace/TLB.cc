
#include "TLB.h"
#include "type_codes.h"

#ifdef USE_TLB_MAP

// Low-lying values are reserved for "non-real" atoms.
unsigned long TLB::uuid = NOTYPE + 1;

std::map<Handle,Atom *> TLB::handle_map;
std::map<Atom *,Handle> TLB::atom_map;

#endif
