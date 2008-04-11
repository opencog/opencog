
#include "TLB.h"

#ifdef USE_TLB_MAP

unsigned long TLB::uuid = 1;
std::map<Handle,Atom *> TLB::handle_map;
std::map<Atom *,Handle> TLB::atom_map;

#endif
