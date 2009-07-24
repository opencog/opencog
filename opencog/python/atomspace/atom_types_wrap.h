#ifndef _OPENCOG_ATOM_TYPES_WRAP_H
#define _OPENCOG_ATOM_TYPES_WRAP_H

#include <boost/python/object.hpp>
using namespace boost::python;

class atom_types_scope_t;

extern object atom_types_scope;

/** Exposes the atom types. */
extern void init_atom_types_py();

#endif // _OPENCOG_ATOM_TYPES_WRAP_H
