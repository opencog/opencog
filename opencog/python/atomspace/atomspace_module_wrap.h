#ifndef _OPENCOG_ATOMSPACE_MODULE_WRAP_H
#define _OPENCOG_ATOMSPACE_MODULE_WRAP_H

#include <boost/python/object.hpp>
using namespace boost::python;

class atomspace_scope_t;

extern object atomspace_scope;

/** Exposes the atomspace module.
 *
 * This function creates the atomspace scope and then exposes parts of the 
 * atomspace library beneeth it.
 */
extern void init_atomspace_module_py();

#endif // _OPENCOG_ATOMSPACE_MODULE_WRAP_H
