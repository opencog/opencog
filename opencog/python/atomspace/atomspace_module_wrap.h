#ifndef _OPENCOG_ATOMSPACE_MODULE_WRAP_H
#define _OPENCOG_ATOMSPACE_MODULE_WRAP_H

#include <boost/python/object.hpp>

class atomspace_scope_t;

boost::python::object atomspace_scope;

/** Exposes the atomspace module.
 *
 * This function creates the atomspace scope and then exposes parts of the 
 * atomspace library beneeth it.
 */
void init_atomspace_module_py();

#endif // _OPENCOG_ATOMSPACE_MODULE_WRAP_H
