#ifndef _OPENCOG_DYNAMICS_MODULE_WRAP_H
#define _OPENCOG_DYNAMICS_MODULE_WRAP_H

#include <boost/python/object.hpp>

class dynamics_scope_t;

boost::python::object dynamics_scope;

/** Exposes the dynamics module.
 *
 * This function creates the dynamics scope and then exposes parts of the 
 * dynamics library beneeth it.
 */
void init_dynamics_module_py();

#endif // _OPENCOG_DYNAMICS_MODULE_WRAP_H
