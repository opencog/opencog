#ifndef _OPENCOG_UTIL_MODULE_WRAP_H
#define _OPENCOG_UTIL_MODULE_WRAP_H

#include <boost/python/object.hpp>

class util_scope_t;

boost::python::object util_scope;

/** Exposes the util module.
 *
 * This function creates the util scope and then exposes parts of the 
 * util library beneeth it.
 */
void init_util_module_py();

#endif // _OPENCOG_UTIL_MODULE_WRAP_H
