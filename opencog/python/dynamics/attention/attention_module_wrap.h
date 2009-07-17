#ifndef _OPENCOG_ATTENTION_MODULE_WRAP_H
#define _OPENCOG_ATTENTION_MODULE_WRAP_H

#include <boost/python/object.hpp>

class attention_scope_t;

boost::python::object attention_scope;

/** Exposes the attention module.
 *
 * This function creates the attention scope and then exposes parts of the 
 * attention library beneeth it.
 */
void init_attention_module_py();

#endif // _OPENCOG_ATTENTION_MODULE_WRAP_H
