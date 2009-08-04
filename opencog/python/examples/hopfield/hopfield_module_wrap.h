#ifndef _OPENCOG_HOPFIELD_MODULE_WRAP_H
#define _OPENCOG_HOPFIELD_MODULE_WRAP_H

#include <boost/python/object.hpp>
using namespace boost::python;

class hopfield_scope_t;

extern object hopfield_scope;

/** Exposes the hopfield module.
 *
 * This function creates the hopfield scope and then exposes parts of the 
 * hopfield example beneeth it.
 */
extern void init_hopfield_module_py();

#endif // _OPENCOG_HOPFIELD_MODULE_WRAP_H
