#ifndef _OPENCOG_SERVER_MODULE_WRAP_H
#define _OPENCOG_SERVER_MODULE_WRAP_H

#include <boost/python/object.hpp>

class server_scope_t;

boost::python::object server_scope;

/** Exposes the server module.
 *
 * This function creates the server scope and then exposes parts of the 
 * server library beneeth it.
 */
void init_server_module_py();

#endif // _OPENCOG_SERVER_MODULE_WRAP_H
