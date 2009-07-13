#ifndef _OPENCOG_REGISTRY_WRAP_H
#define _OPENCOG_REGISTRY_WRAP_H

//#include "Registry.h"
#include <boost/python/wrapper.hpp>

//using namespace opencog;
using namespace boost::python;

/** Exposes the Registry class. */
void init_Registry_py();

/** A class wrapper of the Registry class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
//struct RegistryWrap : Registry, wrapper<Registry>
//{
    // Non-pure virtual functions.

    /*bool register_(const std::string& id, AbstractFactory<_BaseType> const* factory);
    bool default_register_(const std::string& id, AbstractFactory<_BaseType> const* factory);*/
    /*bool unregister(const std::string& id);
    bool default_unregister(const std::string& id);
    _BaseType* create(const std::string& id);
    _BaseType* default_create(const std::string& id);
    std::list<const char*> all(void) const;
    std::list<const char*> default_all(void) const;*/
//};

#endif // _OPENCOG_REGISTRY_WRAP_H
