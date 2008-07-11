#ifndef _ATOMLOOKUPPROVIDER_H
#define _ATOMLOOKUPPROVIDER_H

#include <boost/smart_ptr.hpp>

#include <set>

#include <AtomSpace.h>
#include <types.h>

#ifdef WIN32
#include <string>
#endif

using namespace opencog;

class AtomLookupProvider
{
public:
    /** If name is non-empty, then return the set of all nodes with type T and
     * that name.
     * Else, return the set of all links with type T. Whether to include
     * subclasses too is optional.  NOTE! atoms with confidence < 0.0000001 are
     * not returned!
     */
    virtual boost::shared_ptr<set<Handle> > getHandleSet(Type T, const string&
            name, bool subclass = false) const =0 ;

    /** Return the set of all nodes with type T and the name str
     * NOTE! atoms with confidence < 0.0000001 are not returned!
     */
    virtual Handle getHandle(Type t,const std::string& str) const=0;

    /** return the set of all links with type T and the given outgoing set
      * NOTE! atoms with confidence < 0.0000001 are not returned!
      */
    virtual Handle getHandle(Type t,const HandleSeq& outgoing) const=0;
	virtual ~AtomLookupProvider() { };
};

#endif // _ATOMLOOKUPPROVIDER_H
