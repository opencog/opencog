#ifndef _OPENCOG_ATOMSPACE_REQUEST_H
#define _OPENCOG_ATOMSPACE_REQUEST_H

#include <iostream>
#include <pthread.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include "AtomSpaceImpl.h"
#include "Handle.h"
#include "types.h"

using namespace std;

namespace opencog {

class ASRequest {
    //! Overkill, but doing atomic operations on bool is not technically threadsafe
    mutable boost::mutex complete_mutex;
    bool completed;
protected:
    //! For signalling that the request has been completed
    boost::condition_variable complete_cond;
    //! For blocking while fulfilling the request
    mutable boost::mutex the_mutex;

    AtomSpaceImpl* atomspace;
public:
    ASRequest() : completed(false) {};

    void set_atomspace(AtomSpaceImpl* as) {
        boost::mutex::scoped_lock lock(the_mutex);
        atomspace = as;
    }
    //! We wrap the actual do_work thread so that it doesn't have
    //! to worry about obtaining the lock or notifying of completion
    void run() {
        boost::mutex::scoped_lock lock(the_mutex);
        do_work();
        boost::mutex::scoped_lock lock2(complete_mutex);
        completed = true;
        complete_cond.notify_all();
    }
    virtual void do_work() = 0;

    bool is_complete() {
        // Rely on separate mutex for complete, since we don't want to stall
        // if the do_work method takes a while.
        boost::mutex::scoped_lock lock(complete_mutex);
        return completed;
    }
};

/**
 * Request that stores and returns type T
 */
template<typename T>
class GenericASR: public ASRequest {

    T result;
public:
    GenericASR(AtomSpaceImpl* a){ set_atomspace(a); };

    virtual void do_work() = 0;

    T get_result() {
        boost::mutex::scoped_lock lock(the_mutex);
        if (!is_complete()) complete_cond.wait(lock);
        return result;
    }
    void set_result(T _result) { result = _result; }

};

/**
 * These should be replaced by variadic templates, or just using a protocol
 * buffer.
 */
template<typename T, typename A>
class OneParamASR: public GenericASR<T> {
protected:
    A p1;
public:
    OneParamASR(AtomSpaceImpl *a, const A& param1) : GenericASR<T>(a) {
        p1 = param1;
    }
};

template<typename T, typename A, typename B>
class TwoParamASR: public GenericASR<T> {
protected:
    A p1;
    B p2;
public:
    TwoParamASR(AtomSpaceImpl *a, const A& param1, const B& param2) : GenericASR<T>(a) {
        p1 = param1;
        p2 = param2;
    }
};

template<typename T, typename A, typename B, typename C>
class ThreeParamASR: public GenericASR<T> {
protected:
    A p1;
    B p2;
    C p3;
public:
    ThreeParamASR(AtomSpaceImpl *a, const A& param1, const B& param2, const C& param3) : GenericASR<T>(a) {
        p1 = param1;
        p2 = param2;
        p3 = param3;
    }
};

class AddNodeASR : public ThreeParamASR <Handle,Type,std::string,const TruthValue*> {
public:
    AddNodeASR(AtomSpaceImpl *a, Type type, const std::string& name = "", const TruthValue& tvn = TruthValue::DEFAULT_TV()) :
        ThreeParamASR<Handle,Type,std::string,const TruthValue*>(a,type,name,&tvn)
        {};
    
    virtual void do_work() {
        // TODO do full AtomSpace::addAtom implementation
        set_result(atomspace->addNode(p1, p2, *p3));
    };
    
};

class GetNodeHandleASR : public TwoParamASR <Handle,Type,std::string> {
public:
    GetNodeHandleASR(AtomSpaceImpl *a, Type type, std::string name) :
        TwoParamASR<Handle,Type,std::string>(a,type,name)
        {};
    
    virtual void do_work() {
        set_result(atomspace->getHandle(p1,p2.c_str()));
    };
    
};

class GetLinkHandleASR : public TwoParamASR <Handle,Type,HandleSeq> {
public:
    GetLinkHandleASR(AtomSpaceImpl *a, Type type, HandleSeq outgoing) :
        TwoParamASR<Handle,Type,HandleSeq>(a,type,outgoing)
        {};
    
    virtual void do_work() {
        set_result(atomspace->getHandle(p1, p2));
    };
    
};

typedef boost::shared_ptr< GenericASR<Handle> > HandleRequest;

} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_REQUEST_H
