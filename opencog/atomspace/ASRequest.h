#ifndef _OPENCOG_ATOMSPACE_REQUEST_H
#define _OPENCOG_ATOMSPACE_REQUEST_H

#include <iostream>
#include <condition_variable>

#include <opencog/util/foreach.h>

#include <opencog/atomspace/AtomSpaceImpl.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/types.h>

using namespace std;

namespace opencog {
/** \addtogroup grp_atomspace
 *  @{
 */

/** \defgroup grp_atomspace_requess Asynchronous requests
 *
 * The AtomSpace class is essentially just a wrapper of the asynchronous
 * AtomSpaceAsync which returns ASRequest "futures" as well as allowing
 * thread-local caching of some requests. Functions in this
 * class will block until notified that they've been fulfilled by the
 * AtomSpaceAsync event loop.
 *
 *  @{
 */

class ASRequest {

protected:
    //! Overkill, but doing atomic operations on bool is not technically threadsafe
    //! Whaaaat ?? In what way?? why not ?? Huh? 
    mutable std::mutex complete_mutex;
    bool completed;

    //! For signalling that the request has been completed
    std::condition_variable complete_cond;
    //! For blocking while fulfilling the request
    mutable std::mutex the_mutex;

    AtomSpaceImpl* atomspace;
public:
    ASRequest() : completed(false) {};

    void set_atomspace(AtomSpaceImpl* as) {
        std::lock_guard<std::mutex> lock(the_mutex);
        atomspace = as;
    }
    //! We wrap the actual do_work thread so that it doesn't have
    //! to worry about obtaining the lock or notifying of completion
    void run() {
        std::lock_guard<std::mutex> lock(the_mutex);
        do_work();
        std::lock_guard<std::mutex> lock2(complete_mutex);
        completed = true;
        complete_cond.notify_all();
    }
    virtual void do_work() = 0;

    bool is_complete() {
        // Rely on separate mutex for complete, since we don't want to stall
        // if the do_work method takes a while.
        std::lock_guard<std::mutex> lock(complete_mutex);
        return completed;
    }
};

class AtomSpace;

/**
 * Request that stores and returns type T
 */
template<typename T>
class GenericASR: public ASRequest {

    friend class AtomSpace;
protected:
    T result;
    void set_result(T _result) { result = _result; }

public:
    GenericASR(AtomSpaceImpl* a){ set_atomspace(a); };

    virtual void do_work() = 0;

    T get_result() {
        std::unique_lock<std::mutex> lock(the_mutex);
        if (!is_complete()) complete_cond.wait(lock);
        return result;
    }

};

/**
 * These should be replaced by variadic templates, or we should just replace
 * arguments with protocol buffers.
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


// store and fetching atoms is low level stuff that should be handled by the
// AtomSpace internally
class StoreAtomASR : public OneParamASR <Handle, Handle> {
public:
    StoreAtomASR(AtomSpaceImpl *a, Handle h) :
        OneParamASR<Handle, Handle>(a,h)
        {};
    
    virtual void do_work() {
        atomspace->storeAtom(p1);
        set_result(p1);
    };
    
};

class FetchAtomASR : public OneParamASR <Handle, Handle> {
public:
    FetchAtomASR(AtomSpaceImpl *a, Handle h) :
        OneParamASR<Handle, Handle>(a,h)
        {};
    
    virtual void do_work() {
        set_result(atomspace->fetchAtom(p1));
    };
    
};

class FetchIncomingSetASR : public TwoParamASR <Handle, Handle, bool> {
public:
    FetchIncomingSetASR(AtomSpaceImpl *a, Handle h, bool recurse) :
        TwoParamASR<Handle, Handle, bool>(a,h,recurse)
        {};
    
    virtual void do_work() {
        set_result(atomspace->fetchIncomingSet(p1,p2));
    };
    
};


class GetIncomingASR : public OneParamASR <HandleSeq, Handle> {
public:
    GetIncomingASR(AtomSpaceImpl *a, Handle h) :
        OneParamASR<HandleSeq,Handle>(a,h) {};
    
    virtual void do_work() {
        set_result(atomspace->getIncoming(p1));
    };
    
};

// -----------------
// Search requests

class FilterASR : public GenericASR<HandleSeq> {
    AtomPredicate* p;
    Type t;
    bool subclass;
    VersionHandle vh;
public:
    FilterASR(AtomSpaceImpl *a, AtomPredicate *_p, Type _t = ATOM, bool _subclass=true,
            VersionHandle _vh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a), p(_p), t(_t), subclass(_subclass), vh(_vh) { }
    
    virtual void do_work() {
        HandleSeq _result;
        HandleSeq hs;
        atomspace->getHandleSet(back_inserter(hs), t, subclass, vh);
        foreach (Handle h, hs) {
            if ((*p)(h) && atomspace->containsVersionedTV(h, vh))
                _result.push_back(h);
        }
        set_result(_result);
    }
    
};

class GetNeighborsASR : public GenericASR<HandleSeq> {
    Handle h;
    Type linkType;
    bool subclasses;
    bool fanin;
    bool fanout;

public:
    GetNeighborsASR(AtomSpaceImpl* a,
            const Handle& _h, bool _fanin, bool _fanout,
            Type _linkType, bool _subclasses) :
        GenericASR<HandleSeq>(a), h(_h), linkType(_linkType),
        subclasses(_subclasses), fanin(_fanin), fanout(_fanout)
            { }

    ~GetNeighborsASR() { }

    virtual void do_work() {
        set_result(atomspace->getNeighbors(h,fanin,fanout,linkType,subclasses));
    }
};

class GetHandlesByOutgoingSetASR : public GenericASR<HandleSeq> {
    HandleSeq handles;
    Type* types;
    bool* subclasses;
    Arity arity;
    Type type;
    bool subclass;
    VersionHandle vh;

public:
    GetHandlesByOutgoingSetASR(AtomSpaceImpl* a,
            const HandleSeq& _handles, Type* _types, bool* _subclasses,
             Arity _arity, Type _type, bool _subclass,
             VersionHandle _vh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a) {
        types = NULL; subclasses = NULL;
        handles = _handles;
        // This is nasty - having to malloc...
        // but we can't rely on the parameters being around when the
        // request is actioned.
        arity = _arity;
        if (_types) {
            types = (Type*) malloc(sizeof(Type) * arity);
            memcpy(types, _types, sizeof(Type) * arity);
        }
        if (_subclasses) {
            subclasses = (bool*) malloc(sizeof(bool) * arity);
            memcpy(subclasses, _subclasses,sizeof(bool) * arity);
        }
        type = _type;
        subclass = _subclass;
        vh = _vh;
    }
    ~GetHandlesByOutgoingSetASR() {
        if (types) delete types;
        if (subclasses) delete subclasses;
    }

    virtual void do_work() {
        atomspace->getHandleSet(back_inserter(result),handles,types,subclasses,
                arity,type,subclass,vh);
    }
};

class GetHandlesByNameASR: public GenericASR<HandleSeq> {
    std::string name;
    Type type;
    bool subclass;
    VersionHandle vh;

public:
    GetHandlesByNameASR(AtomSpaceImpl* a,
            const std::string& _name, Type _type, bool _subclass,
             VersionHandle _vh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a) {
        name = _name;
        type = _type;
        subclass = _subclass;
        vh = _vh;
    }

    virtual void do_work() {
        atomspace->getHandleSet(back_inserter(result), name, type, subclass, vh);
    }
};

class GetHandlesByTypeASR: public GenericASR<HandleSeq> {
    Type type;
    bool subclass;
    VersionHandle vh;

public:
    GetHandlesByTypeASR(AtomSpaceImpl* a,
            Type _type, bool _subclass,
            VersionHandle _vh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a) {
        type = _type;
        subclass = _subclass;
        vh = _vh;
    }

    virtual void do_work() {
        atomspace->getHandleSet(back_inserter(result),type,subclass,vh);
    }
};

class GetHandlesByTargetASR: public GenericASR<HandleSeq> {
    Type type, targetType;
    bool subclass, targetSubclass;
    VersionHandle vh, targetVh;

public:
    GetHandlesByTargetASR(AtomSpaceImpl* a,
            Type _type, Type _targetType,
            bool _subclass, bool _targetSubclass,
            VersionHandle _vh = NULL_VERSION_HANDLE,
            VersionHandle _targetVh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a) {
        type = _type;
        subclass = _subclass;
        vh = _vh;
        targetType = _targetType;
        targetSubclass = _targetSubclass;
        targetVh = _targetVh;
    }

    virtual void do_work() {
        atomspace->getHandleSet(back_inserter(result),type,targetType,
                subclass,targetSubclass,vh,targetVh);
    }
};

class GetHandlesByTargetHandleASR: public GenericASR<HandleSeq> {
    Handle h;
    Type type;
    bool subclass;
    VersionHandle vh;

public:
    GetHandlesByTargetHandleASR(AtomSpaceImpl* a,
            Handle _h,
            Type _type,
            bool _subclass,
            VersionHandle _vh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a) {
        h = _h;
        type = _type;
        subclass = _subclass;
        vh = _vh;
    }

    virtual void do_work() {
        atomspace->getHandleSet(back_inserter(result),h,type,
                subclass,vh);
    }
};

class GetHandlesByTargetTypesASR: public GenericASR<HandleSeq> {
    Type* types;
    bool* subclasses;
    Arity arity;
    Type type;
    bool subclass;
    VersionHandle vh;

public:
    GetHandlesByTargetTypesASR(AtomSpaceImpl* a,
             Type* _types, bool* _subclasses,
             Arity _arity, Type _type, bool _subclass,
             VersionHandle _vh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a) {
        // This is nasty - having to malloc...
        // but we can't rely on the parameters being around when the
        // request is actioned.
        arity = _arity;
        types = NULL; subclasses = NULL;
        if (_types) {
            types = (Type*) malloc(sizeof(Type) * arity);
            memcpy(types, _types, sizeof(Type) * arity);
        }
        if (_subclasses) {
            subclasses = (bool*) malloc(sizeof(bool) * arity);
            memcpy(subclasses, _subclasses,sizeof(bool) * arity);
        }
        type = _type;
        subclass = _subclass;
        vh = _vh;
    }
    ~GetHandlesByTargetTypesASR() {
        if (types) delete types;
        if (subclasses) delete subclasses;
    }

    virtual void do_work() {
        atomspace->getHandleSet(back_inserter(result),types,subclasses,
               arity,type,subclass,vh);
    }
};

class GetHandlesByTargetNamesASR: public GenericASR<HandleSeq> {
    char** names;
    Type* types;
    bool* subclasses;
    Arity arity;
    Type type;
    bool subclass;
    VersionHandle vh;

public:
    GetHandlesByTargetNamesASR(AtomSpaceImpl* a,
            const char** _names, Type* _types, bool* _subclasses,
             Arity _arity, Type _type, bool _subclass,
             VersionHandle _vh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a) {
        // This is nasty - having to malloc...
        // but we can't rely on the parameters being around when the
        // request is actioned.
        arity = _arity;
        names = NULL; types = NULL; subclasses = NULL;
        if (_names) {
            names = (char**) malloc(sizeof(char*) * arity);
            for (int i=0; i < arity; i++) {
                names[i] = NULL;
                if (_names[i]) names[i] = (char*) malloc(sizeof(char) * strlen(_names[i]));
            }
        }
        if (_types) {
            types = (Type*) malloc(sizeof(Type) * arity);
            memcpy(types, _types, sizeof(Type) * arity);
        }
        if (_subclasses) {
            subclasses = (bool*) malloc(sizeof(bool) * arity);
            memcpy(subclasses, _subclasses,sizeof(bool) * arity);
        }
        type = _type;
        subclass = _subclass;
        vh = _vh;
    }
    ~GetHandlesByTargetNamesASR() {
        if (types) delete types;
        if (subclasses) delete subclasses;
        if (names) {
            for (int i=0; i < arity; i++) {
                if (names[i]) delete names[i];
            }
            delete names;
        }
    }

    virtual void do_work() {
        atomspace->getHandleSet(back_inserter(result),
               (const char**)names,types,subclasses,
               arity,type,subclass,vh);
    }
};

class GetHandlesByTargetNameASR: public GenericASR<HandleSeq> {
    std::string targetName;
    Type targetType;
    Type type;
    bool subclass;
    VersionHandle vh;
    VersionHandle targetVh;

public:
    GetHandlesByTargetNameASR(AtomSpaceImpl* a,
            const char* _targetName,
            Type _targetType,
            Type _type,
            bool _subclass,
            VersionHandle _vh = NULL_VERSION_HANDLE,
            VersionHandle _targetVh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a) {
        targetName = _targetName;
        targetType = _targetType;
        type = _type;
        subclass = _subclass;
        vh = _vh;
        targetVh = _targetVh;
    }

    virtual void do_work() {
        atomspace->getHandleSet(back_inserter(result),targetName.c_str(),
                targetType,type,subclass,vh,targetVh);
    }
};

class GetSortedHandleSetASR: public GenericASR<HandleSeq> {
    Type type;
    bool subclass;
    VersionHandle vh;
    AtomComparator* compare;

public:
    GetSortedHandleSetASR(AtomSpaceImpl* a,
            Type _type, bool _subclass,
            AtomComparator* _compare,
            VersionHandle _vh = NULL_VERSION_HANDLE) :
            GenericASR<HandleSeq>(a) {
        type = _type;
        subclass = _subclass;
        vh = _vh;
        compare = _compare;
    }

    virtual void do_work() {
        atomspace->getSortedHandleSet(back_inserter(result),type,subclass,compare,vh);
    }
};

// Requests are based on their parent class that defines the return type
typedef std::shared_ptr< GenericASR<Handle> > HandleRequest;
typedef std::shared_ptr< GenericASR<HandleSeq> > HandleSeqRequest;

/** @}*/
/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_REQUEST_H
