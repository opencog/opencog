#ifndef _OPENCOG_ATOMSPACE_ASYNC_H
#define _OPENCOG_ATOMSPACE_ASYNC_H

#include <iostream>
#include <thread>

#include <opencog/util/concurrent_queue.h>

#include <opencog/atomspace/AtomSpaceImpl.h>
#include <opencog/atomspace/ASRequest.h>
#include <opencog/atomspace/Handle.h>

class AtomSpaceAsyncUTest;

namespace opencog {
/** \addtogroup grp_atomspace
 *  @{
 */

class AtomSpace;
class SavingLoading;

class AtomSpaceAsync
{
    friend class ::AtomTableUTest;
    friend class ::AtomSpaceAsyncUTest;
    friend class AtomSpaceBenchmark;
    friend class AtomSpace;
    friend class SavingLoading;
    friend class PersistModule;

    bool processingRequests;
    std::thread m_Thread;

    int counter;

    void startEventLoop();
    void stopEventLoop();

    typedef std::shared_ptr<ASRequest> ASRequestPtr;
    concurrent_queue<ASRequestPtr> requestQueue;

    void eventLoop();

    AtomSpaceImpl atomspace;
public: 
    AtomSpaceImpl& getImpl() { return atomspace; }
    const AtomSpaceImpl& getImplconst() const { return atomspace; }

    AtomSpaceAsync();
    ~AtomSpaceAsync();

    /** @todo should be protected by mutex */
    int get_counter() { return counter; }

    bool isQueueEmpty() { return requestQueue.empty(); } ;

    //--------------
    // These functions are not run in the event loop, but may need guards as
    // a result. They are also too low level for a network API and should only
    // be used by modules that know what they are doing.
    inline void registerBackingStore(BackingStore *bs) { atomspace.registerBackingStore(bs); }
    inline void unregisterBackingStore(BackingStore *bs) { atomspace.unregisterBackingStore(bs); }

    //--------------
    // These functions are query methods - they currently return HandleSeqs,
    // but in future the Request objects returned from these functions will be more
    // functional and allow/limit users to retrieving N results, resubmit
    // the request for more results if they exist, and specify whether to
    // search beyond the in-memory AtomTable (i.e. disk/distributed atom stores)

    /**
     * Returns neighboring atoms, following links and returning their
     * target sets.
     * @param h Get neighbours for the atom this handle points to.
     * @param fanin Whether directional links point to this node should be
     * considered.
     * @param fanout Whether directional links point from this node to
     * another should be considered.
     * @param linkType Follow only these types of links.
     * @param subClasses Follow subtypes of linkType too.
     */
    HandleSeqRequest getNeighbors(const Handle& h, bool fanin, bool fanout,
            Type linkType=LINK, bool subClasses=true) {
        HandleSeqRequest hr(new GetNeighborsASR(&atomspace,h,fanin,fanout,linkType,subClasses));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Returns the set of atoms of a given name (atom type and subclasses
     * optionally).
     *
     * @param name The desired name of the atoms.
     * @param type The type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type and name.
     */
    HandleSeqRequest getHandlesByName(const std::string& name, Type type,
                 bool subclass=true, VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new GetHandlesByNameASR(&atomspace,name,type,subclass,vh));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally).
     *
     * @param type The desired type.
     * @param subclass Whether type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     */
    HandleSeqRequest getHandlesByType(Type type,
                 bool subclass = false,
                 VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new GetHandlesByTypeASR(&atomspace,type,subclass,vh));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Returns the set of atoms of a given type which have atoms of a
     * given target type in their outgoing set (subclasses optionally).
     *
     * @param type The desired type.
     * @param targetType The desired target type.
     * @param subclass Whether type subclasses should be considered.
     * @param targetSubclass Whether target type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @param targetVh only atoms whose target contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of a given type and target type (subclasses
     * optionally).
     */
    HandleSeqRequest getHandlesByTarget(Type type,
            Type targetType,
                 bool subclass, bool targetSubclass,
                 VersionHandle vh = NULL_VERSION_HANDLE,
                 VersionHandle targetVh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new GetHandlesByTargetASR(&atomspace,type,targetType,
                    subclass,targetSubclass, vh, targetVh));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Returns the set of atoms with a given target handle in their
     * outgoing set (atom type and its subclasses optionally).
     *
     * @param handle The handle that must be in the outgoing set of the atom.
     * @param type The type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the given handle in
     * their outgoing set.
     */
    HandleSeqRequest getHandlesByTargetHandle( Handle handle, Type type,
                 bool subclass, VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new GetHandlesByTargetHandleASR(&atomspace,handle,type,
                    subclass, vh));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Returns the set of atoms whose outgoing set contains at least one
     * atom with the given name and type (atom type and subclasses
     * optionally).
     *
     * @param targetName The name of the atom in the outgoing set of the searched
     * atoms.
     * @param targetType The type of the atom in the outgoing set of the searched
     * atoms.
     * @param type type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh return only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type and name whose outgoing
     * set contains at least one atom of the given type and name.
     */
    HandleSeqRequest getHandlesByTargetName(
                 const char* targetName,
                 Type targetType,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE,
                 VersionHandle targetVh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new GetHandlesByTargetNameASR(&atomspace,
                    targetName,targetType,type,subclass, vh, targetVh));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type
     * and subclasses arguments of the searched atoms are optional.
     *
     * @param names An array of names to match the outgoing sets of the searched
     * atoms. This array (or each of its elements) can be null, if
     * the names do not matter or if do not apply to the specific search.
     * Note that if this array is not null, it must contain "arity" elements.
     * @param types An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. If array of names is not null,
     * this parameter *cannot* be null as well. Besides, if an element in a
     * specific position in the array of names is not null, the corresponding
     * type element in this array *cannot* be NOTYPE as well.
     * @param subclasses An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Not that if this
     * array is not null, it must contains "arity" elements.
     * @param arity The length of the outgoing set of the atoms being searched.
     * @param type The optional type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh return only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     */
    HandleSeqRequest getHandlesByTargetNames(
                 const char** names,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new GetHandlesByTargetNamesASR(&atomspace,names,
                    types, subclasses, arity, type, subclass, vh));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Returns the set of atoms with the given target handles and types
     * (order is considered) in their outgoing sets, where the type and
     * subclasses of the atoms are optional.
     *
     * @param handles An array of handles to match the outgoing sets of the searched
     * atoms. This array can be empty (or each of its elements can be null), if
     * the handle value does not matter or if it does not apply to the
     * specific search.
     * Note that if this array is not empty, it must contain "arity" elements.
     * @param types An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. If this array is not null, it must
     * contains "arity" elements.
     * @param subclasses An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Note that if this
     * array is not null, it must contains "arity" elements.
     * @param arity The length of the outgoing set of the atoms being searched.
     * @param type The type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     */
    HandleSeqRequest getHandlesByOutgoingSet(
                 const HandleSeq& handles, Type* types, bool* subclasses,
                 Arity arity, Type type, bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new GetHandlesByOutgoingSetASR(&atomspace,
                    handles,types,subclasses,arity,type,subclass,vh));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type
     * and subclasses arguments of the searched atoms are optional.
     *
     * @param types An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. This parameter can be null (or any of
     * its elements can be NOTYPE), what means that the type doesnt matter.
     * Not that if this array is not null, it must contains "arity" elements.
     * @param subclasses An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Not that if this
     * array is not null, it must contains "arity" elements.
     * @param arity The length of the outgoing set of the atoms being searched.
     * @param type The optional type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh returns only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     */
    HandleSeqRequest getHandlesByTargetTypes(
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new GetHandlesByTargetTypesASR(&atomspace,
                    types,subclasses,arity,type,subclass,vh));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally), sorted according to the given comparison
     * structure.
     *
     * @param type The desired type.
     * @param subclass Whether type subclasses should be considered.
     * @param compare The comparison struct to use in the sort.
     * @param vh returns only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     *
     * @return The sorted set of atoms of a given type (subclasses optionally).
     */
    HandleSeqRequest getSortedHandleSet(
                 Type type,
                 bool subclass,
                 AtomComparator* compare,
                 VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new GetSortedHandleSetASR(&atomspace, type, subclass, compare, vh));
        requestQueue.push(hr);
        return hr;
    }

    HandleSeqRequest filter(AtomPredicate* p, Type t, bool subclass = true,
            VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeqRequest hr(new FilterASR(&atomspace, p,t,subclass,vh));
        requestQueue.push(hr);
        return hr;
    }

    // TODO XXX FIXME convert to boost::signals2 ASAP for thread safety.
    boost::signals::connection addAtomSignal(const AtomSignal::slot_type& function) {
        return atomspace.addAtomSignal().connect(function);
    }
    boost::signals::connection removeAtomSignal(const AtomPtrSignal::slot_type& function) {
        return atomspace.removeAtomSignal().connect(function);
    }
    boost::signals::connection AVChangedSignal(const AVCHSigl::slot_type& function) {
        return atomspace.AVChangedSignal().connect(function);
    }
    boost::signals::connection TVChangedSignal(const TVCHSigl::slot_type& function) {
        return atomspace.TVChangedSignal().connect(function);
    }

    //--------------
    inline AttentionBank& getAttentionBank()
    { return atomspace.getAttentionBank(); }

    inline const AttentionBank& getAttentionBankconst() const
    { return atomspace.getAttentionBankconst(); }

    const AtomTable& getAtomTable() { return atomspace.getAtomTable(); };
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_ASYNC_H
