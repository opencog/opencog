#ifndef _OPENCOG_ATOMSPACE_ASYNC_H
#define _OPENCOG_ATOMSPACE_ASYNC_H

#include <iostream>
#include <pthread.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <opencog/util/concurrent_queue.h>

#include "AtomSpaceImpl.h"
#include "ASRequest.h"
#include "Handle.h"
#include "types.h"

class AtomSpaceAsyncUTest;

namespace opencog {

class AtomSpaceAsync {

    friend class ::AtomTableUTest;
    friend class ::AtomSpaceAsyncUTest;

    bool processingRequests;
    boost::thread m_Thread;
    mutable pthread_mutex_t atomSpaceLock;
    int counter;

    AtomSpaceImpl atomspace;

    TimeServer* timeServer;
    SpaceServer* spaceServer;

    void startEventLoop();
    void stopEventLoop();

    concurrent_queue< boost::shared_ptr<ASRequest> > requestQueue;

    void eventLoop();

    const AtomTable& getAtomTable() { return atomspace.getAtomTable(); };

public: 

    AtomSpaceAsync();
    ~AtomSpaceAsync();

    // TODO: should be protected by mutex 
    int get_counter() { return counter; }

    bool isQueueEmpty() { return requestQueue.empty(); } ;

    //--------------
    // These functions are not run in the event loop, but may need guards as
    // a result. They are also too low level for a network API and should only
    // be used by modules that know what they are doing.
    inline void registerBackingStore(BackingStore *bs) { atomspace.registerBackingStore(bs); }
    inline void unregisterBackingStore(BackingStore *bs) { atomspace.unregisterBackingStore(bs); }

    //--------------
    // These functions are the core API for manipulating the AtomSpace
    // hypergraph.

    HandleRequest addNode(Type t, const std::string& str = "",
            const TruthValue& tvn = TruthValue::DEFAULT_TV() ) {
        // We need to clone the TV as the caller's TV may go out of scope
        // before the request is processed.
        TruthValue* tv = tvn.clone();
        HandleRequest hr(new AddNodeASR(&atomspace,t,str,tv));
        requestQueue.push(hr);
        return hr;
    }

    HandleRequest addLink(Type t, const HandleSeq& outgoing,
            const TruthValue& tvn = TruthValue::DEFAULT_TV() ) {
        // We need to clone the TV as the caller's TV may go out of scope
        // before the request is processed.
        TruthValue* tv = tvn.clone();
        HandleRequest hr(new AddLinkASR(&atomspace,t,outgoing,tv));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Retrieve from the Atom Table the Handle of a given node
     *
     * @param t     Type of the node
     * @param str   Name of the node
    */
    HandleRequest getHandle(Type t, const std::string& str) {
        HandleRequest hr(new GetNodeHandleASR(&atomspace,t,str));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Retrieve from the Atom Table the Handle of a given link
     * @param t        Type of the node
     * @param outgoing a reference to a HandleSeq containing
     *        the outgoing set of the link.
    */
    HandleRequest getHandle(Type t, const HandleSeq& outgoing) {
        HandleRequest hr(new GetLinkHandleASR(&atomspace,t,outgoing));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Recursively store the atom to the backing store.
     * I.e. if the atom is a link, then store all of the atoms
     * in its outgoing set as well, recursively.
     */
    HandleRequest storeAtom(Handle h) {
        HandleRequest hr(new StoreAtomASR(&atomspace,h));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Return the atom with the indicated handle. This method will
     * explicitly use the backing store to obtain an instance of the
     * atom. If an atom corresponding to the handle cannot be found,
     * then an undefined handle is returned. If the atom is found, 
     * then the corresponding atom is guaranteed to have been
     * instantiated in the atomspace.
     */
    HandleRequest fetchAtom(Handle h) {
        HandleRequest hr(new FetchAtomASR(&atomspace,h));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Use the backing store to load the entire incoming set of the atom.
     * If the flag is true, then the load is done recursively. 
     * This method queries the backing store to obtain all atoms that 
     * contain this one in their outgoing sets. All of these atoms are
     * then loaded into this atomtable/atomspace.
     */
    HandleRequest fetchIncomingSet(Handle h, bool recursive) {
        HandleRequest hr(new FetchIncomingSetASR(&atomspace,h,recursive));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Get the size of the AtomSpace
     */
    IntRequest getSize() {
        IntRequest ir(new GetSizeASR(&atomspace));
        requestQueue.push(ir);
        return ir;
    }

    /**
     * Print out the AtomSpace to the referenced stream.
     * @param output  the output stream where the atoms will be printed.
     * @param type  the type of atoms that should be printed.
     * @param subclass  if true, matches all atoms whose type is
     *              subclass of the given type. If false, matches
     *              only atoms of the exact type.
     * @warning STL streams are not thread safe but things usually work
     * passably, stream output can just get messed up.
     * @todo We should really use an alternative streaming object that is
     * thread safe.
     */
    VoidRequest print(std::ostream& output = std::cout,
               Type type = ATOM, bool subclass = true) {
        VoidRequest pr(new PrintASR(&atomspace,output,type,subclass));
        requestQueue.push(pr);
        return pr;
    }

    /**
     * Removes an atom from the atomspace
     *
     * When the atom is removed from the atomspace, all memory associated
     * with it is also deleted; in particular, the atom is removed from
     * the TLB as well, so that future TLB lookups will be invalid. 
     *
     * @param h The Handle of the atom to be removed.
     * @param recursive Recursive-removal flag; if set, the links in the
     *        incoming set of the atom to be removed will also be
     *        removed.
     * @return True if the Atom for the given Handle was successfully
     *         removed. False, otherwise.
     */
    BoolRequest removeAtom(Handle h, bool recursive = false) {
        BoolRequest r(new RemoveAtomASR(&atomspace,h,recursive));
        requestQueue.push(r);
        return r;
    }

    /** Get the atom referred to by Handle h represented as a string. */
    StringRequest atomAsString(Handle h, bool terse = true) {
        StringRequest r(new AtomAsStringASR(&atomspace,h,terse));
        requestQueue.push(r);
        return r;
    }

    BoolRequest isValidHandle(Handle h) {
        BoolRequest r(new ValidateHandleASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the name of a given Handle */
    StringRequest getName(Handle h) {
        StringRequest r(new GetNameASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the type of a given Handle */
    TypeRequest getType(Handle h) {
        TypeRequest r(new GetTypeASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the outgoing set of a given link */
    HandleSeqRequest getOutgoing(Handle h) {
        HandleSeqRequest hr(new GetOutgoingASR(&atomspace,h));
        requestQueue.push(hr);
        return hr;
    }

    /** Retrieve a single Handle from the outgoing set of a given link */
    HandleRequest getOutgoing(Handle h, int idx) {
        HandleRequest hr(new GetOutgoingIndexASR(&atomspace,h,idx));
        requestQueue.push(hr);
        return hr;
    }

    /** Retrieve the arity of a given link */
    IntRequest getArity(Handle h) {
        IntRequest r(new GetArityASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the incoming set of a given atom */
    HandleSeqRequest getIncoming(Handle h) {
        HandleSeqRequest hr(new GetIncomingASR(&atomspace,h));
        requestQueue.push(hr);
        return hr;
    }

    /** Return whether "source" is the source handle in link "link" */
    BoolRequest isSource(Handle source, Handle link) {
        BoolRequest r(new IsSourceASR(&atomspace,source,link));
        requestQueue.push(r);
        return r;
    }

    AttentionValueRequest getAV(Handle h) {
        AttentionValueRequest r(new GetAttentionValueASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    VoidRequest setAV(Handle h,const AttentionValue& av) {
        VoidRequest r(new SetAttentionValueASR(&atomspace,h,av));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the TruthValue of a given Handle */
    TruthValueRequest getTV(Handle h, VersionHandle vh = NULL_VERSION_HANDLE) {
        TruthValueRequest r(new GetTruthValueASR(&atomspace,h,vh));
        requestQueue.push(r);
        return r;
    }

    /** Change the TruthValue of a given Handle */
    VoidRequest setTV(Handle h, const TruthValue& tv, VersionHandle vh = NULL_VERSION_HANDLE) {
        VoidRequest r(new SetTruthValueASR(&atomspace,h,tv,vh));
        requestQueue.push(r);
        return r;
    }

    /** Change the primary TV's mean of a given Handle */
    VoidRequest setMean(Handle h, float mean) {
        VoidRequest r; //(new SetTruthValueMeanASR(&atomspace,h,tv,vh));
        //requestQueue.push(r);
        return r;
    }

    /** Change the Short-Term Importance of an Atom */
    VoidRequest setSTI(Handle h, AttentionValue::sti_t sti) {
        VoidRequest r(new SetAttentionValueSTIASR(&atomspace,h,sti));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the Short-Term Importance of a given Handle */
    STIRequest getSTI(Handle h) {
        STIRequest r(new GetAttentionValueSTIASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    /** Change the Long-Term Importance of an Atom */
    VoidRequest setLTI(Handle h, AttentionValue::lti_t lti) {
        VoidRequest r(new SetAttentionValueLTIASR(&atomspace,h,lti));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the Long-Term Importance of a given Handle */
    LTIRequest getLTI(Handle h) {
        LTIRequest r(new GetAttentionValueLTIASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    /** Change the Very Long-Term Importance of an Atom */
    VoidRequest setVLTI(Handle h, AttentionValue::vlti_t vlti) {
        VoidRequest r(new SetAttentionValueVLTIASR(&atomspace,h,vlti));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the Very Long-Term Importance of a given Handle */
    VLTIRequest getVLTI(Handle h) {
        VLTIRequest r(new GetAttentionValueVLTIASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

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
    HandleSeqRequest getNeighbors(const Handle h, bool fanin, bool fanout,
            Type linkType=LINK, bool subClasses=true) {
        HandleSeqRequest hr(new GetNeighborsASR(&atomspace,h,fanin,fanout,linkType,subClasses));
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
     * Decays STI of all atoms (one cycle of importance decay).
     * @deprecated importance updating should be done by ImportanceUpdating
     * Agent, but currently still used by Embodiment code.
     */
    VoidRequest decayShortTermImportance() {
        VoidRequest r(new DecaySTIASR(&atomspace));
        requestQueue.push(r);
        return r;
    }

    // wrap these in a mutex
    boost::signals::connection addAtomSignal(const AtomSignal::slot_type& function) {
        return atomspace.addAtomSignal().connect(function);
    }
    boost::signals::connection removeAtomSignal(const AtomSignal::slot_type& function) {
        return atomspace.removeAtomSignal().connect(function);
    }
    boost::signals::connection mergeAtomSignal(const AtomSignal::slot_type& function) {
        return atomspace.mergeAtomSignal().connect(function);
    }

    //--------------
    // These functions will eventually be removed. The time and space servers
    // should run separately and listen to add/remove atom signals. They are
    // also not thread safe.
    inline TimeServer& getTimeServer() const
    { return *timeServer; }

    inline SpaceServer& getSpaceServer() const
    { return *spaceServer; }

    inline BoolRequest saveToXML(const std::string& filename) {
        BoolRequest r(new SaveToXMLASR(&atomspace,filename));
        requestQueue.push(r);
        return r;
    }


};

} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_ASYNC_H
