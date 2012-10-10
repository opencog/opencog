#ifndef _OPENCOG_ATOMSPACE_ASYNC_H
#define _OPENCOG_ATOMSPACE_ASYNC_H

#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <opencog/util/concurrent_queue.h>

#include "AtomSpaceImpl.h"
#include "ASRequest.h"
#include "Handle.h"
#include "types.h"

#ifdef ZMQ_EXPERIMENT
    #include <zmq.hpp>
	#include <opencog/atomspace/ZMQMessages.pb.h>
#endif

#ifdef ZMQ_EXPERIMENT
/*
Setup:
	Install zeroMQ (should already be installed)
 		http://www.zeromq.org/intro:get-the-software
 		eclipse: add zmq to libraries
 	Install protocol buffers (should already be installed)
 		http://code.google.com/p/protobuf/downloads/list
 		./configure
 		make
 		sudo make install
 		eclipse: add protobuf to libraries
 	Compile proto file (if changed)
		protoc --cpp_out=. AtomSpaceMessages.proto
	launch cogserver
		cd ~/ochack; bin/opencog/server/cogserver -c lib/opencog.conf
	location of scm files
		/usr/local/share/opencog/scm
Compiling in Eclipse:
	Folders
		~/ochack: my opencog working folder
		~/ocserver: my eclipse folder
	Compile opencog
		cd ~/ochack/bin
		cmake ..
		make
	Create a new Eclipse project in ocserver
	Copy server and atomspace folders from ochack to ocserver/opencog
		ocserver
			opencog
				server
				atomspace
		Note: instead of server you'll probably want to write your own test executable
	Eclipse settings
		Note: All of the following settings are done in project properties/C C++ build/Settings/Tool settings
		Add Include paths (-I) (compile errors: "xxx not found")
			/home/erwin/ocserver
			/home/erwin/ochack
			Note: order is important, add full path not ~/ocserver
		Add defined symbols
			 -DCONFDIR=\"/usr/local/etc\" -DDATADIR=\"/usr/local/share/opencog\" -Dserver_EXPORTS -DHAVE_EXPAT -DHAVE_GSL -DHAVE_GUILE -DHAVE_PROTOBUF -DHAVE_CYTHON -DHAVE_SQL_STORAGE -DHAVE_ZMQ -DHAVE_UBIGRAPH
			 Note: you can add them under Preprocessor but it's easier to just paste them in miscellaneous/other flags
			 Note: in future also -DZMQ_EXPERIMENT if you want to enable ZMQ
		Exclude template implementations (compile errors: "xxx already defined")
			right click on opencog/atomspace/HandleMap.cc and choose properties then check Exclude resource from build
		Exclude or delete test files that define Main (compile error: "Main() already defined")
		Add Libraries (-l) (link errors)
			boost_filesystem
			persist
			smob
			gsl
			gslcblas
			boost_signals
			boost_thread
			xml
			util
			SpaceMap
			zmq
			boost_system
			protobuf
			Note: gsl has to come before gslcblas (change order if needed)
		Add Library search path (-L) (link errors)
			/home/erwin/ochack/bin/opencog/spatial
			/home/erwin/ochack/bin/opencog/persist/sql
			/home/erwin/ochack/bin/opencog/guile
			/home/erwin/ochack/bin/opencog/util
			/home/erwin/ochack/bin/opencog/persist/xml
			Note: you have to enter the full paths here not ~/ochack/...
		Change LD_LIBRARY_PATH (runtime error: "xxx cannot open shared object file"
			Note: this setting is in Run Debug settings/Environment tab, not environment in Build
			Append to the existing value :/home/erwin/ochack/bin/opencog/persist/sql:/home/erwin/ochack/bin/opencog/guile:/home/erwin/ochack/bin/opencog/util:/home/erwin/ochack/bin/opencog/spatial:/home/erwin/ochack/bin/opencog/persist/xml

How it works:
	ZeroMQ allows you to connect a client process to the atomspace managed by the server (normally the
	cogserver). Use cases include deploying an agent on a separate server, easier debugging of agents,
	connecting components that are written in different languages (any language that supports protocolbuffers
	(see http://code.google.com/p/protobuf/wiki/ThirdPartyAddOns) can use a simple message based wrapper to talk
	to the server. Of course implementing the full atomspace OO interface takes a lot more work. )
	and connecting tools that need high performance access to the atomspace (e.g. the atomspace visualizer).
	You can connect multiple clients to one server. The clients can be anywhere in the world as long as
	the server is reachable via TCP/IP.

	Server:
		if server is enabled run zmqloop
		zmqloop
			check for RequestMessage
			switch(function)
				case getAtom:
					call getAtom with arguments from RequestMessage
					store return value in ReplyMessage
				case getName:
					etc...
			send ReplyMessage
	Client:
		async queue works the same as usual but if client is enabled call server instead of Run
			copy ASRequest arguments and function number to RequestMessage
			send RequestMessage to server
			wait for ReplyMessage
			copy ReplyMessage to ASRequest

How to compile:
	 Add new files and libs to opencog/atomspace/CMakeLists.txt
	 	 add ZMQMessages.pb.cc to ADD_LIBRARY (atomspace SHARED
	 	 add protobuf to SET(ATOMSPACE_LINK_LIBS
	 	 add ZMQMessages.pb.h to INSTALL (FILES
	 	 run cmake .. in bin folder
	Add  -DZMQ_EXPERIMENT to CXX_DEFINES in the flags.make file in the following folders:
		/home/erwin/ochack/bin/opencog/atomspace/CMakeFiles/atomspace.dir
		/home/erwin/ochack/bin/opencog/server/CMakeFiles/cogserver.dir
		/home/erwin/ochack/bin/opencog/server/CMakeFiles/server.dir
Problem solving:
	It doesn't matter if you start the server or client first but if the server crashes or is restarted
	you have to restart the client(s) as well.
	If you get error "Address already in use" then another instance of the server is already running
	also serialize node and link when sending
	also serialize node and link when receiving
Performance
	Both ZMQ and protocolbuffers are very fast. In this test it serializes/deserializes 500 messages per millisecond http://stackoverflow.com/questions/647779/high-performance-serialization-java-vs-google-protocol-buffers-vs
	If you use the async interface correctly (send commands to fetch all the data you need first then start processing it) the local processing, network communications and database activity all overlap and it may actually be faster than a synchronized local approach. Ideally by the time you call getresult() the data will already be waiting in local memory.
    e.g. if you have a loop that does some processing on atoms
		for(i)
			atom=atomspace.getAtom
			//non-trivial processing
     split it into two loops for up to 100% performance improvement:
		 vector<> future
		 for(i)
			future[i]=atomspaceasync.getAtom
		 for(i)
			atom=future[i].getResult();
			//non-trivial processing

TODO:
    -febcorpus.scm doesn't load
	-cogserver test: use getAtom (worst case) and getSTI (best case)
	-the server zmqloop will be a bottleneck unless we can make atomspaceimpl multithreaded and run multiple zmqloops (advanced)
	-don't use the server eventloop
	-skip the eventloop for atomspace (only queue atomspaceasync)
	-don't clone atom in getatom
	-use protobuf messages in ASRequest?
	-use zero copy for zmq http://zguide.zeromq.org/page:all#Zero-Copy
	-clean exit of atomspaceasync without using sleep or a busy loop
		send ctrl+C signal to the thread and break when NULL message received
			char *reply = zstr_recv (client);
			if (!reply)
				if(exitSeverloop)
					logger.info "Serverloop exitting"
					break;
				else
					logger.error "Serverloop quit unexpectedly"
	-what if dowork throws an exception on the server? catch exceptions, put them in reply message and rethrow at client
		-try to handle communications exceptions locally
	-allow cogserver to listen to multiple ports e.g. inproc for agents that run in the same process, file
	based for agents that run in a different process on the same server and TCP/IP for everything else )
 */
#endif

class AtomSpaceAsyncUTest;

namespace opencog {

class AtomSpace;
class SavingLoading;

class AtomSpaceAsync {

    friend class ::AtomTableUTest;
    friend class ::AtomSpaceAsyncUTest;
    friend class AtomSpaceBenchmark;
    friend class AtomSpace;
    friend class SavingLoading;
    friend class PersistModule;

    bool processingRequests;
    boost::thread m_Thread;

#ifdef ZMQ_EXPERIMENT
    bool zmqClientEnabled;
    zmq::socket_t *zmqClientSocket;

    bool zmqServerEnabled;
    boost::thread zmqServerThread;
#endif

    int counter;

    AtomSpaceImpl atomspace;

    TimeServer* timeServer;
    SpaceServer* spaceServer;

    void startEventLoop();
    void stopEventLoop();

    concurrent_queue< boost::shared_ptr<ASRequest> > requestQueue;

    void eventLoop();

#ifdef ZMQ_EXPERIMENT
    void zmqLoop(string networkAddress);
#endif

    const AtomTable& getAtomTable() { return atomspace.getAtomTable(); };

public: 
#ifdef ZMQ_EXPERIMENT
    static zmq::context_t* zmqContext;
#endif

    AtomSpaceAsync();
    ~AtomSpaceAsync();

    // TODO: should be protected by mutex 
    int get_counter() { return counter; }

    bool isQueueEmpty() { return requestQueue.empty(); } ;

#ifdef ZMQ_EXPERIMENT
    //void enableZMQClient(string networkAddress="inproc:///AtomSpaceZMQ"); //in process communication //TODO also enable this for threads running in the atomspaceserver process (highest performance)?
    //void enableZMQServer(string networkAddress="inproc:///AtomSpaceZMQ");
    void enableZMQClient(string networkAddress="ipc:///tmp/AtomSpaceZMQ.ipc"); //named pipe for processes running on the same server
    void enableZMQServer(string networkAddress="ipc:///tmp/AtomSpaceZMQ.ipc");
    //void enableZMQClient(string networkAddress="tcp://127.0.0.1:5555"); //TCP/IP //TODO also enable this for processes running on different servers?
    //void enableZMQServer(string networkAddress="tcp://*:5555");
#endif

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
        const TruthValue* tv;
        if (!tvn.isNullTv()) tv = tvn.clone();
        // Unless it is a NULL_TV as this can't be cloned
        else tv = &TruthValue::NULL_TV();

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

    AtomRequest getAtom(const Handle& h) {
        AtomRequest r(new GetAtomASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    BoolRequest commitAtom(const Atom& a) {
        BoolRequest r(new CommitAtomASR(&atomspace,a));
        requestQueue.push(r);
        return r;
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
     * Get the number of Nodes in the AtomSpace
     */
    IntRequest nodeCount(const VersionHandle& vh) {
        IntRequest ir(new NodeCountASR(&atomspace,vh));
        requestQueue.push(ir);
        return ir;
    }

    /**
     * Get the number of Nodes in the AtomSpace
     */
    IntRequest linkCount(const VersionHandle& vh) {
        IntRequest ir(new LinkCountASR(&atomspace,vh));
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

    //! Clear the atomspace, remove all atoms
    VoidRequest clear() {
        VoidRequest r(new ClearASR(&atomspace));
        requestQueue.push(r);
        return r;
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

/*
 //TODO EJ still needed?
 #ifdef ZMQ_EXPERIMENT
    //Retrieve the TruthValue summary of a given Handle
    TruthValueRequest getTV(Handle h, VersionHandle vh = NULL_VERSION_HANDLE) {
        TruthValueRequest r(new GetTruthValueASR(&atomspace,h,vh));
        requestQueue.push(r);
        return r;
    }
#endif
*/

    /** Retrieve the complete TruthValue of a given Handle */
    TruthValueCompleteRequest getTVComplete(Handle h, VersionHandle vh = NULL_VERSION_HANDLE) {
        TruthValueCompleteRequest r(new GetCompleteTruthValueASR(&atomspace,h,vh));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the complete TruthValue of a given Handle */
    FloatRequest getMean(Handle h, VersionHandle vh = NULL_VERSION_HANDLE) {
        FloatRequest r(new GetTruthValueMeanASR(&atomspace,h,vh));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the complete TruthValue of a given Handle */
    FloatRequest getConfidence(Handle h, VersionHandle vh = NULL_VERSION_HANDLE) {
        FloatRequest r(new GetTruthValueConfidenceASR(&atomspace,h,vh));
        requestQueue.push(r);
        return r;
    }

    /** Change the TruthValue summary of a given Handle */
    VoidRequest setTV(Handle h, const TruthValue& tv, VersionHandle vh = NULL_VERSION_HANDLE) {
        VoidRequest r(new SetTruthValueASR(&atomspace,h,tv,vh));
        requestQueue.push(r);
        return r;
    }

    /** Change the primary TV's mean of a given Handle */
    VoidRequest setMean(Handle h, float mean) {
        VoidRequest r(new SetTruthValueMeanASR(&atomspace,h,mean));
        requestQueue.push(r);
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

    /** Retrieve the normalised Short-Term Importance for a given Handle.
     * Unless positive parameter is true, STI above and below the attentional
     * focus threshold is normalised separately and linearly.
     *
     * @param h The atom handle to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to -1..1, or 0..1 with
     * positive==true? Outside these ranges can be returned if average=true
     * @param positive Linearly normalise the STI from 0..1 instead of from
     * -1..1.
     * @return normalised STI
     */
    FloatRequest getNormalisedSTI(Handle h, bool average=true, bool clip=false, bool positive=false) {
        FloatRequest r(new GetNormalisedAttentionValueSTIASR(
                    &atomspace,h,average,clip,positive));
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

    /** Increase the Very Long-Term Importance of an Atom by 1*/
    VoidRequest incVLTI(Handle h) {
        VoidRequest r(new IncAttentionValueVLTIASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    /** Decrease the Very Long-Term Importance of an Atom by 1*/
    VoidRequest decVLTI(Handle h) {
        VoidRequest r(new DecAttentionValueVLTIASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    /** Retrieve the Very Long-Term Importance of a given Handle */
    VLTIRequest getVLTI(Handle h) {
        VLTIRequest r(new GetAttentionValueVLTIASR(&atomspace,h));
        requestQueue.push(r);
        return r;
    }

    HashRequest getAtomHash(const Handle& h) {
        HashRequest r(new GetAtomHashASR(&atomspace,h));
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

    inline AttentionBank& getAttentionBank()
    { return atomspace.getAttentionBank(); }

    inline BoolRequest saveToXML(const std::string& filename) {
        BoolRequest r(new SaveToXMLASR(&atomspace,filename));
        requestQueue.push(r);
        return r;
    }


};

} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_ASYNC_H
