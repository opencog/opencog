#ifndef ATOMSPACE_H
#define ATOMSPACE_H

#include <vector>
#include <set>

#include "ClassServer.h"
#include "AtomTable.h"
#include "TimeServer.h"
#include "TruthValue.h"
#include "utils2.h"
#include "CompositeTruthValue.h"
#include "AttentionValue.h"
#include "exceptions.h"

#ifndef WIN32
#include <ext/hash_map>
using __gnu_cxx::hash_map;
#define USE_ATOM_HASH_MAP
#else
#include <map>
#endif

typedef std::vector<Handle> HandleSeq;
typedef std::vector<HandleSeq> HandleSeqSeq;

typedef short stim_t;

#ifdef USE_ATOM_HASH_MAP
typedef hash_map<Atom*, stim_t, hashAtom, eqAtom> AtomHashMap;
#else
typedef map<Atom*, stim_t> AtomMap; 
#endif

class AtomSpace {

  friend class SavingLoading;

  public:
    // USED TO SEEK MEMORY LEAK
    //std::set<std::string> uniqueTimestamp;
    
    static const char* SPACE_MAP_NODE_NAME;

    ~AtomSpace();
    AtomSpace();
    
    /** 
     * @return a const reference to the AtomTable object of this AtomSpace
     */
    const AtomTable& getAtomTable() const;

    /** 
     * @return a const reference to the TimeServer object of this AtomSpace
     */
    const TimeServer& getTimeServer() const; 
        
    /**
     * Prints atoms of this AtomTable to the given output stream.
     * @param output  the output stream where the atoms will be printed.
     * @param type  the type of atoms that should be printed.
     * @param subclass  if true, matches all atoms whose type is 
     *              subclass of the given type. If false, matches 
     *              only atoms of the exact type.
     */
     void print(std::ostream& output = std::cout, 
                Type type = ATOM, bool subclass = true) const;

    /**
     * Adds both the AtTime(TimeNode <timestamp>, atom) atom 
     * representation into the AtomTable and the entry (atom,
     * timestamp) into the TimeServer of the given AtomSpace.
     *
     * @param atom the Handle of the atom to be associated to the timestamp
     * @param timestamp The timestamp to be associated to the atom.
     * @param tv Truth value for the AtTimeLink created (optional) 
     * @return the Handle of the AtTimeLink added into AtomSpace.
     */
    Handle addTimeInfo(Handle atom, unsigned long timestamp, 
                       const TruthValue& tv = TruthValue::NULL_TV());

    /**
     * Adds both the AtTime(TimeNode <t>, atom) atom representation 
     * into the AtomTable and the entry (atom, t) into the TimeServer
     * of the given AtomSpace.
     *
     * @param atom the Handle of the atom to be associated to the timestamp
     * @param t The Temporal object to be associated to the atom.
     * @param tv Truth value for the AtTimeLink created (optional)
     * @return the Handle of the AtTimeLink added into AtomSpace.
     */
    Handle addTimeInfo(Handle atom, const Temporal& t,
                       const TruthValue& tv = TruthValue::NULL_TV());

    /**
     * Removes both the AtTime(TimeNode <timestamp>, atom) atom
     * representation from the AtomTable and the entry (atom, 
     * timestamp) from the TimeServer of the given AtomSpace.
     * 
     * NOTE1: All handles in the incoming set of the corresponding
     * AtTimeLink Atom will also be removed recursively (unless the
     * recursive argument is explicitely set to false).
     *
     * NOTE2: The TimeNode that corresponds to the given removed
     * time info is also removed if its incoming set becomes empty 
     * after the removal of an AtTimeLink link (unless the 
     * removeDisconnectedTimeNodes argument is explicitly set to false).
     * 
     * @param atom the Handle of the atom to be associated to
     *        the timestamp. This argument cannot be an UNDEFINED_HANDLE. 
     *        If it is, a RuntimeException is thrown.
     * @param timestamp The timestamp to be associated to the atom.  
     *        This argument cannot be an UNDEFINED_TEMPORAL. If
     *        so, it throws a RuntimeException.
     * @param the Temporal relationship criteria to be used for 
     *        this removal operation. 
     *
     *        This method only removes the time info related to 
     *        the HandleTemporalPair objects whose Temporal matches with
     *        this argument (search criteria) applied to the given 
     *        timestamp argument. 
     *
     *        The default temporal relationship is "exact match".
     *        See the definition of TemporalRelationship enumeration 
     *        to see other possible values for it.
     * @param removeDisconnectedTimeNodes Flag to indicate if any 
     *        TimeNode whose incoming set becomes empty after the
     *        removal of the AtTimeLink link must be removed.
     * @param recursive Flag to indicate if all atoms in the
     *        incoming set of the AtTimeLink link must be 
     *        removed recursively.
     * 
     * @return True if the matching pairs (Handle, Temporal) were
     *        successfully removed. False, otherwise (i.e., no 
     *        mathing pair or any of them were not removed)
     */
    bool removeTimeInfo(Handle atom, unsigned long timestamp, TemporalTable::TemporalRelationship = TemporalTable::EXACT, bool removeDisconnectedTimeNodes = true, bool recursive = true);

    /**
     * Removes both the AtTime(TimeNode <t>, atom) atom representation from the AtomTable and the
     * entry (atom, t) from the TimeServer of the given AtomSpace.
     * 
     * NOTE1: All handles in the incoming set of the corresponding AtTimeLink Atom will also be removed recursively 
     *        (unless the recursive argument is explicitely set to false).
     * NOTE2: The TimeNode that corresponds to the given removed time info is also removed if its incoming set becomes empty 
     *        after the removal of an AtTimeLink link (unless the removeDisconnectedTimeNodes argument is explicitly set to false).
     * 
     * @param atom the Handle of the atom to be associated to the timestamp. This argument cannot be an UNDEFINED_HANDLE. 
     *        If so, it throws a RuntimeException.
     * @param t The Temporal object to be associated to the atom. This argument cannot be an UNDEFINED_TEMPORAL. 
     *        If so, it throws a RuntimeException.
     * @param removeDisconnectedTimeNode Flag to indicate if the TimeNode that corresponds to the given timestamp should be removed, 
     *        if its incoming set becomes empty after the removal of the AtTimeLink link.
     * @param the Temporal relationship criteria to be used for this removal operation, if the given Temporal object argument is not UNDEFINED_TEMPORAL. 
     *        This method only removes the time info related to the HandleTemporalPair objects whose Temporal matches with
     *        this argument (search criteria) applied to the given Temporal object argument. 
     *        The default temporal relationship is "exact match". See the definition of TemporalRelationship enumeration  
     *        to see other possible values for it.
     * @param removeDisconnectedTimeNodes Flag to indicate if any TimeNode whose incoming set becomes empty after the removal of the AtTimeLink link must be removed.
     * @param recursive Flag to indicate if all atoms in the incoming set of the AtTimeLink link must be removed recursively.
     * 
     * @return True if the matching pairs (Handle, Temporal) were successfully removed. False, otherwise (i.e., no mathing pair or any of them were not removed)
     */
    bool removeTimeInfo(Handle atom, const Temporal& t = UNDEFINED_TEMPORAL,TemporalTable::TemporalRelationship = TemporalTable::EXACT, bool removeDisconnectedTimeNodes = true, bool recursive = true);

    /**
     * Gets the corresponding AtTimeLink for the given HandleTemporalPair value
     * @param the pair (Handle, Temporal) that defines an AtTimeLink instance.
     * @return the Handle of the corresponding AtTimeLink, if it exists.
     */
    Handle getAtTimeLink(const HandleTemporalPair& htp) const;

    /**
     * Gets a list of HandleTemporalPair objects given an Atom Handle.
     * 
     * \param outIt The outputIterator to 
     * \param h The Atom Handle
     * \param t The temporal object
     * \param c The Temporal pair removal criterion
     * 
     * \return An OutputIterator list
     * 
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<HandleTemporalPair> ret;
     *         timeServer.get(back_inserter(ret), UNDEFINED_HANDLE);
     */
    template<typename OutputIterator> OutputIterator     
    getTimeInfo(OutputIterator outIt, Handle h, const Temporal& t = UNDEFINED_TEMPORAL,
        TemporalTable::TemporalRelationship criterion = TemporalTable::EXACT) const {
        return timeServer.get(outIt, h, t, criterion);
    }

    /** Add a new node to the Atom Table,
	if the atom already exists then the old and the new truth value is merged
        \param t     Type of the node
        \param name  Name of the node
        \param tvn   Optional TruthValue of the node. If not provided, uses the DEFAULT_TV (see TruthValue.h) */
    Handle addNode(Type t,const string& name,const TruthValue& tvn = TruthValue::DEFAULT_TV());

    /** Add a new link to the Atom Table
	if the atom already exists then the old and the new truth value is merged
        \param t         Type of the link
        \param outgoing  a const reference to a HandleSeq containing the outgoing set of the link
        \param tvn   Optional TruthValue of the node. If not provided, uses the DEFAULT_TV (see TruthValue.h) */
    Handle addLink(Type t,const HandleSeq& outgoing,const TruthValue& tvn = TruthValue::DEFAULT_TV());

    /** 
     * Add an atom represented by a given handle with an optional
     * TruthValue object to the Atom Table
     *
     * @param the handle of the Atom to be added
     * @param the TruthValue object to be associated to the added 
     *        atom. NULL if the own atom's tv must be used.
     * @param a flag to indicate if it does not need to check for
     *         already existing atoms in AtomTable. 
     * @param managed ???
     */
    Handle addRealAtom(const Atom& atom,
                       const TruthValue& tvn=TruthValue::NULL_TV());

    /**
     * Add a new atom represented by a tree of Vertex to the 
     * Atom Table.
     *
     * @param the tree of Vertex that contains the representation
     *        of the atom to be added
     * @param the iterator that points to the Vertex as root of 
     *        the subTree (of the given Tree of Vertex) to be
     *        added as an Atom is.
     * @param the TruthValue object to be associated to the added atom
     * @param a flag to indicate if it does not need to check for
     *         already existing atoms in AtomTable. 
     * @param managed ???
     */
    Handle addAtom(tree<Vertex>& a, 
                   tree<Vertex>::iterator it, const TruthValue& tvn);

    /** 
     * Add a new atom represented by a tree of Vertex to the Atom Table
     * @param the tree of Vertex representation of the atom to be added
     * @param the TruthValue object to be associated to the added atom
     * @param a flag to indicate if it does not need to check for 
     *        already existing atoms in AtomTable. 
     * @param managed ???
     */
    Handle addAtom(tree<Vertex>& a, const TruthValue& tvn);

    /**
     * Removes an atom from the atomspace 
     *
     * @param The Handle of the atom to be removed.
     * @param Recursive-removal flag; if set, the links in the 
     *        incoming set of the atom to be removed will also be
     *        removed.
     * @return True if the Atom for the given Handle was successfully
     *         removed. False, otherwise.  
     */
    bool removeAtom(Handle h, bool recursive = false);

    /**
     * Retrieve from the Atom Table the Handle of a given node 
     *
     * @param t     Type of the node
     * @param str   Name of the node
    */
    Handle getHandle(Type t,const string& str) const;

    /**
     * Retrieve from the Atom Table the Handle of a given link 
     * @param t        Type of the node
     * @param outgoing a reference to a HandleSeq containing
     *        the outgoing set of the link.
    */
    Handle getHandle(Type t,const HandleSeq& outgoing) const;

    /** Retrieve the name of a given Handle */
    const string& getName(Handle) const;

    /** Change the name of a given Handle */
    void setName(Handle, const string& name);

    /** Retrieve the type of a given Handle */
    Type getType(Handle) const;

    /** Retrieve the type of a given Vertex tree */
    Type getTypeV(const tree<Vertex>& _target) const;

    /** Retrieve the TruthValue of a given Handle */
    const TruthValue& getTV(Handle, VersionHandle = NULL_VERSION_HANDLE) const;

    /** Change the TruthValue of a given Handle */
    void setTV(Handle,const TruthValue&, VersionHandle = NULL_VERSION_HANDLE);

    /** Change the primary TV's mean of a given Handle */
    void setMean(Handle, float mean) throw (InvalidParamException);

    /** Retrieve the AttentionValue of a given Handle */
    const AttentionValue& getAV(Handle) const;

    /** Change the AttentionValue of a given Handle */
    void setAV(Handle,const AttentionValue&);

    /** Change the Short-Term Importance of a given Handle */
    void setSTI(Handle, AttentionValue::sti_t);

    /** Change the Long-term Importance of a given Handle */
    void setLTI(Handle, AttentionValue::lti_t);
    
    /** Change the Very-Long-Term Importance of a given Handle */
    void setVLTI(Handle, AttentionValue::vlti_t);
    
    /** Retrieve the Short-Term Importance of a given Handle */
    AttentionValue::sti_t getSTI(Handle) const;

    /** Retrieve the Long-term Importance of a given Handle */
    AttentionValue::lti_t getLTI(Handle) const;
    
    /** Retrieve the Very-Long-Term Importance of a given Handle */
    AttentionValue::vlti_t getVLTI(Handle) const;    

    /** Retrieve a single Handle from the outgoing set of a given link */
    Handle getOutgoing(Handle,int idx) const;

    /** Retrieve the arity of a given link */
    int getArity(Handle) const;

    bool isReal(Handle h) const;

    /** Retrieve the outgoing set of a given link */
#ifdef USE_STD_VECTOR_FOR_OUTGOING
    const HandleSeq& getOutgoing(Handle h) const;
#else
    HandleSeq getOutgoing(Handle h) const;
#endif

    /** Retrieve the incoming set of a given link */
    HandleSeq getIncoming(Handle) const;

    /** Retrieve the Count of a given Handle */
    float getCount(Handle) const;

    /** Returns the default TruthValue */
    static const TruthValue& getDefaultTV();

    //type properties
    Type getAtomType(const string& typeName) const;
    bool isNode(Type t) const;
    bool inheritsType(Type t1,Type t2) const;
    string getName(Type t) const;

    /**
     * Gets a set of handles that matches with the given arguments. 
     * 
     * @param An output iterator.
     * @param the type of the atoms to be searched
     * @param the name of the atoms to be searched. For searching only links, use an empty string
     * @param if subTypes of the given type are accepted in this search
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle. 
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result. 
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator 
    getHandleSet(OutputIterator result, 
                 Type t, 
                 const string& name, 
                 bool acceptSubTypes=true, 
                 VersionHandle vh = NULL_VERSION_HANDLE) const{
        HandleEntry *he = NULL;
        if (name.length()>0) {
            // This will get only nodes
            he = atomTable.getHandleSet(name.c_str(), t, acceptSubTypes, vh);
        } else {
            // In this case, it must use another method so that it returns only links.
            // Check the given type so that it behaves the same way of implementation of 
            // this method for PseudoCore
            if (t == ATOM)
                t = LINK;
            if (!isNode(t))
                he = atomTable.getHandleSet(t, acceptSubTypes, vh);
        }
        return (toOutputIterator(result, he));        
    }

    /**
     * Gets a set of handles that matches with the given type (subclasses optionally).
     * 
     * @param An output iterator.
     * @param The desired type.
     * @param Whether type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle. 
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     *  
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator 
    getHandleSet(OutputIterator result, 
                 Type type, 
                 bool subclass, 
                 VersionHandle vh = NULL_VERSION_HANDLE) const{
    
        HandleEntry * handleEntry = atomTable.getHandleSet(type, subclass, vh);    
        return (toOutputIterator(result, handleEntry));        
    }
    
    /**
     * Returns the set of atoms of a given type which have atoms of a
     * given target type in their outgoing set (subclasses optionally).
     * 
     * @param An output iterator. 
     * @param The desired type.
     * @param The desired target type.
     * @param Whether type subclasses should be considered.
     * @param Whether target type subclasses should be considered.
      * @param if returns only atoms that contains versioned TVS with the given VersionHandle. 
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result. 
     * @return The set of atoms of a given type and target type (subclasses 
     * optionally).
     * 
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true); 
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result, 
                 Type type, 
                 Type targetType, 
                 bool subclass, 
                 bool targetSubclass, 
                 VersionHandle vh = NULL_VERSION_HANDLE, 
                 VersionHandle targetVh = NULL_VERSION_HANDLE) const{
        
        HandleEntry * handleEntry = atomTable.getHandleSet(type, targetType, subclass, targetSubclass, vh, targetVh);    
        return (toOutputIterator(result, handleEntry));        
    }
    
    /**
     * Returns the set of atoms with a given target handle in their
     * outgoing set (atom type and its subclasses optionally).
     * 
     * @param An output iterator.
     * @param The handle that must be in the outgoing set of the atom.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle. 
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result. 
     * @return The set of atoms of the given type with the given handle in
     * their outgoing set.
     * 
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */    
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result, 
                 Handle handle, 
                 Type type, 
                 bool subclass, 
                 VersionHandle vh = NULL_VERSION_HANDLE) const{
        
        HandleEntry * handleEntry = atomTable.getHandleSet(handle, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));        
    }
    
    /**
     * Returns the set of atoms with the given target handles and types
     * (order is considered) in their outgoing sets, where the type and
     * subclasses of the atoms are optional.
     *
     * @param An output iterator.  
     * @param An array of handles to match the outgoing sets of the searched
     * atoms. This array can be empty (or each of its elements can be null), if 
     * the handle value does not matter or if it does not apply to the 
     * specific search. 
     * Note that if this array is not empty, it must contain "arity" elements.
     * @param An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms.
     * @param An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null, 
     * what means that subclasses will not be considered. Note that if this
     * array is not null, it must contains "arity" elements.
     * @param The length of the outgoing set of the atoms being searched.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle. 
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result. 
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     * 
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true); 
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result, 
                 const HandleSeq& handles, 
                 Type* types, 
                 bool* subclasses, 
                 Arity arity, 
                 Type type, 
                 bool subclass, 
                 VersionHandle vh = NULL_VERSION_HANDLE) const{
   
        HandleEntry * handleEntry = atomTable.getHandleSet(handles, types, subclasses, arity, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));    
    }
    
    /**
     * Returns the set of atoms of a given name (atom type and subclasses
     * optionally).
     *
     * @param An output iterator.    
     * @param The desired name of the atoms.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle. 
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.  
     * @return The set of atoms of the given type and name.
     * 
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true); 
     */    
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result, 
                 const char* name, 
                 Type type, 
                 bool subclass, 
                 VersionHandle vh = NULL_VERSION_HANDLE) const{
        
        HandleEntry * handleEntry = atomTable.getHandleSet(name, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));        
    }
    
    /**
     * Returns the set of atoms whose outgoing set contains at least one
     * atom with the given name and type (atom type and subclasses
     * optionally).
     *
     * @param An output iterator.  
     * @param The name of the atom in the outgoing set of the searched
     * atoms.
     * @param The type of the atom in the outgoing set of the searched
     * atoms.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle. 
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.   
     * @return The set of atoms of the given type and name whose outgoing
     * set contains at least one atom of the given type and name.
     * 
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true); 
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result, 
                 const char* targetName, 
                 Type targetType, 
                 Type type, 
                 bool subclass, 
                 VersionHandle vh = NULL_VERSION_HANDLE, 
                 VersionHandle targetVh = NULL_VERSION_HANDLE) const{
        
        HandleEntry * handleEntry = atomTable.getHandleSet(targetName, targetType, type, subclass, vh, targetVh);
        return (toOutputIterator(result, handleEntry));        
    }
    
    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type 
     * and subclasses arguments of the searched atoms are optional.
     *
     * @param An output iterator.  
     * @param An array of names to match the outgoing sets of the searched
     * atoms. This array (or each of its elements) can be null, if 
     * the names do not matter or if do not apply to the specific search. 
     * Note that if this array is not null, it must contain "arity" elements.
     * @param An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. If array of names is not null, 
     * this parameter *cannot* be null as well. Besides, if an element in a 
     * specific position in the array of names is not null, the corresponding 
     * type element in this array *cannot* be NOTYPE as well.
     * @param An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null, 
     * what means that subclasses will not be considered. Not that if this
     * array is not null, it must contains "arity" elements.
     * @param The length of the outgoing set of the atoms being searched.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle. 
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result. 
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     * 
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true); 
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result, 
                 const char** names, 
                 Type* types, 
                 bool* subclasses, 
                 Arity arity, 
                 Type type, 
                 bool subclass, 
                 VersionHandle vh = NULL_VERSION_HANDLE) const{
        
        HandleEntry * handleEntry = atomTable.getHandleSet(names, types, subclasses, arity, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));                
    }
    
    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type 
     * and subclasses arguments of the searched atoms are optional.
     * 
     * @param An output iterator.
     * @param An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. This parameter (as well as any of
     * its elements can be NOTYPE), what means that the type doesnt matter.
     * Not that if this array is not null, it must contains "arity" elements.
     * @param An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null, 
     * what means that subclasses will not be considered. Not that if this
     * array is not null, it must contains "arity" elements.
     * @param The length of the outgoing set of the atoms being searched.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle. 
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.  
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);  
     */    
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result, 
                 Type* types, 
                 bool* subclasses, 
                 Arity arity, 
                 Type type, 
                 bool subclass, 
                 VersionHandle vh = NULL_VERSION_HANDLE) const{

        HandleEntry * handleEntry = atomTable.getHandleSet(types, subclasses, arity, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));    
    }

    /**
     * Decays STI of all atoms (one cycle of importance decay)
     */
    void decayShortTermImportance();

    /* Next three methods are for EconomicAttentionAllocation */

    /**
     * Stimulate a Handle's atom.
     *
     * @param atom handle
     * @param amount of stimulus to give.
     * @return total stimulus given since last reset.
     */
    stim_t stimulateAtom(Handle h, stim_t amount);

    /**
     * Stimulate all atoms in HandleEntry list.
     *
     * @param linked list of atoms to spread stimulus across.
     * @param amount of stimulus to share.
     * @return remainder stimulus after equal spread between atoms.
     */
    stim_t stimulateAtom(HandleEntry* h, stim_t amount);

    /**
     * Reset stimulus.
     *
     * @return new stimulus since reset, usually zero unless another
     * thread adds more.
     */
    stim_t resetStimulus();

    /**
     * Get total stimulus.
     *
     * @return total stimulus since last reset.
     */
    stim_t getTotalStimulus();

    /**
     * Get stimulus for Atom.
     *
     * @param handle of atom to get stimulus for.
     * @return total stimulus since last reset.
     */
    stim_t getAtomStimulus(Handle h);

    //for convenience
    bool isNode(Handle) const;
    bool isVar(Handle) const;
    bool isList(Handle) const;
    bool containsVar(Handle) const;

    Handle createHandle(Type t,const string& str,bool managed=false);
    Handle createHandle(Type t,const HandleSeq& outgoing,bool managed=false);

    int Nodes(VersionHandle = NULL_VERSION_HANDLE) const;
    int Links(VersionHandle = NULL_VERSION_HANDLE) const;
    
    bool containsVersionedTV(Handle h, VersionHandle vh) const;

// ---- filter templates

    template<typename Predicate>
    HandleSeq filter(Predicate compare, VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeq result;
        _getNextAtomPrepare();
        Handle next;
        while ((next=_getNextAtom())!=0)
            if (compare(next) && containsVersionedTV(next, vh)) 
                result.push_back(next);
        return result;
    }
  
    template<typename OutputIterator, typename Predicate>
    OutputIterator filter(OutputIterator it, Predicate compare, VersionHandle vh = NULL_VERSION_HANDLE) {
      _getNextAtomPrepare();
      Handle next;
      while ((next=_getNextAtom())!=0)
        if (compare(next) && containsVersionedTV(next, vh)) 
          *it++=next;
  
      return it;
    }

// ---- custom filter templates

    HandleSeq filter_type(Type type, VersionHandle vh = NULL_VERSION_HANDLE) {
      HandleSeq result;
      _getNextAtomPrepare_type(type);
      Handle next;
      while ((next=_getNextAtom_type(type))!=0)
        if (containsVersionedTV(next, vh)) 
          result.push_back(next);
  
      return result;
    }
  
    template<typename OutputIterator>
    OutputIterator filter_type(OutputIterator it, Type type, VersionHandle vh = NULL_VERSION_HANDLE) {
      _getNextAtomPrepare_type(type);
      Handle next;
      while ((next=_getNextAtom_type(type))!=0)
        if (containsVersionedTV(next, vh)) 
          *it++=next;
  
      return it;
    }

  protected:

    HandleIterator* _handle_iterator;
    HandleEntry* _handle_entry;
    // these methods are used by the filter_* templates
    void _getNextAtomPrepare();
    Handle _getNextAtom();
    void _getNextAtomPrepare_type(Type type);
    Handle _getNextAtom_type(Type type);

  private:
    
    TimeServer timeServer;
    AtomTable atomTable;
    string emptyName;
    
    // Total stimulus given out to atoms
    stim_t totalStimulus;
#ifdef USE_ATOM_HASH_MAP
    // Hash table of atoms given stimulus since reset
    AtomHashMap* stimulatedAtoms;
#else
    AtomMap* stimulatedAtoms;
#endif

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_t stimulatedAtomsLock;
#endif

    template <typename OutputIterator> OutputIterator
    toOutputIterator(OutputIterator result, HandleEntry * handleEntry) const{
    
        HandleEntry * toRemove = handleEntry;
        while (handleEntry)
        {
            *(result++) = handleEntry->handle;
            handleEntry = handleEntry->next;
        }    
        // free memory
        if(toRemove) delete toRemove;        
        return result;        
    }
    
    /*
     * Adds both the AtTime(TimeNode <timeNodeName>, atom) atom representation into the AtomTable and the
     * corresponding entry (atom, t) into the TimeServer of the given AtomSpace.
     * @param atom the Handle of the atom to be associated to the timestamp
     * @param timeNodeName the name of the TimeNode to be associated to the atom via an AtTimeLink.
     * @param tv Truth value for the AtTimeLink created (optional) 
     * @return the Handle of the AtTimeLink added into the AtomSpace.
     */
    Handle addTimeInfo(Handle h, const std::string& timeNodeName, const TruthValue& tv = TruthValue::NULL_TV());
    
};

#endif // ATOMSPACE_H
