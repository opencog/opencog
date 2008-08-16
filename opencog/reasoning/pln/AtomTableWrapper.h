#ifndef ATW_H
#define ATW_H

#include <time.h>
#include <queue>
#include <boost/smart_ptr.hpp>

#include "iAtomTableWrapper.h"
#include "rules/Rule.h"
#include "utils/fim.h"
#include "utils/Singleton.h"
//#include "CoreWrapper.h"

//! The value at which PLN considers a TV a binary True.
#define PLN_TRUE_MEAN 0.989

// Each of the below merely passes through to an appropriate AtomSpace
// function
// TODO: should be placed in AtomTableWrapper class
//! return the handle of atom that has index i in h's outgoing set
Handle child(Handle h, int i);
//! returns whether the type of h is T or inherits from T
bool isSubType(Handle h, Type T);
//! returns whether 
bool inheritsType(Type subT, Type superT);

namespace reasoning
{
//! Construct a vtree around Handle h
vtree make_vtree(Handle h);

/** The bridge between the AtomSpace and PLN.
 * @todo rename to AtomSpaceWrapper... AtomSpace is the interface, AtomTable the
 * implementation.
 */
class AtomTableWrapper : public iAtomTableWrapper
{
    //! How to represent the universe size
    // CONST_SIZE = constant value
    // REAL_SIZE  = actual size of knowledge/experience
    enum USizeMode_t { CONST_SIZE, REAL_SIZE };

    //! Universe size
	uint USize;
    //! How the universe size is being calculated
	USizeMode_t USizeMode;

    //! To get around the lack of a fresh=true method in OpenCog, and to allow
    //! multiple atoms with either the same type, name and/or outgoing set, we
    //! create a number of dummy PLN contexts, each providing a different
    //! VersionHandle with which to store multiple TruthValues in an Atom.
    //! This parameter indicates the maximum number dummy contexts that have
    //! been needed so far.
    int dummyContexts;

    // ARI: is this a correct description?
    //! Add a tree of non real atoms to AtomSpace.
    //! @param vtree of atoms to add.
    //! @param iterator to start adding atoms from
    //! @param what truth value they should be given?
    //! @param fresh allows atoms to be added with the same name/outgoing set
    //! @param some kind of mechanism to manage memory?
	Handle addAtom(vtree&, vtree::iterator, const TruthValue& tvn, bool fresh, bool managed);
public:
    //! Which XML files have been loaded by PLN to populate the AtomSpace
	set<std::string> loadedFiles;
	
    //! Debug method to display nodes
	void DumpCoreNodes(int logLevel);
    //! Debug method to display links
	void DumpCoreLinks(int logLevel);
    //! Debug method to display all atoms of Type T 
	void DumpCore(Type T);

    //! return the size of the universe
    //! TODO: get the universe from the real AtomSpace if USizeMode == REAL_SIZE
	unsigned int getUniverseSize() const { return USize; }

    //! set the universe size (only if USizeMode == CONST_SIZE)
	void setUniverseSize(USizeMode_t _USizeMode, unsigned int _USize)
	{
		assert(_USizeMode == CONST_SIZE);
		USizeMode = _USizeMode;
		USize = _USize;
	}

    //! Get handles with type t and name str optionally subtypes as well
	virtual Btr<set<Handle> > getHandleSet(Type T, const string& name,
            bool subclass = false) const;
    //! Get handle of node with type t and name str
	Handle getHandle(Type t,const std::string& str) const;
    //! Get handle of link with type t and outgoing set 
    Handle getHandle(Type t,const HandleSeq& outgoing) const;

    //! Reset the AtomSpace
    void reset();

    //! Initialize new AtomSpaceWrapper with const universe size
	AtomTableWrapper() : USize(800), USizeMode(CONST_SIZE), dummyContexts(0) {}
	virtual ~AtomTableWrapper() {}

// TODELETE, replace with getAtomSpace()
//	combo::NMCore* getCore() const { return core; }

    //! Load axioms from given xml filename
	bool loadAxioms(const string& path);
    //! Load other axioms from given xml filename and optionally replace?
	bool loadOther(const std::string& path, bool replaceOld);
// TODELETE?
//	std::vector<atom> LoadAnds(const std::string& path);

	/// Makes sure that the loaded stuff is in a correct normal form etc.
    // ARI: Currently this just sets the random seed and sets link notifications
    // to true... can we merge it with the constructor?
	bool prepare();
		
// TODELETE does nothing
	int implicationConstruction();

    //! Add atom from tree vertex
	Handle addAtom(tree<Vertex>&, const TruthValue& tvn, bool fresh,
            bool managed);
    //! Add link
	virtual Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
            bool fresh, bool managed=true)=0;
    //! Add node
	virtual Handle addNode(Type T, const std::string& name,
            const TruthValue& tvn, bool fresh, bool managed=true)=0;

// TODELETE not called from anywhere
    bool hasFalsum(float minAllowedError = 0.5) { return false; }

// TODELETE not called from anywhere
    //! return a random handle of type T
	Handle getRandomHandle(Type T);

    //! Adds handle h and linked nodes (if h is a link) to AtomSpace again with
    //! fresh set to true
	Handle freshened(Handle h, bool managed);

    //! Whether the handle h has high enough TruthValue to be consider a binary True.
    //! @todo move to TruthValue classes
	bool binaryTrue(Handle h);

    //! @todo Move the below conversion tools in a Converter class
    
    //! Wrap h in a NOT_LINK and return that 
	Handle invert(Handle h);
    //! Convert from AND to OR link
	Handle AND2ORLink(Handle& andL, Type _ANDLinkType, Type _ORLinkType);
    //! Convert from Equivalence to Implication link
	hpair Equi2ImpLink(Handle& exL);
    //! Convert from Existance to For All link
	Handle Exist2ForAllLink(Handle& exL);
    //! Convert from OR to AND link
	Handle OR2ANDLink(Handle& andL);
    //! Convert from AND to OR link
	Handle AND2ORLink(Handle& andL);
	
    //! Add atom using dummy contexts if it already exists
    Handle addAtomDC(Atom *a, bool fresh);

// ARI: ok to delete this? TODELETE
/*	void VariableMPforms(const atom& src, set<atom, lessatom_ignoreVarNameDifferences>& res,
					 set<subst>* forbiddenBindings);				 */
};

// ARI: you said that this isn't used, however NormalizingATW inherits from
// it...
/** Passes the atoms via FIM analyzer. To turn this off, set FIM=0 in Config.
*/
class FIMATW : public AtomTableWrapper
{
// TODELETE
//	std::map<atom,int,lessatom> node2pat_id;
	fim::pat_id next_free_pat_id;
public:
	/// Semi-haxx::
	fim::grim myfim;

//	FIMATW(combo::NMCore* core) : AtomTableWrapper(core), next_free_pat_id(30001) {}
	FIMATW() : next_free_pat_id(30001) {}
	virtual ~FIMATW() {}

	Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
            bool fresh, bool managed=true);
	Handle addNode(Type T, const std::string& name, const TruthValue& tvn,
            bool fresh, bool managed=true);
};

/** Normalizes atoms before passing forward */
class NormalizingATW : public FIMATW
{
    NormalizingATW();
public:
	virtual ~NormalizingATW() {}
       
    static NormalizingATW& getInstance() {
        static NormalizingATW* instance = new NormalizingATW();
		return *instance;
    }
    
	Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
            bool fresh, bool managed=true);
	Handle addNode(Type T, const std::string& name, const TruthValue& tvn,
            bool fresh, bool managed=true);
	Btr<set<Handle> > getHandleSet(Type,const string&,bool = false);

};

/** Forwards the requests without normalizing */
class DirectATW : public AtomTableWrapper
{
	DirectATW();
public:
	virtual ~DirectATW() { }

    static DirectATW& getInstance() {
        static DirectATW* instance = new DirectATW();
		return *instance;
    }
    
	Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
            bool fresh, bool managed=true);
	Handle addNode(Type T, const std::string& name, const TruthValue& tvn,
            bool fresh, bool managed=true);
	Btr<set<Handle> > getHandleSet(Type, const string&, bool = false);

};


// ARI: Can we get rid of this? The implementation was commented out so I have
// done the same with the definition...
// TODELETE
/*using namespace boost;
class LocalATW : public AtomTableWrapper, public Singleton<LocalATW>
{
	LocalATW();

	//vector<Handle> mindShadow;
	map<Type, shared_ptr<set<Handle> > > mindShadowMap;
	map<Type, std::queue<std::set<Handle>::iterator > > q;
	
	unsigned long capacity;
	
public:
	virtual ~LocalATW() { }
	friend class Singleton<LocalATW>;
	
	static bool inHandleSet(Type T, const HandleSeq& hs, shared_ptr<set<Handle> > res, Handle* ret);
	static bool inHandleSet(Type T, const string& name, shared_ptr<set<Handle> > res, Handle* ret);
  
	Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh, bool managed=true);
	Handle addNode(Type T, const std::string& name, const TruthValue& tvn, bool fresh, bool managed=true);

	Btr<set<Handle> > getHandleSet(Type T, const string& name, bool subclass = false) const;
	
	void ClearAtomSpace();
	void DumpCore(Type T = 0);
	void SetCapacity(unsigned long atoms);
}; */


} //~namespace reasoning

#endif
