#ifndef ATW_H
#define ATW_H

#include <time.h>
#include <queue>
#include <vector>

#include <boost/config.hpp>
#include <boost/smart_ptr.hpp>
// Attempted to use boost::bimap in boost 1.35+ but had too much trouble
//#include <boost/bimap/unordered_set_of.hpp>
//#include <boost/bimap/multiset_of.hpp>
//#include <map>
//#include <boost/bimap/bimap.hpp>

#include <TimeServer.h>
#include <type_codes.h>

#include "PLN.h"

#include "iAtomTableWrapper.h"
#include "rules/Rule.h"
#include "utils/fim.h"
#include "utils/Singleton.h"

//! The value at which PLN considers a TV a binary True.
#define PLN_TRUE_MEAN 0.989

//using namespace boost::bimaps;

//! TODO: use static accessor method within AtomTableWrapper class instead of global
#define GET_ATW ((AtomTableWrapper*) ::haxx::defaultAtomTableWrapper)

namespace reasoning
{
//! Construct a vtree around Handle h
vtree make_vtree(Handle h);

typedef std::pair<Handle,VersionHandle> vhpair;

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
    //! This parameter indicates the name of the dummy contexts that have
    //! been used so far.
    std::set<VersionHandle> dummyContexts;
    
    //! This string is the prefix of PLN dummy contexts 
    std::string contextPrefix;

    // typedef bimap< unordered_set_of< vhpair >, set_of<Handle> > vhmap_t;
    // typedef vhmap_t::value_type vhmap_pair_t;
    //! Bidrectional map with right index being PLN "handle", left a <handle,
    //! versionhandle> pair.
    //! TODO: Handle is used, but eventually all references to Handle in PLN
    //! should be replaced by pHandle (typedefded to long int) to ensure
    //! distinctness from general OpenCog Handles... which could potentially change from
    //! long int to another type in the future. OR, PLN should be adapted to
    //! use vhpairs directly instead of relying on the AtomTableWrapper.
    // vhmap_t vhmap;
    
    typedef std::map< Handle, vhpair > vhmap_t;
    typedef vhmap_t::value_type vhmap_pair_t;
    //! Instead of above bimap, we will temporarily use a normal map, which is
    //! slower but simpler until PLN is fully functional.
    vhmap_t vhmap;

    //! template to search a map for a value (instead of a key) this is to be
    //! removed once bimap is integrated
    template < typename M, typename T > 
    typename M::const_iterator findValueInMap (const M m, const T value) const
    {
        typename M::const_iterator i;
        for (i = vhmap.begin(); i != vhmap.end(); i++) {
            if (i->second == value) {
                return i;
            }
        }
        return i;
    }

    // TODO: +1000 to be safe, but should check that no extra types are
    // defined elsewhere and just use +1.
    const static unsigned int mapOffset = NOTYPE + 1000;

    // ARI: is this a correct description?
    //! Add a tree of non real atoms to AtomSpace.
    //! @param vtree of atoms to add.
    //! @param iterator to start adding atoms from
    //! @param what truth value they should be given?
    //! @param fresh allows atoms to be added with the same name/outgoing set
    //! @param some kind of mechanism to manage memory?
    Handle addAtom(vtree&, vtree::iterator, const TruthValue& tvn, bool fresh, bool managed);

    Handle realToFakeHandle(const Handle h, const VersionHandle vh);
    std::vector< Handle > realToFakeHandle(const Handle hs);
    std::vector< Handle > realToFakeHandle(std::vector< Handle > hs, bool expand=false);

    vhpair fakeToRealHandle(const Handle f) const;
    bool hasAppropriateContext(const Handle o, VersionHandle& vh) const;
    bool isSubcontextOf(const Handle sub, const Handle super);
    Handle getSuperContext(const Handle sub) const;

    //! used by filter_type to merge collections of handles.
    template<class T>
    void mergeCopy(T& a, const T& b) {
        //T ret(a);
        copy(b.begin(), b.end(), back_inserter(a));
        //return ret;
    }

    //! Used by getImportantHandles
    struct compareSTI {
        // Warning, uses real atomspace handles in comparison
        bool operator()(const Handle& a, const Handle& b) {
            return TLB::getAtom(a)->getAttentionValue().getSTI() >
                TLB::getAtom(b)->getAttentionValue().getSTI();
        }
    };
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
            bool subclass = false);
    //! Get handle of node with type t and name str
    Handle getHandle(Type t,const std::string& str);
    //! Get handle of link with type t and outgoing set 
    Handle getHandle(Type t,const HandleSeq& outgoing);

    std::vector<Handle> getOutgoing(const Handle h);

    //! Both below are the same, code should be cleaned to remove ..AtIndex
    //! version
    Handle getOutgoing(const Handle h, const int i);
    Handle getOutgoingAtIndex(const Handle h, const int i);

    Type getType(const Handle h) const;
    std::string getName(const Handle h) const;

    //! Reset the AtomSpace
    void reset();

    //! Initialize new AtomSpaceWrapper with const universe size
    AtomTableWrapper() : USize(800), USizeMode(CONST_SIZE), contextPrefix("__PLN__") {}
    virtual ~AtomTableWrapper() {}

// TODELETE, replace with getAtomSpace()
//  combo::NMCore* getCore() const { return core; }

    //! Load axioms from given xml filename
    bool loadAxioms(const string& path);
    //! Load other axioms from given xml filename and optionally replace?
    bool loadOther(const std::string& path, bool replaceOld);
// TODELETE?
//  std::vector<atom> LoadAnds(const std::string& path);

    /// Makes sure that the loaded stuff is in a correct normal form etc.
    // ARI: Currently this just sets the random seed and sets link notifications
    // to true... can we merge it with the constructor?
    bool prepare();
        
// TODELETE does nothing
    //int implicationConstruction();

    //! Add atom from tree vertex
    Handle addAtom(tree<Vertex>&, const TruthValue& tvn, bool fresh=false,
            bool managed=true);
    //! Add link
    virtual Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
            bool fresh=false, bool managed=true)=0;
    //! Add node
    virtual Handle addNode(Type T, const std::string& name,
            const TruthValue& tvn, bool fresh=false, bool managed=true)=0;

// TODELETE not called from anywhere
    bool hasFalsum(float minAllowedError = 0.5) { return false; }

    //! return a random handle of type T
    Handle getRandomHandle(Type T);

    //! get a number of high STI handles
    std::vector<Handle> getImportantHandles(int number);

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
    
    //! Add Link via dummy contexts method
    Handle addLinkDC(Type t, const HandleSeq& hs, const TruthValue& tvn,
            bool fresh, bool managed);
    //! Add Node via dummy contexts method
    Handle addNodeDC(Type t, const string& name, const TruthValue& tvn,
            bool fresh, bool managed);
    //! Add concrete atom using dummy contexts if it already exists
    Handle addAtomDC(Atom &a, bool fresh, bool managed);

    Handle directAddLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
        bool fresh,bool managed);

    //! return the handle of atom that has index i in h's outgoing set
    Handle child(const Handle h, const int i);
    //! returns whether the type of h is T or inherits from T
    bool isSubType(Handle h, Type T);
    //! returns whether 
    bool inheritsType(Type subT, Type superT);

    void HandleEntry2HandleSet(HandleEntry& src, set<Handle>& dest) const;
    const TruthValue& getTV(Handle h);

    bool isReal(const Handle h) const;

    const TimeServer& getTimeServer() const;

    /** Retrieve the arity of a given link */
    int getArity(Handle) const;

    HandleSeq filter_type(Type t);

    bool equal(const HandleSeq& lhs, const HandleSeq& rhs);
    bool equal(Handle A, Handle B);

    int getFirstIndexOfType(HandleSeq hs, Type T) const;
    bool symmetricLink(Type T);
    bool is_empty_link(Handle h);
    bool hasFalsum(HandleSeq hs);
    bool containsNegation(Handle ANDlink, Handle h);
    Type getTypeV(const tree<Vertex>& _target) const;

// ARI: ok to delete this? TODELETE
/*  void VariableMPforms(const atom& src, set<atom, lessatom_ignoreVarNameDifferences>& res,
                     set<subst>* forbiddenBindings);                 */
};

// ARI: you said that this isn't used, however NormalizingATW inherits from
// it...
/** Passes the atoms via FIM analyzer. To turn this off, set FIM=0 in Config.
*/
class FIMATW : public AtomTableWrapper
{
// TODELETE
//  std::map<atom,int,lessatom> node2pat_id;
    fim::pat_id next_free_pat_id;
public:
    /// Semi-haxx::
    fim::grim myfim;

//  FIMATW(combo::NMCore* core) : AtomTableWrapper(core), next_free_pat_id(30001) {}
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
    //Btr<set<Handle> > getHandleSet(Type,const string&,bool = false);

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
    //Btr<set<Handle> > getHandleSet(Type, const string&, bool = false);

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
