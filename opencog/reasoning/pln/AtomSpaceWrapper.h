/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Authors:
 * Ari Heljakka
 * Joel Pitt
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

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

#include <opencog/atomspace/TimeServer.h>

#include "PLN.h"

#include "iAtomSpaceWrapper.h"
#include "rules/Rule.h"
#include "utils/fim.h"
#include "utils/Singleton.h"

//! The value at which PLN considers a TV a binary True.
#define PLN_TRUE_MEAN 0.989

//using namespace boost::bimaps;

//#define GET_ATW ((AtomSpaceWrapper*) ::haxx::defaultAtomSpaceWrapper)*/
#define GET_ATW ((AtomSpaceWrapper*) ASW())

namespace reasoning
{
//! Construct a vtree around Handle h
vtree make_vtree(Handle h);

typedef std::pair<Handle,VersionHandle> vhpair;

/** The bridge between the OpenCog AtomSpace and PLN.
 *
 * <h2>Fixing Fresh = true</h2>
 * 
 * The original version of Probabilistic Logic Networks made improper use of a
 * parameter in Novemente called \b fresh when adding new atoms to the
 * AtomSpace. This allowed atoms to be added to the AtomSpace without checking
 * for duplicates. The end result is that atoms were no longer unique: nodes
 * with the same name and type, or links that had the same outgoing set and
 * type, could duplicated. OpenCog does not allow this behaviour.
 * 
 * To fix this, all accesses to the AtomSpace now occur through the
 * AtomSpaceWrapper, which presents fake Handles to PLN and interprets them to a
 * combination of a real \c Handle and a \c VersionHandle. A system of dummy
 * contexts is used to emulate the behaviour of allowing duplicate atoms in the
 * AtomSpace by storing multiple \c TruthValues within a CompositeTruthValue.
 *
 * The AtomSpaceWrapper did exist in the original PLN, but it was more for
 * carrying out normalisation and allowing different AtomSpace backends to be
 * used for efficiency reasons. 
 * 
 * Dummy contexts are represented as directed links. A root dummy context,
 * represented as a Concept Node with name "___PLN___" is used for the outgoing
 * set of dummy contexts in duplicate nodes. For links that are duplicates the
 * outgoing set of their dummy context consists of the dummy context of any
 * duplicate atoms they refer to, or the root dummy context if they refer to the
 * original version of an atom.
 * 
 * <h3> Node example </h3>
 *
 * If a node already exists, then a dummy context is created that links to the
 * root PLN dummy context.
 * 
 * E.g. say we have
 *
 * \code
 *  ConceptNode "x" <0.8, 0.9>
 * \endcode
 *
 * and we want to add a new node with the same name and type, but with TV
 * <0.5,0.5>. To achieve this we create a new dummy context link:
 *
 * \code
 *  dc <- OrderedLink (ConceptNode "___PLN___")
 * \endcode
 * 
 * \c dc is then used as the context for the new TV:
 * 
 * \code
 *  ConceptNode "x" <0.8, 0.9> [ Context dc <0.5, 0.5> ]
 * \endcode
 * 
 * <h3> Link example </h3>
 * 
 * If a link already exists, then the appropriate contexts for each outgoing
 * atom are linked by a new dummy context link (with the appropriate context
 * links of each atom in the outgoing set of the link in the same order as the
 * outgoing set of the existing link). This link's outgoing set/list is prefixed
 * by the root dummy context node if such a context to context link doesn't
 * exist, if it does exist then the context links are searched for the "bottom"
 * link and this is used as the prefix.
 * 
 * E.g. say we have an InheritanceLink composed of two ConceptNodes that both
 * have CompositeTruthValues:
 * 
 * \code
 *  dc_x <- OrderedLink (ConceptNode "__PLN__")
 *  dc_y <- OrderedLink (ConceptNode "__PLN__")
 *  InheritanceLink <0.8, 0.9>
 *      ConceptNode "x" <0.8, 0.9> [ Context dc_x <0.5, 0.5> ]
 *      ConceptNode "y" <0.8, 0.9> [ Context dc_y <0.1, 0.5> ]
 * \endcode
 * 
 * If we want to create another link with a different \c TruthValue, but between
 * the versions of x and y that are under the dummy context (instead of the
 * original TruthValues) then we get:
 * 
 * \code
 *  dc_x_y <- OrderedLink (dc_x dc_y)
 *  InheritanceLink <0.8, 0.9> [ Context dc_x_y <0.3, 0.5> ]
 *      ConceptNode "x" <0.8, 0.9> [ Context dc_x <0.5, 0.5> ]
 *      ConceptNode "y" <0.8, 0.9> [ Context dc_y <0.1, 0.5> ]
 * \endcode
 *
 * Then if we want to create yet another with a different truthvalue, we get:
 *
 * \code
 * dc_x_y_2 <- OrderedLink (dc_x_y dc_x dc_y)
 * InheritanceLink <0.8, 0.9> (Context dc_x_y_2 <0.777, 0.5>)
 *   ConceptNode "x" <0.8, 0.9> (Context dc_x <0.5, 0.5>)
 *   ConceptNode "y" <0.8, 0.9> (Context dc_y <0.1, 0.5>)
 * \endcode
 *
 */
class AtomSpaceWrapper : public iAtomSpaceWrapper
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
    
    //! This string is the prefix of PLN dummy context root Node
    std::string rootContext;

    // typedef bimap< unordered_set_of< vhpair >, set_of<Handle> > vhmap_t;
    // typedef vhmap_t::value_type vhmap_pair_t;
    //! Bidrectional map with right index being PLN "handle", left a <handle,
    //! versionhandle> pair.
    //!
    //! @todo Handle is used, but eventually all references to Handle in PLN
    //! should be replaced by pHandle (typedefed to long int) to ensure
    //! distinctness from general OpenCog Handles... which will 
    //! change from long int to another type in the future. Or, PLN should be
    //! adapted to use vhpairs directly instead of relying on the AtomSpaceWrapper.
    // vhmap_t vhmap;
    
    typedef std::map< Handle, vhpair > vhmap_t;
    typedef vhmap_t::value_type vhmap_pair_t;
    typedef std::map< vhpair, Handle > vhmap_reverse_t;
    typedef vhmap_reverse_t::value_type vhmap_reverse_pair_t;
    //! Instead of above bimap stuff, we will temporarily use two normal maps
    vhmap_t vhmap;
    vhmap_reverse_t vhmap_reverse;

    //! template to search a map for a value (instead of a key) this is to be
    //! removed once bimap is integrated
    //template < typename M, typename T > 
    //typename M::const_iterator findValueInMap (const M m, const T value) const
    //{
    //    typename M::const_iterator i;
    //    for (i = vhmap.begin(); i != vhmap.end(); i++) {
    //        if (i->second == value) {
    //            return i;
    //        }
    //    }
    //    return i;
    //}

    // TODO: +1000 to be safe, but should check that no extra types are
    // defined elsewhere and just use +1.
    //const static unsigned int mapOffset = NOTYPE + 100000;
    const static unsigned int mapOffset = (1 << (8 * sizeof(Type))); // NOTYPE+1

    //! Add a tree of non real atoms to AtomSpace.
    //! @param v of atoms to add.
    //! @param vi iterator to start adding atoms from
    //! @param tvn what truth value they should be given
    //! @param fresh allows atoms to be added with the same name/outgoing set
    //! @param managed some kind of mechanism to manage memory
    Handle addAtom(vtree& v, vtree::iterator vi, const TruthValue& tvn, bool fresh, bool managed);

    bool hasAppropriateContext(const Handle o, VersionHandle& vh, unsigned int i = 0) const;
    bool isSubcontextOf(const Handle sub, const Handle super);

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

    bool linkNotifications;

public:

    //! Convert a specific VersionHandled TruthValue to a fake handle
    Handle realToFakeHandle(const Handle h, const VersionHandle vh);
    //! Convert a a real handle into a fake handle for each VersionedHandled TV
    HandleSeq realToFakeHandle(const Handle hs);
    //! Convert a HandleSeq of fake handles to real, optionally expanding to
    //! include every VersionHandled TV in each real handle
    HandleSeq realToFakeHandles(HandleSeq hs, bool expand=false);
    //! Match each context in the outgoing set of "context" with the handles
    HandleSeq realToFakeHandles(Handle h, Handle context);

    vhpair fakeToRealHandle(const Handle f) const;

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

    Handle getOutgoing(const Handle h, const int i);

    //! Get the incoming set for an atom
    HandleSeq getIncoming(const Handle h);

    Type getType(const Handle h) const;
    std::string getName(const Handle h) const;

    //! Reset the AtomSpace
    void reset();

    //! Initialize new AtomSpaceWrapper with const universe size
    AtomSpaceWrapper();
    virtual ~AtomSpaceWrapper() {}

    //! Load axioms from given xml filename
    bool loadAxioms(const string& path);
    //! Load other axioms from given xml filename and optionally replace?
    bool loadOther(const std::string& path, bool replaceOld);

    //! Add atom from tree vertex
    Handle addAtom(tree<Vertex>&, const TruthValue& tvn, bool fresh=false,
            bool managed=true);
    //! Add link
    virtual Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
            bool fresh=false, bool managed=true)=0;
    //! Add node
    virtual Handle addNode(Type T, const std::string& name,
            const TruthValue& tvn, bool fresh=false, bool managed=true)=0;

    //! Remove Atom
    virtual bool removeAtom(Handle h);

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

    /** Add concrete atom using dummy contexts if it already exists
     * 
     * @note Contexts should have actual handles for links, or be empty for
     * nodes. This can be ensured by using the appropriate addNodeDC or
     * addNodeDC classes.
     */
    Handle addAtomDC(Atom &a, bool fresh, bool managed, HandleSeq contexts = HandleSeq());
    Handle getNewContextLink(Handle h, HandleSeq destContexts);

    Handle directAddLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
        bool fresh,bool managed);

    // Generate CrispTheoremRules for all crisp theorems
    void makeTheorems();

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
    bool isEmptyLink(Handle h);
    bool hasFalsum(HandleSeq hs);
    bool containsNegation(Handle ANDlink, Handle h);
    Type getTypeV(const tree<Vertex>& _target) const;

// TODELETE
//  combo::NMCore* getCore() const { return core; }
//  std::vector<atom> LoadAnds(const std::string& path);
//  int implicationConstruction();
//  bool hasFalsum(float minAllowedError = 0.5) { return false; }
//  void VariableMPforms(const atom& src, set<atom, lessatom_ignoreVarNameDifferences>& res,
//                     set<subst>* forbiddenBindings);

};

// singleton instance (following meyer's design pattern)
iAtomSpaceWrapper* ASW();

/** Passes the atoms via FIM analyzer. To turn this off, set FIM=0 in Config.
*/
class FIMATW : public AtomSpaceWrapper
{
    fim::pat_id next_free_pat_id;

// TODELETE
//  std::map<atom,int,lessatom> node2pat_id;
public:
    /// Semi-haxx::
    fim::grim myfim;

    FIMATW() : next_free_pat_id(30001) {}
    virtual ~FIMATW() {}

    Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
            bool fresh, bool managed=true);
    Handle addNode(Type T, const std::string& name, const TruthValue& tvn,
            bool fresh, bool managed=true);
// TODELETE:
//  FIMATW(combo::NMCore* core) : AtomSpaceWrapper(core), next_free_pat_id(30001) {}
};

/** Normalizes atoms before passing forward */
class NormalizingATW : public FIMATW
{
    NormalizingATW();

    template<typename T>
    bool cutVector(const vector<T>& src, int index, vector<T>& dest)
    {
        dest.clear();

        for (int i = 0; i < src.size(); i++)
            if (i != index)
                dest.push_back(src[i]);

        return index < src.size();
    }

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

    // TODELETE:
    //Btr<set<Handle> > getHandleSet(Type,const string&,bool = false);

};

/** Forwards the requests without normalizing */
class DirectATW : public AtomSpaceWrapper
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

// TODELETE
/*using namespace boost;
class LocalATW : public AtomSpaceWrapper, public Singleton<LocalATW>
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
