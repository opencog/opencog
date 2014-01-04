/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#include <sys/types.h>
#include <time.h>
#include <queue>
#include <vector>

#include <boost/config.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/lru_cache.h>

#include "PLN.h"

#include "iAtomSpaceWrapper.h"
#include "rules/Rule.h"
#include "utils/fim.h"
#include "utils/Singleton.h"

// uncomment this out to experiment with contextual inference
// #define CONTEXTUAL_INFERENCE

//! The value at which PLN considers a TV a binary True.
#define PLN_TRUE_MEAN 0.989

//using namespace boost::bimaps;
#define GET_ASW ASW(NULL)

class AtomSpaceWrapperUTest;

namespace opencog {

class AtomSpaceImpl;

namespace pln {

// there exists a method to print (overloading <<) it see at the end of the file
typedef std::pair<Handle,VersionHandle> vhpair;

/** The bridge between the OpenCog AtomSpace and PLN.
 *
 * <h3>Fixing Fresh = true</h3>
 * 
 * The original version of Probabilistic Logic Networks made improper use of a
 * parameter in Novemente called \b fresh when adding new atoms to the
 * #opencog::AtomSpace. This allowed atoms to be added to the AtomSpace without
 * checking for duplicates. The end result is that atoms were no longer unique:
 * nodes
 * with the same name and type, or links that had the same outgoing set and
 * type, could duplicated. OpenCog does not allow this behaviour.
 * 
 * To fix this, all accesses to the AtomSpace now occur through the
 * AtomSpaceWrapper, which presents fake Handles to PLN and interprets them to a
 * combination of a real #opencog::Handle and a #opencog::VersionHandle. A
 * system of dummy contexts is used to emulate the behaviour of allowing
 * duplicate atoms in the AtomSpace by storing multiple #opencog::TruthValues
 * within a #opencog::CompositeTruthValue.
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
 * <h4> Node example </h4>
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
 * <h4> Link example </h4>
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
 *  dc_x <- OrderedLink (ConceptNode "___PLN___")
 *  dc_y <- OrderedLink (ConceptNode "___PLN___")
 *  InheritanceLink <0.8, 0.9>
 *      ConceptNode "x" <0.8, 0.9> [ Context dc_x <0.5, 0.5> ]
 *      ConceptNode "y" <0.8, 0.9> [ Context dc_y <0.1, 0.5> ]
 * \endcode
 * 
 * If we want to create another link with a different \c TruthValue, but between
 * the versions of x and y that are under the dummy context (instead of the
 * original \c TruthValues) then we get:
 * 
 * \code
 *  dc_x_y <- OrderedLink ((ConceptNode "___PLN___") dc_x dc_y)
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
 * @todo short-term: intercept FW_VARIABLE_NODES and prevent them from being put in the
 * AtomSpace
 * @todo long-term: replace FW_VARIABLES_NODES with a string in the Vertex type.
 * @todo Normalisation of AtomSpace: EquivalenceLink <-> 2 x ImplicationLinks
 * @todo Explicit representation of ContextLinks (since AtomSpaceWrapper
 * maps each context TV to a separate Handle) or function to get context atom.
 */
class AtomSpaceWrapper : public iAtomSpaceWrapper
{
    // To allow test introspection
    friend class ::AtomSpaceWrapperUTest;

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
    const std::string rootContext;
    //! The handle of the root context
    Handle rootContextHandle;

    // typedef bimap< unordered_set_of< vhpair >, set_of<Handle> > vhmap_t;
    // typedef vhmap_t::value_type vhmap_pair_t;
    //! Bidrectional map with right index being PLN "handle", left a <handle,
    //! versionhandle> pair.
    //!
    //! @todo pHandle (typedefed above) is used to ensure
    //! distinctness from general OpenCog Handles... which is now defined  
    //! as a class. In the future. PLN may be adapted to use vhpairs
    //! directly instead of relying on the AtomSpaceWrapper.
    typedef std::map< pHandle, vhpair > vhmap_t;
    typedef vhmap_t::value_type vhmap_pair_t;
    typedef std::map< vhpair, pHandle > vhmap_reverse_t;
    typedef vhmap_reverse_t::value_type vhmap_reverse_pair_t;
    //! Instead of above bimap stuff, we will temporarily use two normal maps
    vhmap_t vhmap;
    vhmap_reverse_t vhmap_reverse;

    /**
     * Add a tree of non real atoms to AtomSpace.
     * @param v of atoms to add.
     * @param vi iterator to start adding atoms from
     * @param tvn what truth value they should be given
     * @param tvn what truth value they should be given
     * @param fresh allows atoms to be added with the same name/outgoing set.
     *              If fresh == false and the atom already exist then the new
     *              truth value is merged (via TruthValue::merge) with the old.
     *              Otherwise (fresh == true) then a new dummy context
     *              is associated to that new truth value.
     * @return The pHandle corresponding to the added atom
     */
    pHandle addAtom(vtree& v, vtree::iterator vi, const TruthValue& tvn,
                    bool fresh);

    //! Used by getImportantHandles
    struct compareSTI {
        AtomSpace *atomspace;

        compareSTI(AtomSpace* _a): atomspace(_a) {};

        //! @warning uses real atomspace handles in comparison
        bool operator()(const Handle& a, const Handle& b) {
            return atomspace->getAV(a).getSTI() >
                atomspace->getAV(b).getSTI();
        }
    };
    
    // For monitoring additions to the AtomSpace from outside of PLN
    bool handleAddSignal(AtomSpaceImpl *as, Handle h); //!< Signal handler for atom adds.
    bool handleRemoveSignal(AtomSpaceImpl *as, Handle h); //!< Signal handler for atom removals.

    //! Whether AtomSpaceWrapper is listening for AtomSpace signals.
    bool watchingAtomSpace;

    boost::signals2::connection c_add; //! Connection to add atom signals
    boost::signals2::connection c_remove; //! Connection to remove atom signals

protected:

    //! Keep track of what FW Variables are in the system
    std::map<std::string,pHandle> variableShadowMap;

    //! Add Link via dummy contexts method
    pHandle addLinkDC(Type t, const pHandleSeq& hs, const TruthValue& tvn,
                      bool fresh);
    //! Add Node via dummy contexts method
    pHandle addNodeDC(Type t, const std::string& name, const TruthValue& tvn,
                      bool fresh);

    /** Add concrete atom using dummy contexts if it already exists
     * 
     * @note Contexts should have actual handles for links, or be empty for
     * nodes. This can be ensured by using the appropriate addNodeDC or
     * addNodeDC classes.
     * @param a Atom to be added
     * @param fresh allows atoms to be added with the same name/outgoing set.
     *              If fresh == false and the atom already exist then the new
     *              truth value is merged (via TruthValue::merge) with the old.
     *              Otherwise (fresh == true) then a new dummy context
     *              is associated to that new truth value.
     * @param context @todo add comment
     * 
     * @return The handle of the atom added.
     */
    pHandle addAtomDC(Atom &a, bool fresh,
                      HandleSeq contexts = HandleSeq());

    pHandle directAddLink(Type T, const pHandleSeq& hs, const TruthValue& tvn,
                          bool fresh);

    AtomSpace *atomspace;
public:

    inline AtomSpace* getAtomSpace() const { return atomspace; };

    //! Change whether AtomSpaceWrapper is listening for AtomSpace signals.
    void setWatchingAtomSpace(bool watching);
    //! Whether AtomSpaceWrapper is listening for AtomSpace signals.
    void isWatchingAtomSpace();

    //! Check whether a pHandle is known to the AtomSpaceWrapper
    bool isValidPHandle(const pHandle h) const;
    //! Convert a specific VersionHandled TruthValue to a pln handle
    // Note that realToFakeHandle cannot be const because it modifies vhmap
    // Or vhmap should be declared mutable
    pHandle realToFakeHandle(const Handle h, const VersionHandle vh);
    //! Convert a a real handle into a fake handle for each VersionedHandled TV
    pHandleSeq realToFakeHandle(const Handle hs);
    //! Convert real handles to pln pHandleSeq , optionally expanding to
    //! include every VersionHandled TV in each real handle
    pHandleSeq realToFakeHandles(const HandleSeq& hs, bool expand=false);
    //! Convert real handles to pln pHandleSeq, all under a given context
    pHandleSeq realToFakeHandles(const HandleSeq& hs, Handle context);

    /** Get the pHandle that represents the non-contextual/primary TV the real handle
     * behind ph.
     */
    pHandle getPrimaryFakeHandle(pHandle ph);

    vhpair fakeToRealHandle(const pHandle f) const;

    //! Which XML files have been loaded by PLN to populate the AtomSpace.
    //! No longer accessed (but it is updated).
    std::set<std::string> loadedFiles;
    
    //! Debug method to display nodes
    void DumpCoreNodes(int logLevel);
    //! Debug method to display links
    void DumpCoreLinks(int logLevel);
    //! Debug method to display all atoms of Type T 
    void DumpCore(Type T);

    //! return the size of the universe
    //! @todo get the universe from the real AtomSpace if USizeMode == REAL_SIZE
    unsigned int getUniverseSize() const { return USize; }

    //! set the universe size (only if USizeMode == CONST_SIZE)
    void setUniverseSize(USizeMode_t _USizeMode, unsigned int _USize)
    {
        assert(_USizeMode == CONST_SIZE);
        USizeMode = _USizeMode;
        USize = _USize;
    }

    //! Get pHandles with type t and name str optionally subtypes as well
    virtual Btr<std::set<pHandle> > getHandleSet(Type T,
                                                 const std::string& name,
                                                 bool subclass = false);
    //! Get pHandle corresponding to the context-free Handle of node
    //! with type t and name str
    pHandle getHandle(Type t,const std::string& str);
    //! Get handle of link with type t and outgoing set 
    pHandle getHandle(Type t,const pHandleSeq& outgoing);
    // helper methods for getHandle
    inline pHandle getHandle(Type t, pHandle ha) {
        pHandleSeq oset;
        oset.push_back(ha);
        return getHandle(t, oset);
    }
    inline pHandle getHandle(Type t, pHandle ha, pHandle hb) {
        pHandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        return getHandle(t, oset);
    }

    pHandleSeq getOutgoing(const pHandle h);

    pHandle getOutgoing(const pHandle h, const int i);

    //! Get the incoming set for an atom
    pHandleSeq getIncoming(const pHandle h);

    Type getType(const pHandle h) const;
    std::string getName(const pHandle h) const;

    //! Reset the AtomSpace
    void reset();

    //! Initialize new AtomSpaceWrapper with const universe size
    AtomSpaceWrapper(AtomSpace* as);
    virtual ~AtomSpaceWrapper();

    //! Load axioms from given xml filename
    bool loadAxioms(const std::string& path);
    //! Load other axioms from given xml filename and optionally replace?
    bool loadOther(const std::string& path, bool replaceOld);

    /**
     * Update the TruthValue of a given pHandle. If the given handle
     * does not point an atom (UNDEFINED or Type) then an OC_ASSERT is raised.
     *
     * @param h Handle to apply the update on
     * @param tv TruthValue to update
     * @param fresh If fresh == false then the new truth value is merged
     *              (via TruthValue::merge) with the old.
     *              Otherwise (fresh == true) then a new dummy context
     *              is associated to that new truth value.
     *
     * @return The pHandle updated. Note that it is note necessarily
     *         equal to h because it associate automatically
     *         a new dummy context to it, which gets translated a different
     *         pHandle.
     */
    pHandle updateTV(pHandle h, const TruthValue& tv, bool fresh);

    /**
     * Add atom from tree vertex
     * @param tvn what truth value they should be given
     * @param fresh allows atoms to be added with the same name/outgoing set
     * @param       If fresh == false then the new truth value is merged
     *              (via TruthValue::merge) with the old.
     *              Otherwise (fresh == true) then a new dummy context
     *              is associated to that new truth value.
     * @return the pHandle corresponding to the atom added
     */
    pHandle addAtom(tree<Vertex>&, const TruthValue& tvn, bool fresh=false);

    //! Add link, pure virtual
    //! @param tvn what truth value they should be given
    //! @param fresh allows atoms to be added with the same name/outgoing set
    virtual pHandle addLink(Type T, const pHandleSeq& hs, const TruthValue& tvn,
                            bool fresh=false)=0;

    // helper methods for addLink
    inline pHandle addLink(Type t, pHandle ha, const TruthValue& tvn,
                           bool fresh=false)
    {
        pHandleSeq oset;
        oset.push_back(ha);
        return addLink(t, oset, tvn, fresh);
    }
    inline pHandle addLink(Type t, pHandle ha, pHandle hb,
                           const TruthValue& tvn, bool fresh=false)
    {
        pHandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        return addLink(t, oset, tvn, fresh);
    }
    inline pHandle addLink(Type t, pHandle ha, pHandle hb, pHandle hc,
                           const TruthValue& tvn, bool fresh=false)
    {
        pHandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        return addLink(t, oset, tvn, fresh);
    }

    //! Add node, pure virtual
    //! @param tvn what truth value they should be given
    //! @param fresh allows atoms to be added with the same name/outgoing set
    virtual pHandle addNode(Type T, const std::string& name,
                            const TruthValue& tvn, bool fresh=false)=0;


    //! Remove fake handle from vhmap and vhmap_reverse
    void removeFakeHandle(pHandle h);

    //! Remove Atom
    virtual bool removeAtom(pHandle h);

    // return a random handle of type T
    //pHandle getRandomHandle(Type T);

    //! get a number of high STI handles
    pHandleSeq getImportantHandles(int number);

    //! Whether the handle h has high enough TruthValue to be consider a binary True.
    //! @todo move to TruthValue classes
    bool binaryTrue(pHandle h);

    //! @todo Move the below conversion tools in a Converter class
    
    //! Wrap h in a NOT_LINK and return that 
    pHandle invert(pHandle h);
    //! Convert from And to Or link
    pHandle And2OrLink(pHandle& andL, Type _AndLinkType, Type _OrLinkType);
    //! Convert from Equivalence to Implication link
    hpair Equi2ImpLink(pHandle& exL);
    //! Convert from Existance to For All link
    pHandle Exist2ForAllLink(pHandle& exL);
    //! Convert from Or to And link
    pHandle Or2AndLink(pHandle& andL);
    //! Convert from And to Or link
    pHandle And2OrLink(pHandle& andL);
    
    Handle getNewContextLink(Handle h, HandleSeq destContexts);

    //! Whether to generate CrispTheoremRules for all crisp theorems
    //! and add them to CrispTheoremRule::thms.
    bool archiveTheorems;

    //! Generate CrispTheoremRules for all crisp theorems in AtomSpace
    //! and add to CrispTheoremRule::thms.
    void makeCrispTheorems();

    //! Generate a CrispTheoremRule for crisp theorem pointed to by
    //! p then and add to CrispTheoremRule::thms.
    //! A crisp theorem has the format:
    //! ImplicationLink ( And < tv 1.0 > (...), result )
    void makeCrispTheorem(pHandle p);

    //! returns whether the type of h is T or inherits from T
    bool isSubType(pHandle h, Type T);
    //! returns whether type subT has superT as a parent type
    bool inheritsType(Type subT, Type superT) const;

    //tv_summary_t getTV(pHandle h) const;
    TruthValuePtr getTV(pHandle h) const;
    strength_t getMean(pHandle h) const;
    confidence_t getConfidence(pHandle h) const;

    void setTV(pHandle h, const TruthValue& tv);

    bool isType(const pHandle h) const;

    const TimeServer& getTimeServer() const;

    /** Retrieve the arity of a given link */
    int getArity(pHandle) const;

    pHandleSeq filter_type(Type t);

    bool equal(const HandleSeq& lhs, const HandleSeq& rhs);
    bool equal(Handle A, Handle B);

    int getFirstIndexOfType(pHandleSeq hs, Type T) const;
    bool symmetricLink(Type T);
    bool isEmptyLink(pHandle h);
    bool hasFalsum(pHandleSeq hs);
    bool containsNegation(pHandle Andlink, pHandle h);

    //! return the type of the root of _target
    Type getTypeV(const tree<Vertex>& _target) const;

    bool allowFWVarsInAtomSpace;

    //for debugging
    std::string vhmapToString() const;
    std::string pHandleToString(pHandle ph) const;
};

/*
 * Operator to check that the outgoing pHandles of 2 pHandles
 * both at a given index are equal
 */
struct EqOutgoing : public unary_function<pHandle, bool> {
    EqOutgoing(const pHandle h, const int idx, AtomSpaceWrapper* asw)
        : _h(h), _idx(idx), _asw(asw)
    {}
    bool operator()(const pHandle h)
    {
        return _asw->getOutgoing(_h, _idx) == _asw->getOutgoing(h, _idx);
    }
private:
    const pHandle _h;
    const int _idx;
    AtomSpaceWrapper* _asw;
};

/**
 * Singleton instance of AtomSpaceWrapper
 * (following meyer's design pattern)
 */
AtomSpaceWrapper* ASW(AtomSpace *a = NULL);

/** Passes the atoms via FIM analyzer. To turn this off, set FIM=0 in Config.
*/
class FIMATW : public AtomSpaceWrapper
{
    fim::pat_id next_free_pat_id;

public:
    /// Semi-haxx::
    fim::grim myfim;

    FIMATW(AtomSpace *a) : AtomSpaceWrapper(a), next_free_pat_id(30001) {}
    virtual ~FIMATW() {}

    pHandle addLink(Type T, const pHandleSeq& hs, const TruthValue& tvn,
                    bool fresh);
    pHandle addNode(Type T, const std::string& name, const TruthValue& tvn,
                    bool fresh);
};

/** Normalizes atoms before passing forward */
class NormalizingATW : public FIMATW
{
    NormalizingATW(AtomSpace *a);

    template<typename T>
    bool cutVector(const std::vector<T>& src, int index, std::vector<T>& dest)
    {
        dest.clear();

        for (int i = 0; i < src.size(); i++)
            if (i != index)
                dest.push_back(src[i]);

        return index < src.size();
    }

public:
    virtual ~NormalizingATW() {}
       
    static NormalizingATW& getInstance(AtomSpace *a = NULL)
    {
        static NormalizingATW* instance = NULL;
        if (a == NULL)
            throw RuntimeException(TRACE_INFO,"Atomspace pointer was NULL"); 
        if (instance != NULL) delete instance;
        instance = new NormalizingATW(a);
        return *instance;
    }

    
    pHandle addLink(Type T, const pHandleSeq& hs, const TruthValue& tvn,
            bool fresh);
    pHandle addNode(Type T, const std::string& name, const TruthValue& tvn,
            bool fresh);
};

}} //~namespace opencog::pln

namespace std {

// overload of operator<< to print vhpair
template<typename Out>
Out& operator<<(Out& out, const opencog::pln::vhpair& vhp) {
    out << "(Handle=" << vhp.first
        << ", VersionHandle=" << vhp.second << ")";
    return out;
}

// overload of operator<< to print std::pair<pHandle,vhpair>
//
// Warning: if you templatize this function like above gcc will
// complaim arguing that it is ambiguous with the one above
inline std::ostream& operator<<(std::ostream& out,
                                const std::pair<opencog::pln::pHandle,
                                                opencog::pln::vhpair>& pvp) {
    out << "(pHandle=" << pvp.first
        << ", vhpair=" << pvp.second << ")";
    return out;
}
} // ~namespace std

#endif
