/*
 * Fri Feb 18 11:35:16 2005
 * Copyright (C) 2005  Ari A. Heljakka <heljakka at gmail.com>  / Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * Written by Joel Pitt <joel@fruitionnz.com>
 * All Rights Reserved
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

#ifndef PLN_UTILS_H
#define PLN_UTILS_H

#ifdef WIN32
#pragma warning (disable: 4786)
#endif

#include <map>
#include <vector>
#include <string>
#include <list>
#include <set>
#include <iostream>
#include <stack>
#include <math.h>
#include <functional>

#include <boost/variant.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

// XXX PLNUtils.h gets included in many files, this should be trimmed right
// down to save on compilation time, especially for all the Rule classes!
// CogServer.h imports lots of networking and boost::asio files so is a good
// start to improve compilation
// AtomSpace
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>

// This is a 3rd party tree template library
#include <opencog/util/tree.h>

#include <opencog/reasoning/pln/atom_types.h>
#include "PLN.h"
#include "FitnessEvaluator.h"

#include "AtomLookupProvider.h"

#include "utils/mva.h"

using namespace opencog;
using namespace opencog::pln;
using std::unary_function;

#undef STLhas
#define STLhas(container, entity) ((container).find(entity) != (container).end())
#define STLhas2(container, entity) (std::find((container).begin(), (container).end(), (entity)) != (container).end())

#define Btr boost::shared_ptr
#define DeclareBtr(__T, __varname) Btr< __T > __varname(new __T)

namespace haxx
{
const int STD_VARS = 100;
}

/// meta is a vtree wrapped in a boost::shared_ptr
typedef Btr<vtree> meta;
/// hpair is a pair of pHandles
typedef std::pair<pHandle, pHandle> hpair;

typedef std::set<pHandle> pHandleSet;
typedef pHandleSet::iterator pHandleSetIt;
typedef pHandleSet::const_iterator pHandleSetConstIt;

// defines print levels
#define SILENT          0
#define SHY             1
#define NORMAL          2
#define LOCAL_DEBUG     3
#define VERBOSE         4
#define DEBUG           5
extern int currentDebugLevel;
// Cprintf for log level output
//! @todo replace with OpenCog Logger system
int cprintf(int debugLevel, const char *format, ...);

//! @todo replace with OpenCog Logger system
void ReasoningLog(int l, std::string m);

//! @todo replace with OpenCog Logger system
#define LOG(l, m) ReasoningLog(l,m)
#define SET_LOG(s) _LOGPOS = s; _LOG_FIXED = true
#define LET_LOG _LOG_FIXED = false;
#define LOG_STR (_LOG_FIXED ? _LOGPOS : ( std::string(__FILE__) + ":" + i2str(__LINE__) ))

static inline std::string i2str(int d)
{
    return boost::lexical_cast<std::string>(d);
}

/// Convert a boost::variant into a Handle
#define v2h(v) boost::get<Handle>(v)

// Print out trees
std::string rawPrint(tree<Vertex>& t, tree<Vertex>::iterator top, int _rloglevel);
std::string rawPrint(tree<Vertex>::iterator top, int _rloglevel);

// Make a string with count # of the char c
std::string repeatc(const char c, const int count);

namespace opencog {
namespace pln {

FitnessEvaluatorT getFitnessEvaluator(std::string name);

pHandle _v2h(const Vertex& v);

struct AtomSpaceWrapper;

// Check whether things are equivalent?
bool unifiesTo(AtomSpaceWrapper* asw,
               const vtree& lhs, const vtree& rhs,
               std::map<pHandle, vtree>& Lbindings,
               std::map<pHandle, vtree>& Rbindings,
               bool allow_rhs_binding, Type VarType = FW_VARIABLE_NODE);

//! Check whether lhs_t and rhs_t are the same, allowing different names for
//! FWVariableNodes. Does not allow actually substituting an FWVariable with a
//! normal Atom.
bool unifiesWithVariableChangeTo(AtomSpaceWrapper* asw,
                                 const vtree & lhs_t, const vtree & rhs_t,
                                 std::map<pHandle, pHandle>& bindings);

typedef Btr< VertexSeq > VertexVector;
typedef Btr< std::set<Vertex> > VertexSet;

// What are PostConditions used for ???
template <class T>
class PostCondition
{
public:
    virtual bool operator()(const T& arg) const = 0;
    virtual ~PostCondition();
};

class NoCondition : public PostCondition<pHandle>
{
public:
    bool operator()(const pHandle& arg) const {
        return true;
    }
};

typedef std::map<pHandle, pHandle> bindingsT;
typedef std::map<pHandle, vtree>  bindingsVTreeT;

struct ModifiedVTree : public vtree {
    pHandle original_handle;

    ModifiedVTree()
            : original_handle(PHANDLE_UNDEFINED) {}

    ModifiedVTree(const vtree& rhs,
                  pHandle _original_handle = PHANDLE_UNDEFINED)
        : vtree(rhs), original_handle(_original_handle) {}
};

struct ModifiedBoundVTree : public ModifiedVTree {
    ModifiedBoundVTree(const ModifiedVTree& rhs)
        : ModifiedVTree(rhs) {}

    Btr<bindingsVTreeT> bindings;
};

class BoundVTree : public vtree
{
protected:
    vtree my_std_tree;
public:
    BoundVTree() {}

    // Joel: Had to add constructors to route to the parent
    // How this compiled in Novamente without them, I have no idea.
    BoundVTree(const vtree::iterator_base& i) : vtree(i) {}
    BoundVTree(const Vertex& v) : vtree(v) {}

    BoundVTree(const ModifiedVTree& rhs)
        : vtree(rhs) {}

    BoundVTree(const vtree& rhs, Btr<bindingsT> _bindings = Btr<bindingsT>())
        : vtree(rhs), bindings(_bindings) {}

    BoundVTree* Clone() const {
        return new BoundVTree(*this, this->bindings);
    }

    Btr<bindingsT> bindings;

    const vtree& getStdTree();
    unsigned long getFingerPrint();
    void createMyStdTree();
};

typedef Btr<BoundVTree> BBvtree;

template<typename TransformerT>
Btr<vtree> tree_transform(const vtree& vt_const, TransformerT transformer)
{
    Btr<vtree> ret(new vtree(vt_const));
    std::transform(vt_const.begin(), vt_const.end(), ret->begin(), transformer);
    return ret;
}

/**
 * constructed given a mapping (by reference) with same domain and codomain.
 * The operator takes a key in input, if that key is associated to a value
 * in the mapping the value is returned, otherwise the key is returned.
 *
 * @todo: this class seems useless (only used in commented code.
 * Its name is bad (mapper is hardly self explanatory).
 * And mbegin and mend is simply not used
 * I suggest to remove it in case mapper is not used anywhere,
 * which needs to be evaluated based on how consisten_bindingsVTreeT code
 * should work.
 */
template<typename T2, typename T>
struct mapper {
    T mbegin, mend;
    std::map<T2, T2>& m;

    mapper(std::map<T2, T2>& _m, T _mbegin, T _mend)
            : mbegin(_mbegin), mend(_mend), m(_m) {}
    T2 operator()(const T2& key) {
        T i = m.find(key);
        return (i == m.end()) ? key : i->second;
    }
};

/**
We may have a case like:
First bindings:
64 => Britney2:ConceptNode (11) <1.000000, 0.000000>   [209]
65 => Osama2:ConceptNode (11) <1.000000, 0.000000>     [210]
88 => :EvaluationLink (77) <0.800000, 0.200000>        [212]
  friendOf:PredicateNode (26) <1.000000, 0.000000>     [203]
  :ListLink (32) <1.000000, 0.000000>  [211]
     Britney2:ConceptNode (11) <1.000000, 0.000000>    [209]
     Osama2:ConceptNode (11) <1.000000, 0.000000>      [210]
2nd bindings:
88 => :EvaluationLink (77) <1.000000, 0.000000>        [281]
  friendOf:PredicateNode (26) <1.000000, 0.000000>     [203]
  :ListLink (32) <1.000000, 0.000000>  [280]
     $AAAAAAAAD:FWVariableNode (146) <1.000000, 0.000000>      [264]
     $AAAAAAAAE:FWVariableNode (146) <1.000000, 0.000000>      [265]

which must be found consistent when these guys are combined!
*/

template<typename T>
Btr<std::map<Vertex, Vertex> > toVertexMap(T mbegin, T mend)
{
    Btr<std::map<Vertex, Vertex> >
        ret(new std::map<Vertex, Vertex>(mbegin, mend));
    return ret;
}

//! Construct a vtree around Handle h
vtree make_vtree(pHandle h);

template<typename T1, typename bindContainerIterT, typename TM>
bool consistent(TM& b1, TM& b2, bindContainerIterT b1start, bindContainerIterT b1end, bindContainerIterT b2start, bindContainerIterT b2end)
{
    assert(b1.begin() == b1start);

    for (bindContainerIterT b = b2start;
            b != b2end;
            b++) {
        bindContainerIterT bit;

        if ((bit = b1.find(b->first)) != b1end &&
                !(bit->second == b->second)) {
            ///The same var bound different way. First virtualize them:

            vtree binder1(make_vtree(_v2h(b->second)));
            vtree binder2(make_vtree(_v2h(bit->second)));

            /// Then apply all bindings on both sides to both, to "normalize away" dependencies

            Btr<vtree> binder1A(tree_transform(binder1,   mapper<T1, bindContainerIterT>(b1, b1.begin(), b1.end())));
            Btr<vtree> binder1B(tree_transform(*binder1A, mapper<T1, bindContainerIterT>(b2, b2start, b2end)));
            Btr<vtree> binder2A(tree_transform(binder2,   mapper<T1, bindContainerIterT>(b1, b1start, b1end)));
            Btr<vtree> binder2B(tree_transform(*binder2A, mapper<T1, bindContainerIterT>(b2, b2start, b2end)));

            return *binder2B == *binder1B; //Check if it's still inconsistent.
        }
    }

    return true;
}

void print_binding(std::pair<Handle, vtree> i);

struct PLNexception {
    std::string msg;
    PLNexception(std::string _msg) : msg(_msg) {}
    const char* what() const {
        return msg.c_str();
    }
};

template<typename T1, typename T2, typename T3>
void insert_with_consistency_check(std::map<T1, T2>& m, T3 rstart, T3 rend)
{
    ///haxx::
    Btr<std::map<Vertex, Vertex> > mV(toVertexMap(m.begin(), m.end()));
    Btr<std::map<Vertex, Vertex> > rV(toVertexMap(rstart, rend));

    if (consistent<Vertex>(*mV, *rV, mV->begin(), mV->end(), rV->begin(), rV->end()))
        m.insert(rstart, rend);
    else {
        /*  puts("First bindings:");
          for_each(m.begin(), m.end(), &print_binding);
          puts("2nd bindings:");
          for_each(rstart, rend, &print_binding);*/

        throw PLNexception("InconsistentBindingException");
    }
}

void insert_with_consistency_check_bindingsVTreeT(std::map<pHandle, vtree>& m,
                                                  std::map<pHandle, vtree>::iterator rstart,
                                                  std::map<pHandle, vtree>::iterator rend);


typedef std::pair<std::string, pHandle> hsubst;

bool equal_vectors(pHandle* lhs, int lhs_arity, pHandle* rhs);

template<typename ATOM_REPRESENTATION_T>
struct weak_atom {
    ATOM_REPRESENTATION_T value;
    Btr<bindingsT> bindings;

    ATOM_REPRESENTATION_T GetValue() const {
        return value;
    }

    weak_atom( ATOM_REPRESENTATION_T _value,
               bindingsT* _bindings = NULL)
            : value(_value), bindings(_bindings) {}

    weak_atom( ATOM_REPRESENTATION_T _value,
               Btr<bindingsT> _bindings)
            : value(_value), bindings(_bindings) {}
    weak_atom() : bindings(new bindingsT) {}
    ~weak_atom() { }
    /// Shared ownership of bindings!
    weak_atom(const weak_atom& rhs)
            : value(rhs.value), bindings(rhs.bindings) {}
    /// Does not share ownership of bindings!!!
    weak_atom(const weak_atom& rhs, Btr<bindingsT> _bindings)
            : value(rhs.value) {
        this->bindings = Btr<bindingsT>(rhs.bindings ?
                                        new bindingsT(*rhs.bindings)
                                        : new bindingsT);
        insert_with_consistency_check(*this->bindings,
                                      _bindings->begin(), _bindings->end());
    }

    bool operator()(pHandle h);
    bool operator<(const weak_atom<ATOM_REPRESENTATION_T>& rhs) const {
        return value < rhs.value;
    }
    /// this method is used nowhere and where reading it is seems buggy
    void apply_bindings();
};

typedef weak_atom<Vertex> BoundVertex;
typedef std::set<Vertex> BasicVertexSet;
typedef std::vector<BoundVertex> BV_Vector;
typedef std::set<BoundVertex> BV_Set;


/**
 * TableGather retrieves links given a template
 * (_MP that stands for meta predicate)
 * Warning: apparently it does not work with node type
 * as root of the meta predicate
 * but it works well with a link type, see TableGatherUTest for an example
 */
struct TableGather : public std::set<weak_atom<Vertex> > {
    TableGather(tree<Vertex>& _MP, AtomSpaceWrapper* asw,
                const Type VarT = FW_VARIABLE_NODE, int index = -1);
    void gather(tree<Vertex>& _MP, AtomSpaceWrapper* asw,
                const Type VarT = FW_VARIABLE_NODE, int index = -1);
    // for debugging
    std::string toString() const;
};

typedef TableGather::iterator TableGatherIt;
typedef TableGather::const_iterator TableGatherConstIt;

bool getLargestIntersection(const std::set<pHandle>& keyelem_set, const std::set<pHandle>& link_set, pHandle& result);

std::string printTree(pHandle h, int level = 0, int LogLevel = 5);
std::string GetRandomString(int size);

bool equal(pHandle A, pHandle B);

Handle satisfyingSet(Handle h);

/**
 * Return the set of membership links of a given concept node
 * (or inheriting concept)
 * 
 * @param concept The pHandle of the concept node (or inheriting concept)
 * @param min_membershipStrength The minimum strength of the  membership
 *                               to be taken in consideration
 * @param min_membershipCount The minimum count of the membership
 * @param asw AtomSpaceWrapper where to look for the concept
 *        Note that it is not const because TableGather is not const
 *        it must be fixed
 * @return The pHandleSet containing all memberLinks of members of P and
 *         have their TV above the given threshold strength and count
 */
pHandleSet memberLinkSet(pHandle concept,
                         strength_t min_membershipStrength,
                         count_t min_membershipCount,
                         AtomSpaceWrapper* asw);

/**
 * Return the set of members of a given concept node (or inheriting concept)
 * 
 * @param concept The pHandle of the concept node (or inheriting concept)
 * @param min_membershipStrength The minimum strength of the membership
 *                               to be taken in consideration
 * @param min_membershipCount The minimum count of the membership
 * @param asw AtomSpaceWrapper where to look for the concept P
 *        Note that it is not const because TableGather is not const
 *        it must be fixed
 * @return The pHandleSet containing all elements that are member of concept and
 *         have their memberLink TV above the given threshold strength and count
 */
pHandleSet constitutedSet(pHandle concept,
                          strength_t min_membershipStrength,
                          count_t min_membershipCount,
                          AtomSpaceWrapper* asw);

/**
 * Return the set of members of a given memberLink set
 *
 * @param memberLinks memberLink set
 * @param asw AtomSpaceWrapper where to look for the concept P
 *        Note that it is not const because TableGather is not const
 *        it must be fixed
 * @return The pHandleSet containing all elements of the memberLink set
 */
pHandleSet constitutedSet(const pHandleSet& memberLinks,
                          AtomSpaceWrapper* asw);


template<typename T>
std::vector<T*> NewCartesianProduct( std::vector<std::vector<T> >& matrix);

bool MPunify(vtree& lhs_t,
             vtree::iterator lhs_ti,
             const pHandle rhs,
             bindingsT& bindings,
             bool* restart = NULL, const Type VarT = FW_VARIABLE_NODE);

struct atom;

// rhs is a real Handle
bool MPunifyHandle(pHandle lhs,
                   const atom& rhs,
                   bindingsT& bindings,
                   bool* restart = NULL, const Type VarT = FW_VARIABLE_NODE);

// rhs is a vector of atoms (i.e. an outgoing set). The atoms can be real handles or virtual
bool MPunifyVector(tree<Vertex>& lhs_t, tree<Vertex>::iterator lhs_top,
                   const std::vector<Btr<atom> >& rhsv,
                   bindingsT& bindings,
                   bool* restart = NULL, const Type VarT = FW_VARIABLE_NODE);

// rhs is an atom containing a real handle or virtual stuff.
bool MPunify1(tree<Vertex>& lhs_t, tree<Vertex>::iterator lhs_ti,
              const atom& rhs,
              bindingsT& bindings,
              bool* restart = NULL, const Type VarT = FW_VARIABLE_NODE);

template<typename OP1, typename OP2, typename ArgT, typename RetT>
class Concat
{
public:
    OP1 op1;
    OP2 op2;
    RetT operator()(ArgT& arg) {
        return op2(op1(arg));
    }
};

class GetHandle
{
public:
    GetHandle() {}
    pHandle operator()(const Vertex& v) {
        return boost::get<pHandle>(v);
    }
};

class DropVertexBindings
{
public:
    Vertex operator()(const BoundVertex& rhs) {
        if (!rhs.bindings)
            return rhs.value;

        bindingsT::const_iterator i = rhs.bindings->find(boost::get<pHandle>(rhs.value));

        /// The variable may be bound to another variable, so we have to call this recursively.

        return (i == rhs.bindings->end())
               ? rhs.value
               : DropVertexBindings()(BoundVertex(i->second, rhs.bindings));
//     : Vertex(i->second);
    }
};

void convertTo(const VertexSeq& args, std::unique_ptr<Handle>& ret);
void convertTo(const VertexVector& args, std::unique_ptr<Handle>& ret);
void convertTo(const std::vector<BoundVertex>& args, std::unique_ptr<Handle>& ret);
void convertTo(const std::set<BoundVertex>& args, std::unique_ptr<Handle>& ret);
void convertTo(const VertexSet& args, std::unique_ptr<Handle>& ret);
void convertTo(const std::vector<Handle>& args, std::unique_ptr<Handle>& ret);
void convertTo(const VertexVector& args, Handle*& ret);
void convertTo(const VertexSeq& args, Handle*& ret);
void convertTo(const std::vector<BoundVertex>& args, Handle*& ret);
void convertTo(const std::vector<BoundVertex>& args, HandleSeq& ret);
void convertTo(const std::set<BoundVertex>& args, Handle*& ret);
void convertTo(const VertexSet& args, Handle*& ret);
void convertTo(const std::vector<Handle>& args, Handle*& ret);

struct getOutgoingFun : public std::binary_function<pHandle, int, pHandle> {
    getOutgoingFun(AtomSpaceWrapper* _asw) : asw(_asw) {}
    pHandle operator()(pHandle h, int i);
private:
    AtomSpaceWrapper* asw;
};

#define getTypeFun std::bind1st(std::mem_fun(&AtomSpaceWrapper::getType), GET_ASW)
#define getTypeVFun bind(getTypeFun, bind(&_v2h, _1))

#define getFW_VAR(vt) (std::find_if((vt).begin(), (vt).end(), \
                               bind(std::equal_to<Type>(), \
                                    bind(getTypeFun, bind(&_v2h, _1)), \
                                    (Type)FW_VARIABLE_NODE )))

#define hasFW_VAR(vt) (getFW_VAR(vt) != (vt).end())

const char* Type2Name(Type t);
std::string condensed_form(const atom& a);

void bind(BoundVTree& bbvt, hpair new_bind);
meta bind_vtree(vtree &targ, const std::map<pHandle, pHandle>& binds);
void bind_Bvtree(meta arg, const bindingsVTreeT& binds);
void removeRecursionFromHandleHandleMap(bindingsT& ret_bindings);

void printBinding(const std::pair<const Handle, Handle> p);
bool equalVariableStructure(const vtree& lhs, const vtree& rhs);
bool equalVariableStructure2(BBvtree lhs, BBvtree rhs);

void printSubsts(BoundVertex a, int LogLevel);
pHandle make_real(vtree& vt);

template<typename T, typename T2>
T2 second(const std::pair<T, T2>& p)
{
    return p.second;
}
template<typename T, typename T2>
T first(const std::pair<T, T2>& p)
{
    return p.first;
}

template<typename T, typename T2>
void removeRecursionFromMap(T mbegin, T mend)
{
    T mnext = mbegin;
    cprintf(4, "removeRecursionFromMap...\n");
    while (mnext != mend) {
        T2 mnodenext = mnext->second.begin();

        while (mnodenext != mnext->second.end()) {
            T next_mapping = find_if(mbegin, mend, bind(std::equal_to<pHandle>(), _v2h(*mnodenext), bind(&first<pHandle, vtree>, _1)) ); //mbegin->second == _1);

            if (next_mapping != mend) {
                assert(next_mapping->first == _v2h(*mnodenext));
                mnext->second.replace(mnodenext, next_mapping->second.begin());

                /// The replace call invalidates the iterator. Re-initialize:
                mnodenext = mnext->second.begin();
            } else
                mnodenext++;
        }
        mnext++;
    }
    cprintf(4, "removeRecursionFromMap OK!\n");
}

//probably deprecated
#define NewNode(_T, _NAME) mva(GET_ASW->addNode(_T, _NAME, TruthValue::TRIVIAL_TV(), false))
#define makemeta(atom_description) meta(new tree<Vertex>(atom_description))

template<typename T1, typename T2>
bool overlap(T1 & abegin, T1& aend, T2& b)
{
    for (T1 ai = abegin; ai != aend; ai++)
        if (STLhas(b, *ai))
            return true;

    return false;
}

meta ForceAllLinksVirtual(meta target);
meta ForceRootLinkVirtual(meta target);

void print_progress();

bool RealHandle(meta _target, Btr<std::set<BoundVertex> > result_set);

template<typename MapIteratorT, typename MapItemT>
void removeRecursionFromMapSimple(MapIteratorT mbegin, MapIteratorT mend)
{
    cprintf(4, "removeRecursionFromMap...\n");

    MapIteratorT mnext = mbegin, next_mapping;

    while (mnext != mend) {
        if ( (next_mapping = find_if(mbegin, mend, bind(std::equal_to<MapItemT>(), mnext->second, bind(&first<MapItemT, MapItemT>, _1))))
                != mend) {
            assert(next_mapping->first == mnext->second);
            mnext->second = next_mapping->second;
        } else
            mnext++;
    }

    cprintf(4, "removeRecursionFromMap OK!\n");
}

void makeHandletree(pHandle h, bool fullVirtual, tree<Vertex>& ret);

bool substitutableTo(pHandle from, pHandle to,
                     std::map<pHandle, pHandle>& bindings);

bool IsIdenticalHigherConfidenceAtom(pHandle a, pHandle b);

/**
 * ostream functions
 */
template<typename Out>
Out& operator<<(Out& out, const std::pair<pHandle, vtree>& pv) {
    return out << pv.first << " => " << pv.second << std::endl;
}
template<typename Out>
Out& operator<<(Out& out, const bindingsVTreeT& bvt) {
    return ostreamContainer(out, bvt, "\n");
}
template<typename Out>
Out& operator<<(Out& out, const ModifiedVTree& mvt) {
    out << "original_handle = " << mvt.original_handle << std::endl;
    return out << static_cast<vtree>(mvt) << std::endl;
}
template<typename Out>
Out& operator<<(Out& out, const ModifiedBoundVTree& mbvt) {
    out << static_cast<ModifiedVTree>(mbvt) << std::endl;
    return out << mbvt.bindings << std::endl;
}

} // ~namespace pln
} // namespace opencog


const int __LLEVEL = 4;

template<typename SetIterT, typename VectorT, typename InputIterT, typename OutputIterT>
void createCombinations( const VectorT& head,
                         InputIterT input_vector_begin,
                         InputIterT input_vector_end,
                         InputIterT this_arg_number,
                         SetIterT this_arg_number_begin,
                         SetIterT this_arg_number_end,
                         OutputIterT& output_iter)
{
    if (this_arg_number != input_vector_end) {
        bool existsConsistentBinding = false;
        assert(*this_arg_number);

        for (SetIterT i = this_arg_number_begin;
                i != this_arg_number_end; i++) {
            VectorT next_head(head);
            next_head.push_back(*i);

            if (true) {
                InputIterT temp_i = this_arg_number;
                if (++temp_i == input_vector_end)
                    existsConsistentBinding = true;
            } else
                continue;


            InputIterT next_arg_number = this_arg_number +
                                         ((this_arg_number + 1 != input_vector_end)
                                          ? 1
                                          : 0);

            createCombinations(
                next_head,
                input_vector_begin,
                input_vector_end,
                this_arg_number + 1,
                (*next_arg_number)->begin(),
                (*next_arg_number)->end(),
                output_iter);
        }
        if (!existsConsistentBinding)
            return;
//    throw std::string("No consistent binding of Rule input std::vector elements was found!");
    } else {
        *(output_iter++) = head;
    }
}

void removeRecursion(std::vector<Btr<opencog::pln::BoundVertex> >& multi_input_vector);

// input: a vector where each set is one possibility
// output: a set where each vector is one (ordered) combination of possibilities
template<typename VectorT, typename InputIterT, typename OutputIterT>
void expandVectorSet( InputIterT multi_input_vector_begin,
                      InputIterT multi_input_vector_end,
                      OutputIterT output_iter)
{

    VectorT empty_head;

    try {
        createCombinations( empty_head,
                            multi_input_vector_begin,
                            multi_input_vector_end,
                            multi_input_vector_begin,
                            (*multi_input_vector_begin)->begin(),
                            (*multi_input_vector_begin)->end(),
                            output_iter);
        /*LOG(__LLEVEL, "Combinations:-------\n");
           for (set<VectorT>::iterator i = output_set.begin();
                   i!= output_set.end();
                   i++)
           {
            LOG(__LLEVEL, "\nNext vector:\n");
            for (uint j=0; j<i->size(); j++)
            {
             const Handle* h_ptr = v2h(&((*i)[j].value));
             if (!h_ptr)
             { LOG(__LLEVEL,"(non-Handle)"); }
             else
              cprintf(__LLEVEL,"[%d]\n", (int)*h_ptr);
              //printTree(*h_ptr,0,3);

             if ((*i)[j].bindings)
              cprintf(__LLEVEL, "HAS BINDINGS %d!\n",(*i)[j].bindings->size());
            }
            LOG(__LLEVEL, "XXXXXXXXXXXXXXXx-------\n");
           }*/

    } catch (std::string s) {
        puts(s.c_str());
    }
}

#define vt2h(vtreeprovider) _v2h(*(vtreeprovider).getVtree().begin())

#endif
