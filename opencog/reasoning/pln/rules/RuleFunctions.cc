/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#include <opencog/util/platform.h>
#include "../PLN.h"

#include "Rule.h"
#include "Rules.h"
#include "RuleFunctions.h"
#include "../AtomSpaceWrapper.h"
#include "../PLNatom.h"
#include "../BackInferenceTreeNode.h"

using std::vector;
using std::set;
using std::map;

/*namespace haxx
{
    map<int, int> total2level1;
    /// Must invert the formula I*(I+1)/2 for 2-level incl. excl. in OR rule
    int contractInclusionExclusionFactorial(int total)
    {
        std::map<int, int>::iterator T2L = total2level1.find(total);

        if(T2L != total2level1.end())
            return T2L->second;

        int val=0;
        int i=0;
        
        for (i = total2level1.size(); val != total; i++)
            val = total2level1[i] = i*(i+1)/2;

        return i-1;
    }
}*/

namespace opencog { namespace pln {

#ifndef WIN32
float max(float a, float b) { return ((a>b)?a:b); }
#endif

Vertex CreateVar(iAtomSpaceWrapper* asw, const std::string& varname)
{
    // Use the AtomSpaceWrapper to create a new node representing
    // the variable and called varname, don't try to replace it
    // if it already exists
    pHandle ret = asw->addNode(FW_VARIABLE_NODE, varname,
                               TruthValue::TRIVIAL_TV(), false);
    
    cprintf(4, "CreateVar: added fwvar node %s [%u]\n", varname.c_str(), ret);
    return Vertex(ret);
}

Vertex CreateVar(iAtomSpaceWrapper* asw)
{ 
    // create a variable node with a name as a generated 10 char string
    return CreateVar(asw, "$"+GetRandomString(10));
}

Rule::setOfMPs makeSingletonSet(const Rule::MPs& mp)
{
    return make_singleton_set<Rule::setOfMPs>(mp);
}

// Redundant, hopefully:
BBvtree atomWithNewType(pHandle h, Type T, AtomSpaceWrapper* asw)
{
    assert(asw->inheritsType(T, LINK));
    vector<pHandle> children = asw->getOutgoing(h);
    BBvtree ret_m(new BoundVTree(mva((pHandle)T)));
    foreach(pHandle c, children)
    {
        ret_m->append_child(ret_m->begin(), mva(c).begin());
    }
    
    return ret_m;
}
    
BBvtree atomWithNewType(const tree<Vertex>& v, Type T, AtomSpaceWrapper* asw)
{
    pHandle *ph = boost::get<pHandle>(&*v.begin());
//! @todo just call the overloaded Vertex version below
    if (!ph || asw->isType(*ph)) //Virtual: just replace the root node
    {
    
        BBvtree ret_m(new BoundVTree(v));
        *ret_m->begin() = Vertex((pHandle)T);
        return ret_m;
    }
    else //Real: construct a new tree from T root and v's outgoing set.
    {
        assert(asw->inheritsType(T, LINK));

        vector<pHandle> children = asw->getOutgoing(*ph);
        BBvtree ret_m(new BoundVTree(mva((pHandle)T)));
        foreach(pHandle c, children)
        {
            ret_m->append_child(ret_m->begin(), mva(c).begin());
        }
        
        return ret_m;
    }   
}

BBvtree atomWithNewType(const Vertex& v, Type T, AtomSpaceWrapper* asw)
{
    const pHandle *ph = boost::get<pHandle>(&v);
    if (!ph || asw->isType(*ph)) //Virtual: just replace the root node
    {
    
        BBvtree ret_m(new BoundVTree(v));
        *ret_m->begin() = Vertex((pHandle)T);
        return ret_m;
    }
    else //Real: construct a new tree from T root and v's outgoing set.
    {
        assert(asw->inheritsType(T, LINK));

        vector<pHandle> children = asw->getOutgoing(*ph);
        BBvtree ret_m(new BoundVTree(mva((pHandle)T)));
        foreach(pHandle c, children)
        {
            ret_m->append_child(ret_m->begin(), mva(c).begin());
        }
        
        return ret_m;
    }   
}

/// haxx:: \todo VariableNodes not memory-managed.
bool UnprovableType(Type T)
{
    return  //inheritsType(T, OR_LINK) ||
            GET_ASW->inheritsType(T, CONCEPT_NODE) ||
            GET_ASW->inheritsType(T, FORALL_LINK);
}

template<Type T>
pHandle Join(pHandle* h, int N, AtomSpaceWrapper& asw)
{
    vector<pHandle> hs(h, h+N);
    return asw.addLink(T, hs, TruthValue::TRUE_TV(), false);
}

template<Type T, typename ASW>
pHandle Join(pHandle h1, pHandle h2, ASW& asw)
{
    pHandle h[] = { h1, h2 };
    return Join<T>(h, 2, asw);
}

void insertAllAndCombinations(set<atom, lessatom_ignoreVarNameDifferences> head, vector<atom> tail, set<atom, lessatom_ignoreVarNameDifferences>& And_combinations)
{
    int totalSize = tail.size() + head.size();

    if (totalSize < 2)
        return;

    if (tail.empty())
    {
        atom neBoundVertex(AND_LINK, 0);
        neBoundVertex.hs = vector<Btr<atom> >(head.size());

        if (!head.empty()) {
            foreach(const atom &a, head)
                neBoundVertex.hs.push_back(Btr<atom>(new atom(a)));
//          copy(head.begin(), head.end(), neBoundVertex.hs.begin());
        }

        And_combinations.insert(neBoundVertex);
    }
    else
    {
        atom natom = tail.back();
        tail.pop_back();
        
        /// Combinations without natom

        if (totalSize >= 3) //If AndLink producible even if 1 element was removed.
            insertAllAndCombinations(head, tail, And_combinations);
        
        head.insert(natom);

        /// Combinations with natom

        insertAllAndCombinations(head, tail, And_combinations);
    }
}

template<typename C>
void createPermutation(vector<C> seed, set< vector<C> >& result, int index)
{
    if (index >= seed.size()-1)
        result.insert(seed);
    else
    {
        for (int i = index+1; i < seed.size(); i++)
        {
            createPermutation<C>(seed, result, index+1);

            //C temp = seed[i];
            std::swap<C>(seed[index], seed[i]);

            createPermutation<C>(seed, result, index+1);

            std::swap<C>(seed[index], seed[i]); //Cancel
        }
    }
}

template<typename C >
set<vector<C> >* newCreatePermutations(vector<C> seed)
{
    set<vector<C> >* ret = new set<vector<C> >;
    createPermutation<C>(seed, *ret, 0);

    return ret;
}

pHandle UnorderedCcompute(AtomSpaceWrapper *asw, Type linkT,
                          const ArityFreeFormula& fN,
                          pHandle* premiseArray, const int n, pHandle CX,
                          bool fresh)
{
    // tvs is to contain the TVs of the premises from premiseArray
    TVSeq tvs(n);
    for (int i = 0; i < n; i++)
        tvs[i] = asw->getTV(premiseArray[i]);

    // compute TV result
    TruthValue* retTV = fN.compute(tvs);

    // copy premiseArray into outgoing
    pHandleSeq outgoing(premiseArray, premiseArray + n);

    // add link
    pHandle ret = asw->addLink(linkT, outgoing, *retTV, fresh);

    /// Release
    delete retTV;
    
    return ret;
}

Rule::setOfMPs PartitionRule_o2iMetaExtra(meta outh, bool& overrideInputFilter,
                                          Type OutLinkType,
                                          AtomSpaceWrapper* asw)
{
        const int N = outh->begin().number_of_children();
        
        if (!asw->isSubType(_v2h(*outh->begin()), OutLinkType)
            || N <= MAX_ARITY_FOR_PERMUTATION)
            return Rule::setOfMPs();

        Rule::MPs ret;

        int parts = N / MAX_ARITY_FOR_PERMUTATION;
        int remainder = N % MAX_ARITY_FOR_PERMUTATION;

        vector<atom> hs;
        tree<Vertex>::sibling_iterator ptr = outh->begin(outh->begin());
        BBvtree root(new BoundVTree);
        root->set_head((pHandle)OutLinkType);

        for (int i = 0; i <= parts; i++)
        {
            int elems = ( (i<parts) ? MAX_ARITY_FOR_PERMUTATION : remainder);
            
            BoundVTree elem(mva((pHandle)AND_LINK));
            
            if (elems>1)
            {
                for (int j = 0; j < elems; j++)
                    elem.append_child(elem.begin(), *ptr++);
                root->append_child(root->begin(), elem.begin());
            }
            else if (elems>0)
                root->append_child(root->begin(), *ptr++);
        }

        assert(ptr == outh->end(outh->begin()));

        ret.push_back(root);

        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
}

#if DISABLE_FOR_NOW
//! @todo Update to BoundVTree
boost::shared_ptr<set<BoundVertex > > attemptDirectAndProduction(iAtomSpaceWrapper *destTable,
                                        const meta& outh,
                                        ArityFreeAndRule* rule)
    {
        if (!isSubType(v2h(*outh->begin()), AND_LINK))
            return Rule::setOfMPs();

        tree<Vertex>::iterator top = outh->begin();
        const int N = outh->begin(top);

        set<atom, lessatom_ignoreVarNameDifferences> qnodes, And_combinations;
        vector<atom> units;

        /// Add Individual nodes
    
        for (tree<Vertex>::sibling_iterator i = outh->begin(top);
                                            i!= outh->end  (top);
                                            i++)
            qnodes.insert(v2h(*i));

        And_combinations = qnodes;

        /// Add AndLinks

        insertAllAndCombinations(set<atom, lessatom_ignoreVarNameDifferences>(), outh.hs, And_combinations);

        /// Create ring MP

        meta gatherMP(new BoundVTree((Handle)OR_LINK));

        if (!And_combinations.empty())
        {
            //gatherMP.hs = vector<atom>(And_combinations.size());
            for (set<atom, lessatom_ignoreVarNameDifferences>::iterator a =
                And_combinations.begin(); a != And_combinations.end(); a++)
            {
                gatherMP.append_child(  gatherMP.begin(),
                                        a->maketree().begin());
            }
            //copy(And_combinations.begin(), And_combinations.end(), gatherMP.hs.begin());
        }

        /// Gather

        Handle premises[50];
        
        TableGather nodeInstances(gatherMP, destTable, -1);
                                                
        for (int ni=0;ni < nodeInstances.size(); ni++)
        {
            premises[ni] = nodeInstances[ni];

            atom a(nodeInstances[ni]);

            if (a.T == AND_LINK)
                for (vector<atom>::iterator i = a.hs.begin(); i!=a.hs.end();i++)
                    qnodes.erase(*i);
            else
                qnodes.erase(a);
        }

        if (qnodes.empty()) //All found
            return rule->compute(premises, nodeInstances.size());
        else
            return Vertex((Handle)NULL);

}
#endif

/*
  [IntInh A B].tv.s = [ExtInh Ass(B) Ass(A)].tv.s
  Ass(A) = { c: [p(C|A) - P(C|-A)] > 0 }
  Wrong ExtInh direction?
  */

#ifdef DISABLED_FOR_NOW
//! @todo Update to tree<Vertex> && new rule storage (not ASW)
Handle Ass(AtomSpaceWrapper *asw, Handle h, vector<Handle>& ret)
{
    TableGather g(mva((Handle)INHERITANCE_LINK,
        mva(CreateVar(asw)),
        mva(h)));
    TableGather reverseg(mva((Handle)INHERITANCE_LINK,
        mva(h),
        mva(CreateVar(asw))));

    for(vector<Handle>::iterator i = reverseg.begin(); i != reverseg.end(); i++)
    {
        Handle hs[] = { *i };
        *i = AtomSpaceWrapper::rule[Inversion_Inheritance]->compute(hs,1);
    }
    
    ret = reverseg;
    for (vector<Handle>::iterator j = g.begin(); j != g.end();j++)
        ret.push_back(*j);

    string ass_name = nm->getName(h);

    LOG(0, "Ass of " + ass_name + ":");
    for_each<vector<Handle>::iterator, handle_print<0> >(ret.begin(),
        ret.end(), handle_print<0>());

    atom ass(CONCEPT_NODE, "ASS(" + (ass_name.empty()?GetRandomString(10):ass_name) + ")");

    /// What's the correct TV of the ASS conceptnode?

    for (vector<Handle>::iterator k = ret.begin(); k != ret.end();k++)
    {
        TruthValuePtr tv = destTable->getTV(*k);

        printTree(
            destTable->addAtom(
                atom(MEMBER_LINK, 2,
                    new atom(nm->getOutgoing(*k)[0]),
                    new atom(ass)
                ),
                *tv,
                true);  
//              false);
            )
            ,0,0
        );
    }

    return ass.attach(destTable);
}
#endif

void pr2(std::pair<pHandle, vtree> i);
void pr(std::pair<pHandle, pHandle> i);

/**
 * Unary operator to converter a given source into a given destination
 * provided by the constructor of the operator struct
 */
template<typename T>
struct converter  : public std::unary_function<T,void> 
{
    T src, dest;
    converter(const T& _src, const T& _dest) : src(_src), dest(_dest) {}
    void operator()(T& _arg) {
        if (_arg == src)
            _arg = dest; 
    }
};

Btr<vtree> convert_all_var2fwvar(const vtree& vt, iAtomSpaceWrapper* table)
{
    Btr<vtree> ret(new vtree(vt));
/// USE THIS IF YOU WISH TO CONVERT VARIABLE_NODEs to FW_VARIABLE_NODEs!
    vtree::iterator vit = ret->begin();

    while( (vit = std::find_if(ret->begin(), ret->end(),
                               bind(std::equal_to<Type>(),
                                    bind(getTypeVFun, _1),
                                    VARIABLE_NODE)
                               )
            ) != ret->end())
    {
        Vertex new_var = CreateVar(table);
        for_each(ret->begin(), ret->end(), converter<Vertex>(*vit, new_var));
#if 0
        printTree(v2h(*vit),0,4);
        rawPrint(*ret, ret->begin(), 4);
#else 
        NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
        printer.print(_v2h(*vit), 4);
        printer.print(ret->begin(), 4);
#endif
    }

    return ret;
}

//Btr<ModifiedVTree> convertToModifiedVTree(Btr<vtree> vt)
Btr<ModifiedVTree> convertToModifiedVTree(pHandle h, Btr<vtree> vt)
{
    return Btr<ModifiedVTree>(new ModifiedVTree(*vt, h));
}
/*Btr<ModifiedVTree> convertToModifiedVTree(vtree& vt)
{
    return Btr<ModifiedVTree>(new ModifiedVTree(vt));
}*/

#if DISABLED_FOR_NOW
template<typename T2, typename T3>
struct mapper
{
    const map<T2, T3>& m;

    mapper(const map<T2, T3> _m) : m(_m) {}
    T3 operator()(const T2& key)
    {
        map<T2, T3>::const_iterator i = m.find(key);
        if (i == m.end())
            return key;
        else
            return i->second;
    }
};
#endif

template<typename T, typename T2>
bool stlhasi(const T& c, const T2& k)
{
    return c.find(k) != c.end();
}

/**
    After a StrictCrispU Node has been created here, it contains some newly converted FW_VARs.
    We have to convert them to FW_VARs to avoid complexities of having to deal with 2 types of
    vars everywhere.
    But we must be able to bind those vars later, by spawning a new copy of this Inference Node.
    But if we do that, then we must be able to connect the FW_VARs with this node.
    But we cannot simply fix the conversion mapping from VARs to FW_VARs for each ForAllLink,
    because we want to be able to use the same ForAllLink several times during the same inference
    chain!
    
*/

template<typename T1, typename bindContainerIterT, typename TM>
bool consistent_bindingsVTreeT(TM& b1, bindContainerIterT b2start, bindContainerIterT b2end)
{
    for (bindContainerIterT b = b2start; b!= b2end; b++)
    {
        bindContainerIterT bit;

        if ((bit = b1.find(b->first)) != b1.end() &&
            !(bit->second == b->second))
        {
            return false;

#if DISABLED_FOR_NOW
            ///The same var bound different way. First virtualize them:

            vtree binder1(make_vtree(v2h(b->second)));
            vtree binder2(make_vtree(v2h(bit->second)));

            /// Then apply all bindings on both sides to both, to "normalize away" dependencies

            Btr<vtree> binder1A(tree_transform(binder1,   mapper<T1, bindContainerIterT>(b1, b1start, b1end)));
            Btr<vtree> binder1B(tree_transform(*binder1A, mapper<T1, bindContainerIterT>(b2, b2start, b2end)));
            Btr<vtree> binder2A(tree_transform(binder2,   mapper<T1, bindContainerIterT>(b1, b1start, b1end)));
            Btr<vtree> binder2B(tree_transform(*binder2A, mapper<T1, bindContainerIterT>(b2, b2start, b2end)));

            return *binder2B == *binder1B; //Check if it's still inconsistent.
#endif
        }
    }

    return true;
}

void insert_with_consistency_check_bindingsVTreeT(map<pHandle, vtree>& m,
                                                  map<pHandle, vtree>::iterator rstart,
                                                  map<pHandle, vtree>::iterator rend)
{
    if (consistent_bindingsVTreeT<Vertex>(m, rstart, rend))
        m.insert(rstart, rend);
    else
        throw PLNexception("InconsistentBindingException");
}

Btr<set<pHandle> > ForAll_handles;


Btr<ModifiedBoundVTree> FindMatchingUniversal(meta target,
                                              pHandle ForAllLink,
                                              AtomSpaceWrapper* asw)
{
    cprintf(4,"FindMatchingUniversal...");
    
    // candidate is a ModifiedVTree containing the body of the ForAll
    // with all its variables replaced by FW_VAR
    Btr<ModifiedVTree> candidate =
        convertToModifiedVTree(ForAllLink, 
                               convert_all_var2fwvar
                               (make_vtree(asw->getOutgoing(ForAllLink, 1)),
                                asw));

    Btr<bindingsVTreeT> bindsInUniversal(new bindingsVTreeT),
                        bindsInTarget(new bindingsVTreeT);

    // debug print
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(candidate->begin(), 4);
    printer.print(target->begin(), 4);
    // ~debug print

    if (!unifiesTo(asw, *candidate, *target,
                   *bindsInUniversal, *bindsInTarget, true)) //, VARIABLE_NODE))
        return Btr<ModifiedBoundVTree>();

    // debug print
    printer.print(candidate->begin(), 4);
    printer.print(candidate->original_handle, 4);

    cprintf(4,"universal binds:");
    for_each(bindsInUniversal->begin(), bindsInUniversal->end(), pr2);
    cprintf(4,"target binds:");
    for_each(bindsInTarget->begin(), bindsInTarget->end(), pr2);
    // ~debug print

    Btr<ModifiedBoundVTree> BoundUniversal(new ModifiedBoundVTree(*candidate));
    /// bindsInTarget will later become constraints/pre-bindings of the new BIN.
    /// UPDATE: THIS FEATURE WAS CANCELCED:
    /// BoundUniversal->bindings = bindsInTarget;
    BoundUniversal->bindings.reset(new bindingsVTreeT);

    Btr<bindingsVTreeT> bindsCombined(new bindingsVTreeT(*bindsInUniversal));
    insert_with_consistency_check_bindingsVTreeT(*bindsCombined,
                                                 bindsInTarget->begin(),
                                                 bindsInTarget->end());

    ///Remove FW_VAR => FW_VAR mappings. WARNING! Potentially goes to infinite loop.
    removeRecursionFromMap<bindingsVTreeT::iterator, vtree::iterator>(bindsCombined->begin(), bindsCombined->end());

    // debug print
    cprintf(4,"\ncombined binds:");
    for_each(bindsCombined->begin(), bindsCombined->end(), pr2);
    // ~debug print

#if PREVIOUS_IMPLEMENTATION
    /// Previously:
    /// Combined bindings are applied to the candidate
    for(vtree::iterator v = BoundUniversal->begin();
        v!= BoundUniversal->end();) {
        bindingsVTreeT::iterator it = bindsCombined->find(v2h(*v));
        if (it != bindsCombined->end())
            {
                BoundUniversal->replace(v, it->second.begin());
                v = BoundUniversal->begin();
            }
        else
            ++v;
    }
#endif

    // debug print
    printer.print(BoundUniversal->begin(), 4);
    // ~debug print

#if PREVIOUS_IMPLEMENTATION
    /// Previously:
    /// All recursion was removed from the combined binds, but only the binds pertaining
    /// to the variables that occurred in target are relevant and will hence be included.

    for (bindingsVTreeT::iterator it = bindsCombined->begin();
         it!= bindsCombined->end(); it++)
        if (STLhas(*bindsInTarget, it->first))
            (*BoundUniversal->bindings)[it->first] = it->second;
#endif

    /// Now:
    /// I changed the unifiesTo function so that lhs/rhs binding distinction was blurred.
    //! @todo Verify that this is correct: now including bindings on both sides!

    BoundUniversal->bindings = bindsCombined;

    // debug print
    cprintf(4,"\nresult binds:");
    for_each(BoundUniversal->bindings->begin(), BoundUniversal->bindings->end(),
             pr2);
    // ~debug print

    return BoundUniversal;
}

Btr< set<Btr<ModifiedBoundVTree> > > FindMatchingUniversals(meta target,
                                                            AtomSpaceWrapper* asw)
{
    DeclareBtr(set<Btr<ModifiedBoundVTree> >, ret);

    if (!ForAll_handles) {
        ForAll_handles = asw->getHandleSet(FORALL_LINK, "");
        puts("Recreated ForAll_handles");
    }

    foreach(pHandle h, *ForAll_handles) {
        Btr<ModifiedBoundVTree> BoundUniversal = FindMatchingUniversal(target,
                                                                       h, asw);
        if (BoundUniversal)
            ret->insert(BoundUniversal);
    }

    return ret;
}

//! ARI: am I correct in thinking this is a debug version of the above?
#if 0
Btr< set<Btr<ModifiedBoundVTree> > > FindMatchingUniversals(meta target,
                                                            AtomSpaceWrapper* asw)
{
    DeclareBtr(set<Btr<ModifiedBoundVTree> >, ret);
    Btr< set<Btr<ModifiedVTree> > > ForAllLinkRepo;

    if (!ForAllLinkRepo)
    {
        if (!ForAll_handles)
            ForAll_handles = asw->getHandleSet(FORALL_LINK, "");

        ForAllLinkRepo.reset(new set<Btr<ModifiedVTree> >);

        /// Handle => vtree and then convert VariableNodes to FW_VariableNodes

        transform(ForAll_handles->begin(), ForAll_handles->end(),
                  inserter(*ForAllLinkRepo, ForAllLinkRepo->begin()),
                  bind(&convertToModifiedVTree,
                       _1,
                       bind(&convert_all_var2fwvar,
                            bind(&make_vtree, bind(getOutgoingFun(), _1, 1)),
                            asw))); /// Ignore arg #0 ListLink)

#if 0
        foreach(const Btr<ModifiedVTree>& mvt, *ForAllLinkRepo)
            rawPrint(*mvt, mvt->begin(),3);
#else 
        NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
        foreach(const Btr<ModifiedVTree>& mvt, *ForAllLinkRepo)
            printer.print(mvt->begin(),3);
#endif
        /*foreach(Handle h, *ForAll_handles)
            printTree(h, 0,0);*/
    }

    foreach(const Btr<ModifiedVTree>& candidate, *ForAllLinkRepo)
    {
        Btr<bindingsVTreeT> bindsInUniversal(new bindingsVTreeT),
                            bindsInTarget(new bindingsVTreeT);

#if 0
        rawPrint(*candidate, candidate->begin(), 4);
        rawPrint(*target, target->begin(), 4);
#else 
        printer.print(candidate->begin(), 4);
        printer.print(target->begin(), 4);
#endif

        if (unifiesTo(*candidate, *target, *bindsInUniversal, *bindsInTarget, true)) //, VARIABLE_NODE))
        {
#if 0
            rawPrint(*candidate, candidate->begin(), 4);
            printTree(candidate->original_handle,0,4);
#else 
            printer.print(candidate->begin(), 4);
            printer.print(candidate->original_handle, 4);
#endif

            cprintf(4,"universal binds:");
            for_each(bindsInUniversal->begin(), bindsInUniversal->end(), pr2);
            cprintf(4,"target binds:");
            for_each(bindsInTarget->begin(), bindsInTarget->end(), pr2);

            Btr<ModifiedBoundVTree> BoundUniversal(new ModifiedBoundVTree(*candidate));
            /// bindsInTarget will later become constraints/pre-bindings of the new BIN.
            /// UPDATE: THIS FEATURE WAS CANCELCED:
            /// BoundUniversal->bindings = bindsInTarget;
            BoundUniversal->bindings.reset(new bindingsVTreeT);

            Btr<bindingsVTreeT> bindsCombined(new bindingsVTreeT(*bindsInUniversal));
            insert_with_consistency_check_bindingsVTreeT(*bindsCombined,
                                                         bindsInTarget->begin(),
                                                         bindsInTarget->end());

            ///Remove FW_VAR => FW_VAR mappings. WARNING! Potentially goes to infinite loop.
            removeRecursionFromMap<bindingsVTreeT::iterator, vtree::iterator>(bindsCombined->begin(), bindsCombined->end());

            cprintf(4,"\ncombined binds:");
            for_each(bindsCombined->begin(), bindsCombined->end(), pr2);

#if PREVIOUS_IMPLEMENTATION
            /// Previously:
            /// Combined bindings are applied to the candidate
            for(vtree::iterator v = BoundUniversal->begin();
                v!= BoundUniversal->end();)
            {
                bindingsVTreeT::iterator it = bindsCombined->find(v2h(*v));
                if (it != bindsCombined->end())
                {
                    BoundUniversal->replace(v, it->second.begin());
                    v = BoundUniversal->begin();
                }
                else
                    ++v;
            }
#endif

#if 0
            rawPrint(*BoundUniversal, BoundUniversal->begin(), 4);
#else 
            printer.print(BoundUniversal->begin(), 4);
#endif

#if PREVIOUS_IMPLEMENTATION
            /// Previously:
            /// All recursion was removed from the combined binds, but only the binds pertaining
            /// to the variables that occurred in target are relevant and will hence be included.

            for (bindingsVTreeT::iterator it = bindsCombined->begin();
                 it!= bindsCombined->end(); it++)
                if (STLhas(*bindsInTarget, it->first))
                    (*BoundUniversal->bindings)[it->first] = it->second;
#endif

            /// Now:
            /// I changed the unifiesTo function so that lhs/rhs binding distinction was blurred.
            //! @todo Verify that this is correct: now including bindings on both sides!

            BoundUniversal->bindings = bindsCombined;


            cprintf(4,"\nresult binds:");

            for_each(BoundUniversal->bindings->begin(), BoundUniversal->bindings->end(), pr2);

            ret->insert(BoundUniversal);
        }
    }

    return ret;
}
#endif

}} // namespace opencog { namespace pln {

