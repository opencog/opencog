#include <platform.h>
#include "PLN.h"

#include "Rule.h"
#include "Rules.h"
#include "AtomTableWrapper.h"
#include "Ptlatom.h"
#include "BackInferenceTreeNode.h"

#define HYPRULE_MAKES_ZERO_CONFIDENCE_ATOMS false

using namespace std;

//haxx::
bool UnificationRuleResultFreshness = true; //false;

namespace haxx
{
    map<string, map<Handle, Handle> > scholemFunctions;
    extern bool AllowFW_VARIABLENODESinCore;
    reasoning::BITNodeRoot* bitnoderoot;
    
    /// \todo This data must persist even if the BITNodeRoot is deleted.
    extern map<Handle,vector<Handle> > inferred_from;
    extern map<Handle,reasoning::Rule*> inferred_with;
}


#ifndef WIN32
  float max(float a, float b) { return ((a>b)?a:b); }
#endif

namespace reasoning
{ 

// Redundant, hopefully:
BBvtree atomWithNewType(Handle h, Type T)
{
    assert(inheritsType(T, LINK));
    AtomSpace *nm = CogServer::getAtomSpace();
    vector<Handle> children = nm->getOutgoing(h);
    BBvtree ret_m(new BoundVTree(mva((Handle)T)));
    foreach(Handle c, children)
    {
        ret_m->append_child(ret_m->begin(), mva(c).begin());
    }
    
    return ret_m;
}
    
BBvtree atomWithNewType(const tree<Vertex>& v, Type T)
{
    Handle *ph = v2h(&*v.begin());
// TODO just call the overloaded Vertex version below
    AtomSpace *nm = CogServer::getAtomSpace();
    if (!ph || !nm->isReal(*ph)) //Virtual: just replace the root node
    {
    
        BBvtree ret_m(new BoundVTree(v));
        *ret_m->begin() = Vertex((Handle)T);
        return ret_m;
    }
    else //Real: construct a new tree from T root and v's outgoing set.
    {
        assert(inheritsType(T, LINK));

        vector<Handle> children = nm->getOutgoing(*ph);
        BBvtree ret_m(new BoundVTree(mva((Handle)T)));
        foreach(Handle c, children)
        {
            ret_m->append_child(ret_m->begin(), mva(c).begin());
        }
        
        return ret_m;
    }   
}

BBvtree atomWithNewType(const Vertex& v, Type T)
{
    Handle *ph = (Handle*) v2h(&v);
    AtomSpace *nm = CogServer::getAtomSpace();
    if (!ph || !nm->isReal(*ph)) //Virtual: just replace the root node
    {
    
        BBvtree ret_m(new BoundVTree(v));
        *ret_m->begin() = Vertex((Handle)T);
        return ret_m;
    }
    else //Real: construct a new tree from T root and v's outgoing set.
    {
        assert(inheritsType(T, LINK));

        vector<Handle> children = nm->getOutgoing(*ph);
        BBvtree ret_m(new BoundVTree(mva((Handle)T)));
        foreach(Handle c, children)
        {
            ret_m->append_child(ret_m->begin(), mva(c).begin());
        }
        
        return ret_m;
    }   
}
/// haxx:: \todo VariableNodes not memory-managed.

extern int varcount;

Vertex CreateVar(iAtomTableWrapper* atw, std::string varname)
{
    Handle ret = atw->addNode(FW_VARIABLE_NODE,varname,
        TruthValue::TRIVIAL_TV(),false,false);
    
cprintf(4, "CreateVar Added node as NEW: %s / [%lu]\n", varname.c_str(), (ulong) ret);

    varcount++;

    return Vertex(ret);
}

Vertex CreateVar(iAtomTableWrapper* atw)
{
    return CreateVar(atw, "$"+GetRandomString(10));
}

BoundVertex Rule::compute(const vector<BoundVertex>& h, Handle CX) const
{
    bindingsT h_b;
    AtomSpace *nm = CogServer::getAtomSpace();
    foreach(const BoundVertex& bv, h)
    {
        if (!nm->isReal(v2h(bv.value)))
        {
            puts("!isReal");
            printf("%lu\n", v2h(bv.value)); 
            NMPrinter(NMP_ALL)(v2h(bv.value), -10);
        }

        assert(nm->isReal(v2h(bv.value)));
        assert(nm->getType(v2h(bv.value)) != FW_VARIABLE_NODE);
    }
    
    vector<Vertex> rule_args(h.size());
    transform(h.begin(), h.end(), rule_args.begin(), DropVertexBindings());

    Btr<bindingsT> bindings_of_all_args(new bindingsT);

    for(vector<BoundVertex>::const_iterator bv = h.begin(); bv != h.end(); bv++) {
        if (bv->bindings) {
            insert_with_consistency_check(*bindings_of_all_args, bv->bindings->begin(), bv->bindings->end());

          cprintf(3,"Bind [%d]:\n", v2h(bv->value));

            foreach(hpair phh, *bv->bindings)
            {
                printTree(phh.first,0,3);
                cprintf(3,"=>");
                printTree(phh.second,0,3);
            }
        }
    }



    return BoundVertex(compute(rule_args, CX), bindings_of_all_args);
}

Rule::Rule(reasoning::iAtomTableWrapper *_destTable,
            bool _FreeInputArity,
            bool _computable,
            std::string _name)
: RULE_INPUT_ARITY_MAX(15), freeInputArity(_FreeInputArity), destTable(_destTable),
computable(_computable), priority(10.0f), name(_name) 
{
}

Rule::Rule() : RULE_INPUT_ARITY_MAX(15)
{ }

bool Rule::validate(const vector<Vertex>& h) const
{
    if (freeInputArity)
        return true;

    const uint n = h.size();
    
    vector<Vertex> myh(h);
    
    if (n != inputFilter.size())
    {
		cprintf(0,"Rule::validate FALSE. Input vector  size: %d\n", n);
#if 0        
        for (int i=0;i<n;i++)
            printTree(v2h(h[i]),0,0);
#else 
        //NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
        //  bind(NMPrinter(), bind(&_v2h, _1))(h[0]);
    //      NMPrinter()(bind(v2h(h[0]));
    //  for_each(h.begin(), h.end(), NMPrinter(NMP_HANDLE|NMP_TYPE_NAME));
    //        for (int i=0;i<n;i++)
    //   printer.print(v2h(h[i]));
#endif             
        return false;
    }
    
    typedef weak_atom<boost::shared_ptr<tree<Vertex> > > vertex_wrapper;

    for (uint i = 0; i < n; i++)
    {
        vertex_wrapper mp(inputFilter[i]);
        
        if (!(mp(v2h(myh[i]))))
        {
            uint    r=0;
            for (   r = i+1;
                    r < n && !mp(v2h(myh[r]));
                    r++);
            if (r < n)
                std::swap(myh[i], myh[r]);
            else
            {
				cprintf(0,"Rule::validate FALSE. Input vector:\n");
#if 0                
                for (int j=0;j<n;j++)
                    printTree(v2h(h[j]),0,0);
#else 
                NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
                for (uint j=0;j<n;j++)
                    printer.print(v2h(h[j]));
#endif             
                
                return false;
            }
        }
    }

    return true;
}

Rule::setOfMPs Rule::o2iMeta(meta outh) const
{
    assert(computable);
    
    Rule::setOfMPs ret;
    
    const int N = outh->begin().number_of_children();
    if (N < 1)
        return ret;
/*  if (Handle* ph = v2h(*outh.begin()))
        return makeSingletonSet(vector2<meta>(
            new tree<Vertex>()
        ));*/
    
    bool overrideInputFilter = false;
    setOfMPs extraFilter = o2iMetaExtra(outh, overrideInputFilter);

    if (extraFilter.empty())
        return setOfMPs();

    set<boost::shared_ptr<MPs> >::iterator e;
    
    /*      /// Ensure that there're no Handles involved! (Or, it's a special unprovable type,
    /// in which case there may be a hack involved.
    
      for (e = extraFilter.begin(); e!=extraFilter.end(); e++)
      {
      for (Rule::MPs::iterator m = (*e)->begin(); m != (*e)->end(); m++)
      {
      puts(":::");
      assert(!(*m)->real || UnprovableType((*m)->T));
      }
}*/
    
    if (overrideInputFilter)
        return extraFilter;
    
/*  TODO: Update this to the new types system
    
    for (e = extraFilter->begin(); e!=extraFilter.end(); e++)
    {
        assert((*e)->size() == inputFilter.size());
    
        boost::shared_ptr<MPs> ret1(new MPs);
        
        for (uint i = 0; i < inputFilter.size(); i++)
            ret1->push_back(boost::shared_ptr<atom>(
                new atom(AND_LINK, 2,
                    new atom(*inputFilter[i]),
                    new atom(*(**e)[i])
                )
            ));
        
        ret->insert(ret1);
    }*/
    
    return ret;
}

BoundVertex Rule::computeIfValid(const vector<Vertex>& h, Handle CX) const
{
    if (validate(h))
        return compute(h,CX);
    else
        return Vertex((Handle)NULL);
}

Rule::~Rule()
{
/*  for (map<atom, setOfMPs, lessatom_ignoreVarNameDifferences>::iterator   i = o2iMetaCache.begin();
                                                i!= o2iMetaCache.end();
                                                i++)
        delete i->second;*/
}

Rule::setOfMPs makeSingletonSet(Rule::MPs& mp)
{
    Rule::setOfMPs ret;
    ret.insert(mp);
    return ret;
}

    meta NotEvaluatorRule::i2oType(const vector<Vertex>& h) const
    {
        assert(1==h.size());

        return meta(new tree<Vertex>(mva((Handle)NOT_LINK,
                tree<Vertex>(h[0])
        )));
    }
    
    Rule::setOfMPs NotEvaluatorRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        AtomSpace *nm = CogServer::getAtomSpace();
        if (!inheritsType(nm->getType(v2h(*outh->begin())), NOT_LINK))
            return Rule::setOfMPs();

LOG(-10, "SHOULD NOT BE HERE!");
getc(stdin);getc(stdin);getc(stdin);
        MPs ret;

        assert(outh->begin().number_of_children() == 1);
    
		ret.push_back(BBvtree(new BoundVTree(outh->begin(outh->begin())))); //1st child

//      printAtomTree(*(*ret)[0],0,4);

        overrideInputFilter = true;

        return makeSingletonSet(ret);
    }
    NotEvaluatorRule::NotEvaluatorRule(reasoning::iAtomTableWrapper *_destTable)
    : GenericRule<reasoning::NotFormula>(_destTable, true, "NotEvaluatorRule")
    {
        inputFilter.push_back(meta(
            new tree<Vertex>(mva((Handle)NOT_LINK,
                mva((Handle)ATOM)))
        ));
    }

    bool UnprovableType(Type T)
{
    return  //inheritsType(T, OR_LINK) ||
            inheritsType(T, CONCEPT_NODE) ||
            inheritsType(T, FORALL_LINK);
}

template<Type T>
Handle Join(Handle* h, int N, AtomTableWrapper& atw)
{
    vector<Handle> hs;
    for (int i = 0; i < N; i++)
        hs.push_back(h[i]);

    return atw.addLink(T, hs, TruthValue::TRUE_TV(), false);
}

template<Type T, typename ATW>
Handle Join(Handle h1, Handle h2, ATW& atw)
{
    Handle h[] = { h1, h2 };
    return Join<T>(h, 2, atw);
}


/**
SubsetEvalRule::SubsetEvalRule(iAtomTableWrapper *_destTable)
{
    inputFilter.push_back(boost::shared_ptr<atom>(new atom(__INSTANCEOF_N, 1, new atom(CONCEPT_NODE,0))));
}

Handle SubsetEvalRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
    {
        assert(n==2);
        
        assert(inheritsType(nm->getType(premiseArray[0]), CONCEPT_NODE));
        assert(inheritsType(nm->getType(premiseArray[1]), CONCEPT_NODE));

        vector<Handle> set1;
        vector<Handle> set2;
        set<Handle> used;

        set1 = constitutedSet(premiseArray[0],0.0f, 1);
        set2 = constitutedSet(premiseArray[1],0.0f, 1);

        TruthValue** tvs1 = new SimpleTruthValue*[set1.size()+set2.size()];
        TruthValue** tvs2 = new SimpleTruthValue*[set1.size()+set2.size()];

        int i=0, tot=0;

        // We pad the missing TVs with zeros:

        for (i = 0; i < set1.size(); i++)
        {
            tvs1[i] = getTruthValue(set1[i]);
            vector<Handle>::iterator h2 =
                find<vector<Handle>::iterator, Handle>(set2.begin(), set2.end(), set1[i]);
            if (h2 == set2.end())
                tvs2[i] = new SimpleTruthValue(0,0);
            else
            {
                tvs2[i] = getTruthValue(*h2)->clone();
                used.insert(*h2);
            }
        }
        tot = i;

        for (i = 0; i < set2.size(); i++)
            if (!STLhas(used, set2[i]))
            {
                tvs2[tot] = getTruthValue(set2[i])->clone();
                tvs1[tot] = new SimpleTruthValue(0,0);
                tot++;
            }
        
        TruthValue* retTV = f.compute(tvs1, tot, tvs2, tot);
        
        for (i = 0; i < set1.size(); i++)
            delete tvs1[i];

        for (i = 0; i < set2.size(); i++)
            delete tvs2[i];

        delete tvs1;
        delete tvs2;

        vector<Handle> hs;
        hs.push_back(premiseArray[0]);
        hs.push_back(premiseArray[1]);

        Handle ret = destTable->addLink(SUBSET_LINK,
                hs,
                retTV,
                                true);  
//                              false);
                
        return ret;
    }
*/
bool UnprovableType(Type T);

BoundVertex BaseCrispUnificationRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
    const int n = premiseArray.size();
    AtomSpace *nm = CogServer::getAtomSpace();
    
	cprintf(4, "BaseCrispUnificationRule::compute:");
#if 0    
    for (int i=0;i<n;i++)
        printTree(v2h(premiseArray[i]), 0, 4);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    for (int i=0;i<n;i++)
        printer.print(v2h(premiseArray[i]), 4);
#endif
    
    assert(nm->getType(v2h(premiseArray[0])) == FORALL_LINK);
    assert(nm->getType(v2h(premiseArray[n-1])) == HYPOTHETICAL_LINK);

    Handle topologicalStub = nm->getOutgoing(v2h(premiseArray[n-1]))[0];

    const TruthValue& tv = getTruthValue(v2h(premiseArray[0]));
    
    
    Handle ret= destTable->addLink( nm->getType(topologicalStub),
                                nm->getOutgoing(topologicalStub),
                                tv,
                                RuleResultFreshness);   

    return Vertex(ret);
}

Btr<set<BoundVertex > > CustomCrispUnificationRule::attemptDirectProduction(meta outh)
{
    AtomSpace *nm = CogServer::getAtomSpace();
    if (inheritsType(nm->getType(v2h(*outh->begin())), FORALL_LINK) ||
        inheritsType(nm->getType(v2h(*outh->begin())), FW_VARIABLE_NODE))
        return Btr<set<BoundVertex > >();

#if 0
    rawPrint(*outh, outh->begin(),0);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(vtree(outh->begin()));
#endif
cprintf(3,"FindMatchingUniversals...\n");
    Btr<ModifiedBoundVTree> i = FindMatchingUniversal(outh, ForallLink, destTable);
cprintf(3,"FindMatchingUniversals OK!\n");
    if (!i)
        return Btr<set<BoundVertex > >();

    Btr<set<BoundVertex > > ret(new set<BoundVertex >);
    
    MPs ret1;
    typedef pair<Handle,vtree> phvt;
    DeclareBtr(bindingsT, pre_binds);

    foreach(phvt vp, *i->bindings)
    {
        (*pre_binds)[vp.first] = make_real(vp.second);
#if 0    
        printTree(vp.first,0,0);
        cprintf(0,"=");
        printTree((*pre_binds)[vp.first],0,0);
#else 
        printer.print(vp.first);
        cprintf(0,"=");
        printer.print((*pre_binds)[vp.first]);
#endif
    }

    BBvtree rootAtom(new BoundVTree(*i, pre_binds));
    bind_Bvtree(rootAtom, *i->bindings);
    Handle topologicalStub = destTable->addAtom(*rootAtom, TruthValue::TRIVIAL_TV(), false, true);

    Handle ret_h = destTable->addLink(  nm->getType(topologicalStub),
                                nm->getOutgoing(topologicalStub),
                                getTruthValue(i->original_handle),
                                UnificationRuleResultFreshness);    
    
    ret->insert(BoundVertex(ret_h, pre_binds));

/*  haxx::bitnoderoot->inferred_with[ret_h] = (Rule*)(int)this;
    if (haxx::bitnoderoot->inferred_from[ret_h].empty()) //comes here often, we want the link only once
        haxx::bitnoderoot->inferred_from[ret_h].push_back(ForallLink);
*/
    haxx::inferred_with[ret_h] = (Rule*)this;
    if (haxx::inferred_from[ret_h].empty()) //comes here often, we want the link only once
        haxx::inferred_from[ret_h].push_back(ForallLink);

    return ret;
}

Rule::setOfMPs CustomCrispUnificationRuleComposer::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    AtomSpace *nm = CogServer::getAtomSpace();
    if (inheritsType(nm->getType(v2h(*outh->begin())), FORALL_LINK) ||
        inheritsType(nm->getType(v2h(*outh->begin())), FW_VARIABLE_NODE))
        return Rule::setOfMPs();

#if 0
    rawPrint(*outh, outh->begin(),0);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(vtree(outh->begin()));
#endif

    Btr<ModifiedBoundVTree> i = FindMatchingUniversal(outh, ForallLink, destTable);

    if (!i)
        return Rule::setOfMPs();

    setOfMPs ret;
    
    MPs ret1;
    typedef pair<Handle,vtree> phvt;
    DeclareBtr(bindingsT, pre_binds);

    foreach(phvt vp, *i->bindings)
        (*pre_binds)[vp.first] = make_real(vp.second);

    ret1.push_back(BBvtree(new BoundVTree(mva(i->original_handle), pre_binds)));

    BBvtree rootAtom(new BoundVTree(mva((Handle)HYPOTHETICAL_LINK, *i), pre_binds));

    ret1.push_back(rootAtom);

    for_each(ret1.begin(),
                ret1.end(),
                boost::bind(&bind_Bvtree, _1, *i->bindings));

    ret.insert(ret1);

    overrideInputFilter = true;
    
cprintf(3,"Crispu.o2i: OK! Solution vector size=%u\n", (uint) ret.size());
    
    return ret;
}

/***

StrictCrispUnificationRule has essentially been obsoleted by CustomCrispUnificationRule,
but we keep it here because its fundamentally different approach.
StrictCrispUnificationRule is a single stand-alone Rule, whereas Custom versions work so that
for each quantified atom struct, there'll be a new associated Custom Rule.

For StrictCrispUnificationRule:

Problem: StrictCrispUnificationRule has to bind FW_VARs, but it uses a design hack
which gathers hints from Atom Table instead of being pure o2i. Hence, it will
have to make var bindings, but it cannot return the information about them in
any way!

Solutions:
a) Do not bind FW_VARs. o2i returns the ForAllLink and the internals as-is,
containing the FW_VARs as they were passed to o2i from above.
BUT: FW_VARs were bound in a specific way in order to make StrictCrispUnificationRule
work. If they are left unbound, then they may be bound elsewhere in a way
which prevents StrictCrispUnificationRule from working, without our being able
to know this.

b) DO bind FW_VARs. 
BUT: Since we use direct production, bindings cannot be returned.
We can apply the bindings immediately to the ForAllLink's internals,
but then we have made local bindings without being able to pass knowledge of
that to other processes. Hence, if $1 (FW_VAR) was bound here in a certain way,
it can be bound on another branch different way, and if these branches are
combined, no conflict will be found!

*/

Rule::setOfMPs StrictCrispUnificationRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    AtomSpace *nm = CogServer::getAtomSpace();
    if (inheritsType(nm->getType(v2h(*outh->begin())), FORALL_LINK) ||
        inheritsType(nm->getType(v2h(*outh->begin())), FW_VARIABLE_NODE))
        return Rule::setOfMPs();

#if 0
    rawPrint(*outh, outh->begin(),0);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(outh->begin());
#endif

    Btr< set<Btr<ModifiedBoundVTree> > > varforms = FindMatchingUniversals(outh, destTable);
    
///return Rule::setOfMPs();

    if (varforms->empty())
        return Rule::setOfMPs();

    setOfMPs ret;
    
    foreach(Btr<ModifiedBoundVTree> i, *varforms)
    {
        MPs ret1;

        typedef pair<Handle,vtree> phvt;
        DeclareBtr(bindingsT, pre_binds);

        foreach(phvt vp, *i->bindings)
        {
            (*pre_binds)[vp.first] = make_real(vp.second);
        }

        ret1.push_back(BBvtree(new BoundVTree(mva(i->original_handle), pre_binds)));

cprintf(3,"And formm:\n");
#if 0
rawPrint(*ret1[0], ret1[0]->begin(),3);
#else 
printer.print(ret1[0]->begin(),3);
#endif

        BBvtree rootAtom(new BoundVTree(mva((Handle)HYPOTHETICAL_LINK, *i), pre_binds));

cprintf(3,"Root:\n");
#if 0
rawPrint(*rootAtom, rootAtom->begin(),3);
#else 
printer.print(rootAtom->begin(),3);
#endif

        ///Record targets to prevent multiple occurrence of the same target in the arg set.

        //set<vtree, less_vtree> arg_targets;
        set<vtree, less_vtree> arg_targets;

        /// rootAtom->hs[0] is the 1st link under the HYPOTHETICAL_LINK
        /// rootAtom->hs[0].hs[0] is the 1st link that this Rule does NOT prove

        for (vtree::sibling_iterator sit = i->begin(i->begin()); sit != i->end(i->begin()); sit++)
        {
            vtree tRes(sit);

            for (vtree::post_order_iterator pit = tRes.begin_post(); pit!=tRes.end_post(); pit++)
			{
                vtree pit_vtree(pit);
                if ( (!nm->isReal(v2h(*pit))
                    || inheritsType(nm->getType(v2h(*pit)), FW_VARIABLE_NODE))
                    && v2h(*pit) != (Handle)LIST_LINK
					&& !STLhas(arg_targets, pit_vtree) )
                {
					ret1.push_back(BBvtree(new BoundVTree(pit_vtree, pre_binds)));

					arg_targets.insert(pit_vtree);
                }
            }
        }

        ret1.push_back(rootAtom);

        for_each(   ret1.begin(),
                ret1.end(),
                boost::bind(&bind_Bvtree, _1, *i->bindings));

/*MPs::iterator m = ret1.begin();
for(; m != ret1.end(); m++)
    if (*m && (*m)->begin() != (*m)->end())
        printer.print((*m)->begin(),3);*/
        
        ret.insert(ret1);
    }

///     atom* ANDform = new atom(AND_LINK, 2,
/// new atom(__INSTANCEOF_N, 1, new atom(FORALL_LINK,0)),
/// new atom(__INDEX2, 1, ORform)

    overrideInputFilter = true;
    
cprintf(3,"Crispu.o2i: OK! Solution vector size=%u\n", (uint) ret.size());
    
    return ret;
}

/** ToDo: Update
bool ArityFreeANDRule::asymmetric(Handle* A, Handle* B) const
    {
        bool provenAsymmetric = false;
        bool provenSymmetric = false;
        bool swapped = false;
        TruthValue* tvs[2];
        vector<Handle> ImpLinks, EvaLinks;
    
        do
        {
            EvaLinks = nm->getOutgoing(*A); //Eval: P,a
            ImpLinks = nm->getOutgoing(*B); //Impl: P->P2

            tvs[0] = getTruthValue(*A);
            tvs[1] = getTruthValue(*B);
        
            if (nm->getType(*B) == IMPLICATION_LINK
                && nm->getType(*A) == EVALUATION_LINK
                && ImpLinks.size()==2   &&   EvaLinks.size()==2   &&   ImpLinks[0] == EvaLinks[0])
                provenAsymmetric = true;
            else if (!swapped)
            {
                swap<Handle>(*A, *B);
                swapped = true;
            }
            else
                provenSymmetric = true;
        } while (!provenSymmetric && !provenAsymmetric);

        return provenAsymmetric;
    }

    BoundVertex ArityFreeANDRule::compute(Handle A, Handle B, Handle CX)  const //vector<Handle> vh)
    {
        HandleSeq SAB;
        vector<Handle> ImpLinks, EvaLinks;

        TruthValue* t=NULL;
        
        TruthValue* tvs[2];

        bool is_asymmetric = asymmetric(&A, &B); //, ImpLinks, EvaLinks);

        EvaLinks = nm->getOutgoing(A); //Eval: P,a
        ImpLinks = nm->getOutgoing(B); //Impl: P->P2
        tvs[0] = getTruthValue(A);
        tvs[1] = getTruthValue(B);
        
        if (is_asymmetric) //if (isLink(nm->getType(B)))
        {
            t = f2.compute(tvs, 2);
            SAB.push_back(ImpLinks[1]);
            SAB.push_back(EvaLinks[1]);
            
            return destTable->addLink(  EVALUATION_LINK, SAB,
                t,
                                RuleResultFreshness);   

        }
        else if (nm->getType(B) == EVALUATION_LINK
            && nm->getType(A) == EVALUATION_LINK)
        {
            t = fN.compute(tvs, 2);
            SAB.push_back(A);
            SAB.push_back(B);
            
            return destTable->addLink(  AND_LINK, SAB,
                t,
                                RuleResultFreshness);   

        }
        else
        { assert(0); }
        
        return Vertex((Handle)NULL);
    }

    BoundVertex ArityFreeANDRule::computeSymmetric(vector<Handle> nodes, Handle CX)
    {
        const int n = nodes.size();
        TruthValue* t=NULL;
        TruthValue** tvs = new SimpleTruthValue*[nodes.size()];
        int ti=0;
        
        for (ti = 0; ti < nodes.size(); ti++)
            tvs[ti] = getTruthValue(nodes[ti]);

        t = fN.compute(tvs, nodes.size());
        
        Handle h = destTable->addLink(  AND_LINK, nodes,
            t,
                                RuleResultFreshness);   
        
        for (ti = 0; ti < nodes.size(); ti++)
            delete tvs[ti];
        
        //  delete[] tvs;
        
        return h;
    }
*/
void ArityFreeANDRule::DistinguishNodes(const vector<Vertex>& premiseArray, set<Handle>& ANDlinks, set<Handle>& nodes) const
  {
      const int n = premiseArray.size();
      
    for (int pi = 0; pi < n; pi++)
        ((((Type)(int)v2h(premiseArray[pi])) == AND_LINK)
            ? ANDlinks
            : nodes
            ).insert(v2h(premiseArray[pi]));
  }

void insertAllANDCombinations(set<atom, lessatom_ignoreVarNameDifferences> head, vector<atom> tail, set<atom, lessatom_ignoreVarNameDifferences>& AND_combinations)
{
    int totalSize = tail.size() + head.size();

    if (totalSize < 2)
        return;

    if (tail.empty())
    {
        atom neBoundVertex(AND_LINK, 0);
        neBoundVertex.hs = vector<Btr<atom> >(head.size());

        if (!head.empty())
            foreach(const atom &a, head)
                neBoundVertex.hs.push_back(Btr<atom>(new atom(a)));
//          copy(head.begin(), head.end(), neBoundVertex.hs.begin());

        AND_combinations.insert(neBoundVertex);
    }
    else
    {
        atom natom = tail.back();
        tail.pop_back();
        
        /// Combinations without natom

        if (totalSize >= 3) //If ANDLink producible even if 1 element was removed.
            insertAllANDCombinations(head, tail, AND_combinations);
        
        head.insert(natom);

        /// Combinations with natom

        insertAllANDCombinations(head, tail, AND_combinations);
    }
}

ImplicationTailExpansionRule::ImplicationTailExpansionRule(reasoning::iAtomTableWrapper *_destTable)
: Rule(_destTable, false, true, "ImplicationTailExpansionRule")
{
    inputFilter.push_back(meta(
        new tree<Vertex>(mva((Handle)
            IMPLICATION_LINK,
                mva((Handle)ATOM),
                mva((Handle)ATOM)
        ))
    ));
    inputFilter.push_back(meta(
        new tree<Vertex>(mva((Handle)
            IMPLICATION_LINK,
                mva((Handle)ATOM),
                mva((Handle)ATOM)
        ))
    ));
}

Rule::setOfMPs ImplicationTailExpansionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    AtomSpace *nm = CogServer::getAtomSpace();
    if (!inheritsType(nm->getType(v2h(*outh->begin())), IMPLICATION_LINK))
        return Rule::setOfMPs();
        
    tree<Vertex>::sibling_iterator hs1 = outh->begin(outh->begin());
    tree<Vertex>::sibling_iterator hs0 = hs1++;
    tree<Vertex>::sibling_iterator hs11 = outh->begin(hs1);
    tree<Vertex>::sibling_iterator hs10 =hs11++;

cprintf(0,"T:%d\n", (Type)(int)v2h(*hs1));
    
    if (hs0  == outh->end(outh->begin()) ||
        hs11 == outh->end(hs1) ||
        !inheritsType(nm->getType(v2h(*hs1)), AND_LINK) )
        return Rule::setOfMPs();

    /// A => (B&C)
    
    MPs ret;

    ret.push_back(BBvtree(new BoundVTree(mva((Handle)IMPLICATION_LINK, 
            mva(*hs0),
            vtree(hs10)))));
    ret.push_back(BBvtree(new BoundVTree(mva((Handle)IMPLICATION_LINK, 
            mva(*hs0),
            vtree(hs11)))));
//      ret.push_back(BBvtree(new BoundVTree(myvar)));

    overrideInputFilter = true;
    
    return makeSingletonSet(ret);
}

BoundVertex ImplicationTailExpansionRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
    AtomSpace *nm = CogServer::getAtomSpace();

/*  for (int i=0;i<premiseArray.size();i++)
        printTree(v2h(premiseArray[i]),0,0);
    getc(stdin);getc(stdin);*/
    
    vtree res(mva((Handle)IMPLICATION_LINK,
        mva(nm->getOutgoing(v2h(premiseArray[0]))[0]),
        mva((Handle)AND_LINK,
            mva(nm->getOutgoing(v2h(premiseArray[0]))[1]),
            mva(nm->getOutgoing(v2h(premiseArray[1]))[1])
        )
    ));
        
    /*for (int i=0;i<premiseArray.size();i++)
        res.append_child(res.begin(), premiseArray[i]*/
    
    return Vertex(destTable->addAtom(res, getTruthValue(v2h(SimpleANDRule<2>(destTable).compute(premiseArray,CX).value)),true,true));
}

/*Rule::setOfMPs ANDSubstRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    if (!inheritsType(nm->getType(v2h(*outh->begin())), IMPLICATION_LINK)
        || outh->number_of_children() != 2)
        return Rule::setOfMPs();        

    tree<Vertex>::sibling_iterator hs1 = outh->begin(outh->begin());
    tree<Vertex>::sibling_iterator hs0 = hs1++;
    
    if (!inheritsType(nm->getType(v2h(*hs0)), AND_LINK)
        || (   !inheritsType(nm->getType(v2h(*hs0)), AND_LINK)
            && !inheritsType(nm->getType(v2h(*hs0)), FW_VARIABLE_NODE)
    )
        return Rule::setOfMPs();
    
}
BoundVertex ANDSubstRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
}
Rule::setOfMPs ImplicationRedundantExpansionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
}
BoundVertex ImplicationRedundantExpansionRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
}
*/

CrispTheoremRule::CrispTheoremRule(reasoning::iAtomTableWrapper *_destTable)
: Rule(_destTable, true, true, "RewritingRule")
{
    /// SHOULD NOT BE USED FOR FORWARD INFERENCE!
    
    inputFilter.push_back(meta(
        new tree<Vertex>(mva(
            (Handle)ATOM))));
}

map<vtree, vector<vtree> ,less_vtree> CrispTheoremRule::thms;

BBvtree bind_vtree(vtree &targ, map<Handle, vtree>& binds)
{
                BBvtree thm_substed(new BoundVTree(targ));
                
                bool changes;
                do
                {               
                    changes = false;
                    
					cprintf(4,"Next change...\n");
                    
                    for(vtree::pre_order_iterator vit = thm_substed->begin(); vit != thm_substed->end(); vit++)
                    {
						cprintf(4,"Next change check...\n");
                        
                        Handle *ph = v2h(&*vit);
                        if (ph)
                        {
							cprintf(4,"(ph)");
                            
                            map<Handle, vtree>::iterator it = binds.find(*ph);
                            if (it != binds.end()) {
                            
                                cprintf(4,"replacing...[%lu]\n", v2h(*vit));
                                cprintf(4,"with...\n");
                                NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
                                printer.print(it->second.begin(),4);
                                
                                thm_substed->replace(vit, it->second.begin());

								cprintf(4,"replace ok\n");

                                changes = true;
                                goto break_inner; //vit = thm_substed->end(); //break inner loop                            
                            }
                        }
                        else
							cprintf(4,"NOT (ph)");
                    }
break_inner:                    
					cprintf(4,"1 change run ok");
                } while (changes);
                
    return thm_substed;             
}

Rule::setOfMPs CrispTheoremRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    AtomSpace *nm = CogServer::getAtomSpace();
    set<MPs> ret;   
bool htemp=false;
    
/*  for (map<vtree, vector<vtree> ,less_vtree>::iterator thm=thms.begin(); thm != thms.end(); thm++)
    {
        rawPrint(thm->first
    }*/

    for (map<vtree, vector<vtree> ,less_vtree>::iterator thm=thms.begin(); thm != thms.end(); thm++)
    {
        map<Handle, vtree> binds;
        set<Handle> vars;
        
        /// Grab the FW_VARs
        
        long count1=0, count2=0;
        
        //vector<vtree> thms2(thms->second);
        //vtree thms1(thms->first);
        
        foreach(vtree& targ, thm->second)
        {
			cprintf(1, "Producing thm: checking...\n");
            
            vtree tmp(thm->first);

			NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
            printer.print(tmp.begin(),1);

            //foreach(Vertex v, targ)
            for(vtree::iterator v  = targ.begin(); v != targ.end(); v++)
                if (nm->getType(v2h(*v)) == FW_VARIABLE_NODE)
                {
                    vars.insert(v2h(*v));
                    count1++;
                }
        }
        //foreach(Vertex v, thm->first)
        for(vtree::iterator v  = thm->first.begin(); v != thm->first.end(); v++)
                if (nm->getType(v2h(*v)) == FW_VARIABLE_NODE)
                {
                    vars.insert(v2h(*v));   
                    count1++;
                }
                        
        /// Rename the vars
        
        bindingsT newPreBinds;
        
        foreach(Handle h, vars)
            newPreBinds[h] = v2h(CreateVar(destTable));
                
         typedef pair<Handle,Handle> hpair;
                
		 cprintf(4,"Bindings are:\n");
                
         foreach(hpair hp, newPreBinds)
			cprintf(4, "%s => %s\n", nm->getName(hp.first).c_str(), nm->getName(hp.second).c_str());
		 cprintf(4,"<<< Bindings\n");
                
        /// Replace the old vars with renamed counterparts
                
        vtree           thm_target(thm->first);
        vector<vtree>   thm_args(thm->second);
                
        foreach(vtree& targ, thm_args)
            //foreach(Vertex& v, targ)
            for(vtree::iterator v  = targ.begin(); v != targ.end(); v++)
            {
                bindingsT::iterator bit = newPreBinds.find(v2h(*v));
                if (bit != newPreBinds.end())
                {
                    *v = Vertex(bit->second);
                    count2++;
                }
            }

        //foreach(Vertex& v, thm_target)
            for(vtree::iterator v  = thm_target.begin(); v != thm_target.end(); v++)
            {
                bindingsT::iterator bit = newPreBinds.find(v2h(*v));
                if (bit != newPreBinds.end())
                {
                    *v = Vertex(bit->second);
                    count2++;
                }
            }
		 cprintf(4,"From,to\n");
            
         vtree tmp2(thm->first);

		 NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
         printer.print(tmp2.begin(),4);
         tmp2 = thm_target;
         printer.print(tmp2.begin(),4);
        
         cprintf(4,"counts: %ld %ld\n", count1, count2);
        
         cprintf(4,"Unifies to %lu:\n",v2h(*outh->begin()));

         printer.print(outh->begin(), 4);

/// haxx:: warning!
/// should have different binds for left and right!!!

        if (unifiesTo(thm_target, *outh, binds, binds, true))
        {
             cprintf(4,"Unifies!\n");
             vtree tmp(thm_target);

             printer.print(tmp.begin(), 4);
            
             for (map<Handle, vtree>::iterator ii=binds.begin(); ii!=binds.end(); ii++)
             {
                cprintf(4,"%lu=>\n", ii->first);
                printer.print(ii->second.begin(), 4);
             }

 
            MPs new_thm_args;

            foreach(vtree targ, thm_args)
            {
                 cprintf(4,"Subst next...\n");
                bool changes = false;
                
                BBvtree thm_substed = bind_vtree(targ, binds);

				 cprintf(4,"FOR:\n");

                 printer.print(outh->begin(), 4);
                 cprintf(4,"thm_substed\n");
                 printer.print(thm_substed->begin(), 4);
                
                 htemp =true;
                
                new_thm_args.push_back(thm_substed);
            }

            new_thm_args.push_back(BBvtree(new BoundVTree(mva((Handle)HYPOTHETICAL_LINK,
                *outh))));

            ret.insert(new_thm_args);
			cprintf(1, "Producing thm found.\n");

            // Now, fix the inputFilter to correspond to the new nr of input args.
            const_cast<MPsIn*>(&inputFilter)->clear();
            for (uint i = 0; i < new_thm_args.size(); i++)
                const_cast<MPsIn*>(&inputFilter)->push_back(meta(
                    new tree<Vertex>(mva(
                    (Handle)ATOM))));
        }

    }


    overrideInputFilter = true;
    
    return ret;
}


BoundVertex CrispTheoremRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
    AtomSpace *nm = CogServer::getAtomSpace();

    /*vtree res(mva((Handle)IMPLICATION_LINK,
        mva(nm->getOutgoing(v2h(premiseArray[0]))[0]),
        mva((Handle)AND_LINK,
            mva(nm->getOutgoing(v2h(premiseArray[0]))[1]),
            mva(nm->getOutgoing(v2h(premiseArray[1]))[1])
        )
    ));*/

    int real_args = premiseArray.size()-1;
    
    //vtree res(mva(nm->getOutgoing(v2h(premiseArray[1]))[1]));
    
    cprintf(3,"CrispTheoremRule::compute... [%lu]\n", nm->getOutgoing(v2h(premiseArray[real_args]))[0]);
    
/*  printTree(nm->getOutgoing(v2h(premiseArray[0]))[0],0,0);
    printTree(nm->getOutgoing(v2h(premiseArray[1]))[0],0,0);
    printTree(nm->getOutgoing(v2h(premiseArray[2]))[0],0,0);
    
    cprintf(0,"<<<\n");*/
    
    /// Unravel the HYPOTHETICAL_LINK:
    
    vtree res(make_vtree(nm->getOutgoing(v2h(premiseArray[real_args]))[0]));
    
//  cprintf(0,"CrispTheoremRule::compute... make_vtree ok\n");
        
    /*for (int i=0;i<premiseArray.size();i++)
        res.append_child(res.begin(), premiseArray[i]*/
    
    TruthValue** tvs = new TruthValue*[premiseArray.size()];
    int i=0;
    foreach(const Vertex& v, premiseArray)
    {
        tvs[i++] = (TruthValue*) &(nm->getTV(v2h(v)));
    }
    
    bool use_AND_rule = (real_args>1);
    
//  cprintf(0,"CrispTheoremRule::compute... TV ok\n");
    
    TruthValue* tv = (use_AND_rule?SymmetricANDFormula().compute(tvs,real_args):tvs[0]->clone());
    Handle ret_h = destTable->addAtom(res, *tv, true,true);
    delete tv;
    
	cprintf(3,"CrispTheoremRule::compute produced:\n");

    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(ret_h);
    
    delete[] tvs;
    
    return Vertex(ret_h);
}

/*
  Rule::setOfMPs ImplicationConstructionRule::o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
    {
        if (!inheritsType(outh.T, IMPLICATION_LINK) || outh.hs.size() != 2)
            return Rule::setOfMPs();

        boost::shared_ptr<MPs> ret(new MPs);

        ret->push_back(boost::shared_ptr<atom>(new atom(outh)));
        (*ret)[0]->T = AND_LINK;

        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
    }
  atom ImplicationConstructionRule::i2oType(Handle* h, const int n) const
    {
        assert(1==n);

        return  atom(IMPLICATION_LINK, 2,
                        new atom(child(h[0], 0)),
                        new atom(child(h[0], 1))
                );
    }
*/

/* TODO: Update to BoundVTree
boost::shared_ptr<set<BoundVertex > > attemptDirectANDProduction(iAtomTableWrapper *destTable,
                                        const meta& outh,
                                        ArityFreeANDRule* rule)
    {
        if (!inheritsType(nm->getType(v2h(*outh->begin())), AND_LINK))
            return Rule::setOfMPs();

        tree<Vertex>::iterator top = outh->begin();
        const int N = outh->begin(top);

        set<atom, lessatom_ignoreVarNameDifferences> qnodes, AND_combinations;
        vector<atom> units;

        /// Add Individual nodes
    
        for (tree<Vertex>::sibling_iterator i = outh->begin(top);
                                            i!= outh->end  (top);
                                            i++)
            qnodes.insert(v2h(*i));

        AND_combinations = qnodes;

        /// Add ANDLinks

        insertAllANDCombinations(set<atom, lessatom_ignoreVarNameDifferences>(), outh.hs, AND_combinations);

        /// Create ring MP

        meta gatherMP(new BoundVTree((Handle)OR_LINK));

        if (!AND_combinations.empty())
        {
            //gatherMP.hs = vector<atom>(AND_combinations.size());
            for (set<atom, lessatom_ignoreVarNameDifferences>::iterator a =
                AND_combinations.begin(); a != AND_combinations.end(); a++)
            {
                gatherMP.append_child(  gatherMP.begin(),
                                        a->maketree().begin());
            }
            //copy(AND_combinations.begin(), AND_combinations.end(), gatherMP.hs.begin());
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

    }*/

/**
    atom UnorderedLinkPermutationRule::i2oType(Handle* h) const
    {
        assert(2==n);

        return  atom(child(h[1],0)); //Child of the topological link gives the topology
    }

    Rule::setOfMPs UnorderedLinkPermutationRule::o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
    {
        if (!outh.matchType(AND_LINK) || outh.hs.size() > MAX_ARITY_FOR_PERMUTATION)
            return Rule::setOfMPs();

printAtomTree(outh,0,1);

        set< vector<atom> > *ps = newCreatePermutations<atom>(outh.hs);

        boost::shared_ptr<MPs> ret(new MPs);

        boost::shared_ptr<MetaPredicate> newa( new atom(OR_LINK, 0) );

        for (set< vector<atom> >::iterator i = ps->begin(); i != ps->end(); i++)
            newa->hs.push_back(atom(outh.T, *i));

        ret->push_back(newa);

        printAtomTree(*(*ret)[0],0,4);

        /// Add the pseudo atom which determines the TOPOLOGY of the desired result

        boost::shared_ptr<atom> pseudoAtom(new atom(HYPOTHETICAL_LINK, 1, new atom(outh)));
        Handle h = pseudoAtom->attach(destTable);
        ret->push_back(pseudoAtom);

            TruthValue *tv = getTruthValue(h);


        //printAtomTree(*(*ret)[1],0,4);

        delete ps;

        overrideInputFilter = true;

        return makeSingletonSet(ret);
    }
*/

    Handle UnorderedCcompute(iAtomTableWrapper *destTable, Type linkT, const ArityFreeFormula<TruthValue,
                   TruthValue*>& fN, Handle* premiseArray, const int n, Handle CX)
    {
        TruthValue** tvs = new TruthValue*[n];
        for (int i = 0; i < n; i++)
            tvs[i] = (TruthValue*) &(getTruthValue(premiseArray[i]));
            //tvs[i] = (TruthValue*) destTable->getTruthValue(premiseArray[i]);
//puts("Computing formula");
        TruthValue* retTV = fN.compute(tvs, n);
//puts("Creating outset");
        HandleSeq outgoing;
        for (int j = 0; j < n; j++)
            outgoing.push_back(premiseArray[j]);
//puts("Adding link");
        Handle ret = destTable->addLink(linkT, outgoing,
            *retTV,
            RuleResultFreshness);   

        /// Release
        
        delete[] tvs;
        delete retTV;

        return ret;
    }
    
    Rule::setOfMPs PartitionRule_o2iMetaExtra(meta outh, bool& overrideInputFilter, Type OutLinkType)
    {
        AtomSpace *nm = CogServer::getAtomSpace();
        const int N = outh->begin().number_of_children();
        
        if (!inheritsType(nm->getType(v2h(*outh->begin())), OutLinkType) ||
            N <= MAX_ARITY_FOR_PERMUTATION)
            return Rule::setOfMPs();

        Rule::MPs ret;

        int parts = N / MAX_ARITY_FOR_PERMUTATION;
        int remainder = N % MAX_ARITY_FOR_PERMUTATION;

        vector<atom> hs;
        tree<Vertex>::sibling_iterator ptr = outh->begin(outh->begin());
        BBvtree root(new BoundVTree);
        root->set_head((Handle)OutLinkType);

        for (int i = 0; i <= parts; i++)
        {
            int elems = ( (i<parts) ? MAX_ARITY_FOR_PERMUTATION : remainder);
            
            BoundVTree elem(mva((Handle)AND_LINK));
            
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

    Rule::setOfMPs ANDPartitionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        return PartitionRule_o2iMetaExtra(outh, overrideInputFilter, AND_LINK);
    }


    Rule::setOfMPs ORPartitionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        return PartitionRule_o2iMetaExtra(outh, overrideInputFilter, OR_LINK);
        
/*      if (!inheritsType(nm->getType(v2h(*outh->begin())), OR_LINK) ||
            outh->begin().number_of_children() <= 2)
            return Rule::setOfMPs();

        
        
** TODO: Update to BoundVTree. I no longer remember how this was supposed to work!
        MPs ret;

        //vector<atom> hs;
//          hs.push_back(out_hs[0]);

        vector<atom>::iterator bigstart = out_hs.begin();
        vector<atom> hs2(++bigstart, out_hs.end());
//      hs.push_back(atom(OR_LINK, hs2));

        ret.push_back(BBvtree(new BoundVTree(atom(out_hs[0]).maketree()));
        ret.push_back(BBvtree(new BoundVTree(atom(OR_LINK, hs2).maketree()));

        overrideInputFilter = true;
        
        return makeSingletonSet(ret);*/
    }

    Rule::setOfMPs ORRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        AtomSpace *nm = CogServer::getAtomSpace();
        tree<Vertex>::iterator top = outh->begin();
        
        if (!inheritsType(nm->getType(v2h(*top)), OR_LINK) ||
            top.number_of_children() > 2)
            return Rule::setOfMPs();

        MPs ret;
        
        for (tree<Vertex>::sibling_iterator i = outh->begin(top); i != outh->end(top); i++)
        {
            //ret.push_back(BBvtree(new BoundVTree((ModifiedVTree *) *i)));
			ret.push_back(BBvtree(new BoundVTree(i)));
        }       
            
        overrideInputFilter = true;

        return makeSingletonSet(ret);
    }


Rule::setOfMPs ANDRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    AtomSpace *nm = CogServer::getAtomSpace();

    //return Rule::setOfMPs();
    
    ///// Temporarily disabled due to strange bugs!
    
    tree<Vertex>::iterator top = outh->begin();

        if (!inheritsType(nm->getType(v2h(*top)), AND_LINK) ||
            top.number_of_children() <= 2)  
            return Rule::setOfMPs();

        /// This Rule cannot produce nested ANDLinks. Try SimpleANDRule instead.

        for (tree<Vertex>::sibling_iterator j = outh->begin(top); j != outh->end(top); j++)
            if (inheritsType(nm->getType(v2h(*j)), AND_LINK))
                return Rule::setOfMPs();

        MPs ret;

        set<atom,lessatom> query_set;
        for (tree<Vertex>::sibling_iterator i = outh->begin(top); i != outh->end(top); i++)
        {
            tree<Vertex> t_tmp(i);
            query_set.insert(atom(t_tmp, t_tmp.begin()));
        }

        /// Smart lookup begins

        Btr<std::set<Handle> > sANDLink_set = destTable->getHandleSet(AND_LINK,"");
        std::vector<Handle> ANDLink_set(sANDLink_set->size());
        std::copy(sANDLink_set->begin(), sANDLink_set->end(), ANDLink_set.begin());

        while (1)
        {
            std::vector<Btr<atom> > max_subset;

            getLargestIntersection2(query_set, ANDLink_set, max_subset);

            if (max_subset.size() > 1)
            {
                for (uint s=0; s < max_subset.size(); s++)
                    query_set.erase(*max_subset[s]);

				ret.push_back(BBvtree(new BoundVTree(atom(AND_LINK, max_subset).makeHandletree(destTable))));

                continue;
            }
            else break;
        }

        /// Add the remaining ones.

		for (set<atom, lessatom>::iterator i = query_set.begin(); i != query_set.end(); i++)
			ret.push_back(BBvtree(new BoundVTree(i->makeHandletree(destTable))));

        overrideInputFilter = true;

        
        return makeSingletonSet(ret);
}

/*boost::shared_ptr<set<BoundVertex > > ANDRule::attemptDirectProduction(meta outh);
{
    return attemptDirectANDProduction(destTable, outh, this);
}*/

BoundVertex ANDRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
    AtomSpace *nm = CogServer::getAtomSpace();
    const int n = premiseArray.size();
  try
  {     
    for (int i=0; i < n; i++)
        if (!v2h(&premiseArray[i]))
            return Vertex((Handle)NULL);

    // The items in the the premiseArray are divided into ANDLinks and nodes.
    // Then, the nodes are combined into a single ANDlink by symmetric AND formula,
    // and added to the list of ANDlinks.
    
    int p=0;
    
    
LOG(3, "ANDRule::compute");

    set<Handle> premises, nodes;
    DistinguishNodes(premiseArray, premises, nodes);
LOG(4, "ANDRule::compute");
//  if (!nodes.empty())
//      premises.push_back( computeSymmetric(nodes) );

    for (set<Handle>::iterator j = nodes.begin(); j != nodes.end(); j++) //Nodes included also separately.
    {
        premises.insert(*j);
    }
LOG(4, "ANDRule::compute");
    TruthValue **partialTVs = new TruthValue*[premises.size()];

    set<Handle>::const_iterator i;
    set<Handle> conjunct;
    set<TruthValue*> TVowner;

    /// Create the set of elements in the result conjunction
LOG(4, "ANDRule::computeCC");
    for (i = premises.begin(); i != premises.end(); i++)
    {
        std::vector<Handle> inc2;

        if (isSubType(*i, AND_LINK))
            inc2 = nm->getOutgoing(*i);
        else
            inc2.push_back(*i);

        const std::vector<Handle>* inc = &inc2;
        
        for (std::vector<Handle>::const_iterator j = inc->begin(); j != inc->end(); j++)
            if (conjunct.find(*j) == conjunct.end())
                conjunct.insert(*j);
    }
LOG(4, "22 ANDRule::compute");
    /// Loop thru the premises, creating the partialTVs.

    for (p=0, i = premises.begin(); i != premises.end(); i++, p++)
    {
LOG(4, "Q ANDRule::compute");
    /// Put into Di all the elements of the result conjunct not present in the premise #i
        set<Handle> Di;
        for (set<Handle>::const_iterator j = conjunct.begin(); j != conjunct.end(); j++)
        {
            std::vector<Handle> inc2; // = nm->getOutgoing(*i);

            if (isSubType(*i, AND_LINK))
                inc2 = nm->getOutgoing(*i);
            else
                inc2.push_back(*i);

            std::vector<Handle>* inc = &inc2;
            if (!vectorHas<Handle>(*inc, *j))
                Di.insert(*j);
        }
LOG(4, "W ANDRule::compute");
        int Dis = Di.size();
    
        set<Handle> DiSubsets;
        Handle largest_intersection;

LOG(4,"ANDRule::compute:");

NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
for (set<Handle>::const_iterator di = Di.begin(); di != Di.end(); di++)
    printer.print(*di, 4);
        
LOG(4, "ANDRule:: getLargestIntersection");
        while (getLargestIntersection(Di, premises, largest_intersection))
        {
cprintf(4,"Y ANDRule::compute Di size = %u\n", (uint) Di.size());          
            const std::vector<Handle> new_elem2 = nm->getOutgoing(largest_intersection);

#ifdef WIN32            
            for (std::vector<Handle>::const_iterator k = new_elem2.begin(); k != new_elem2.end();)
            {   
                std::vector<Handle>::const_iterator next_k = k;
                next_k++;
                Di.erase(*k);
                k = next_k;
            }
#else
            std::set<Handle> new_elemset(new_elem2.begin(), new_elem2.end());
            std::set<Handle> old_Di = Di;
            Di.clear();
            
            for (std::set<Handle>::const_iterator d = old_Di.begin();
                d != old_Di.end(); d++)
                if (!STLhas(new_elemset, *d))
                    Di.insert(*d);          
            
/*          std::vector<Handle>::iterator Dii;
            
            *Dii = 0; //old_Di.begin();
            
            set_difference< std::set<Handle>::iterator,
                            std::vector<Handle>::const_iterator,
                            std::set<Handle>::iterator>
                (old_Di.begin(), old_Di.end(),
                 new_elem2.begin(), new_elem2.end(),
                 Dii);*/
#endif          
            DiSubsets.insert(largest_intersection); 
        }

LOG(4, "ANDRule:: getLargestIntersection OK!");
        TruthValue** tvs = new TruthValue*[1 + DiSubsets.size()];
        
        tvs[0] = (TruthValue*) &(getTruthValue(*i));
    
        int h=0;
        set<Handle>::const_iterator ss;
        for (h = 0, ss = DiSubsets.begin(); ss != DiSubsets.end(); ss++, h++)
            tvs[h+1] = (TruthValue*) &(getTruthValue(*ss));
LOG(4, "R ANDRule::compute");

/*      if (DiSubsets.size()>0)
        {
            printf("AND ");
            for (set<Handle>::const_iterator z=DiSubsets.begin();z != DiSubsets.end(); z++)
            {
                printf("(");
                for (int y=0; y < nm->getOutgoing(*z).size(); y++)
                    printf("%s ", nm->getName(nm->getOutgoing(*z)[y]).c_str());
                printf(")");
            }
            printf("\n");
        }       
*/
        // Calculate either the ANDNode evaluation or just use the premise strength:

        if (DiSubsets.size()>0)
        {
            partialTVs[p] = fN.compute(tvs, 1+DiSubsets.size());
            TVowner.insert(partialTVs[p]); // TVs to be deleted later
        }
        else
            partialTVs[p] = tvs[0];

//      printf("Part: %f\n", partialTVs[p]->getMean());
        
        delete[] tvs;
    }
    /// Combine the partialTVs
LOG(4, "33 ANDRule::compute");

    TruthValue* retTV = fN.compute(partialTVs, premises.size());

    HandleSeq outgoing;
    for (set<Handle>::const_iterator c = conjunct.begin(); c != conjunct.end(); c++)
        outgoing.push_back(*c);
LOG(4, "44 ANDRule::compute");
    Handle ret = destTable->addLink(AND_LINK, outgoing,
                    *retTV,
                    RuleResultFreshness);   
    
LOG(4, "55 ANDRule::compute");

    delete[] partialTVs;
    delete retTV;
    
    for (set<TruthValue*>::iterator t= TVowner.begin(); t != TVowner.end(); t++)
        delete *t;
    
LOG(3, "ANDRule::compute ok.");		
    
        return Vertex(ret);

  } catch(...) { LOG(-10, "Exception in ANDRule::compute!");
        #ifdef NMDEBUG
            getc(stdin);
        #endif
      return Vertex((Handle)NULL); }
}   

Rule::setOfMPs ScholemFunctionProductionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    return Rule::setOfMPs();
}

/** Won't produce EXISTING Scholem function mappings! Thus, run lookupRule 1st!
    Scholem links are not memory-managed.
*/

boost::shared_ptr<set<BoundVertex > > ScholemFunctionProductionRule::attemptDirectProduction(meta outh)
{
    AtomSpace *nm = CogServer::getAtomSpace();

    boost::shared_ptr<set<BoundVertex > > ret;
    
    if (!inheritsType(nm->getType(v2h(*outh->begin())), SCHOLEM_LINK))
        return ret;

    //assert(outh->begin().number_of_children() == 2);
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    if (outh->begin().number_of_children() != 2)
    {
        printer.print(outh->begin(), 2);
        LOG(2, "ScholemFunctionProductionRule::attemptDirectProduction(meta outh) with != 2 args.");
        printer.print(v2h(*outh->begin()), 2);
    }

    tree<Vertex> old_subst(*outh);
    
    tree<Vertex>::sibling_iterator child2 = old_subst.begin(old_subst.begin());
    tree<Vertex>::sibling_iterator child1 = child2++;

#ifdef _MSC_VER
#pragma warning(Remove this hack ASAP!)
#endif // _MSC_VER
//#warning "Remove this hack ASAP!"
haxx::AllowFW_VARIABLENODESinCore = true;
    
//  assert(!inheritsType(nm->getType(v2h(*child1)), VARIABLE_NODE));
//  assert(!inheritsType(nm->getType(v2h(*child2)), VARIABLE_NODE));
    if (inheritsType(nm->getType(v2h(*child1)), VARIABLE_NODE))
    {
        printer.print(outh->begin(), 2);
        LOG(2, "ScholemFunctionProductionRule::attemptDirectProduction(meta outh) child1 problem.");
        printer.print(v2h(*child1), 2);
    }
    
    if (inheritsType(nm->getType(v2h(*child2)), VARIABLE_NODE))
    {
        printer.print(outh->begin(), 2);
        LOG(2, "ScholemFunctionProductionRule::attemptDirectProduction(meta outh) child2 problem.");
        printer.print(v2h(*child2), 2);
    }

    
    *child2 = CreateVar(destTable);
    
    TableGather s(old_subst, destTable);

    if (s.empty())
    {
        ret = boost::shared_ptr<set<BoundVertex > >(new set<BoundVertex>);
        ret->insert(BoundVertex(Vertex(destTable->addAtom(*atomWithNewType(*outh, SCHOLEM_LINK),
                TruthValue::TRUE_TV(),
                false, false))));
        
        return ret;
    }

haxx::AllowFW_VARIABLENODESinCore = false;

    /// This Rule shouldn't be used to produce EXISTING Scholem function mappings!
    
	LOG(3, "Tried to re-map a scholem function argument.");
    
    return ret;

/*  map<string, map<Handle, Handle> >::iterator f = haxx::scholemFunctions.find(outh.name);
    if (f != scholemFunctions.end())
    {
        Handle arg1 = outh.hs[0].attach(destTable);

        map<Handle, Handle> old_subst = f->second.find(arg1);

        if (old_subst != haxx::scholemFunctions.end())
        {
            if (atom(old_subst->second) == outh)
                return 
        }
        else
        {
            f->second[arg1] = outh.hs[1].attach(destTable);
        }
    }
    else
        haxx::scholemFunctions*/
}

#define USE_INCLUSION_EXCLUSION_IN_OR_RULE 0

TruthValue** ORRule::formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
    {
        const int N = (int)premiseArray.size();
        
cprintf(3, "ORRule::formatTVarray...");
#if USE_INCLUSION_EXCLUSION_IN_OR_RULE
        *newN = N*(N+1)/2; // (N-1)+(N-2)+...+1 = N*(N-1)/2
#else
        *newN = N;
#endif
//printTree(premiseArray[0],0,3);
        TruthValue** tvs = new TruthValue*[*newN];

        int i = 0, ii=0;
        for (i = 0; i < N; i++)
        {
            tvs[ii++] = (TruthValue*) &(getTruthValue(v2h(premiseArray[i])));
cprintf(4,"TV Arg: %s -\n", tvs[i]->toString().c_str());
        }
        
        for (i = 0; i < N-1; i++)
            for (int j = i+1; j < N; j++)
            {
#if USE_INCLUSION_EXCLUSION_IN_OR_RULE

cprintf(4,"Look up ANDLINK for args #%d,%d\n", i,j);
                TableGather comb(mva((Handle)AND_LINK,
                                    mva(premiseArray[i]),
                                    mva(premiseArray[j])
                                ), destTable);
cprintf(4,"Look up %s\n", (comb.empty() ? "success." : "fails."));
                tvs[ii++] = 
                                (!comb.empty()
                                ? getTruthValue(v2h(comb[0]))
#if 0
// Welter's comment: this change is waiting for Ari's aproval 
                                : TruthValue::TRIVIAL_TV());
#else
                                : SimpleTruthValue(0.0,0.0)); // TODO: Use a static variable...
#endif
#else
#endif
            }
cprintf(4, "ORRule::formatTVarray OK.");
        return tvs;
    }

    meta ORRule::i2oType(const vector<Vertex>& h) const
    {
        meta ret(new tree<Vertex>(mva((Handle)OR_LINK)));

        for (uint i=0; i < h.size(); i++)
            ret->append_child(ret->begin(), mva(h[i]).begin());

        return  ret;
    }

ORRule::ORRule(iAtomTableWrapper *_destTable)
: GenericRule<ORFormula>(_destTable, true, "OR Rule")
{
}


/*setOfMPs ANDBreakdownRule::o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
{
    boost::shared_ptr<MPs> ret(new MPs);

    MPs->push_back(
}*/

/*
  [IntInh A B].tv.s = [ExtInh Ass(B) Ass(A)].tv.s
  Ass(A) = { c: [p(C|A) - P(C|-A)] > 0 }
  Wrong ExtInh direction?
  */

/** TODO: Update to tree<Vertex> && new rule storage (not ATW)


Handle Ass(iAtomTableWrapper *destTable, Handle h, vector<Handle>& ret)
{
    TableGather g(mva((Handle)INHERITANCE_LINK,
        mva(CreateVar(destTable)),
        mva(h)));
    TableGather reverseg(mva((Handle)INHERITANCE_LINK,
        mva(h),
        mva(CreateVar(destTable))));

    for(vector<Handle>::iterator i = reverseg.begin(); i != reverseg.end(); i++)
    {
        Handle hs[] = { *i };
        *i = AtomTableWrapper::rule[Inversion_Inheritance]->compute(hs,1);
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
        TruthValue* tv = getTruthValue(*k);

        printTree(
            destTable->addAtom(
                atom(MEMBER_LINK, 2,
                    new atom(nm->getOutgoing(*k)[0]),
                    new atom(ass)
                ),
                tv->clone(),
                                true);  
//                              false);
            )
            ,0,0
        );
    }

    return ass.attach(destTable);
}*/

unsigned long now_interval_len = 50000;

bool ExpandEvaluationLinks(vtree& target, iAtomTableWrapper* destTable)
{
    AtomSpace *nm = CogServer::getAtomSpace();
    bool is_changed = false;
    
    for(vtree::iterator i = target.begin(); i != target.end(); i++)
        if (nm->inheritsType(nm->getType(v2h(*i)), CONCEPT_NODE))
        {
            string name = nm->getName(v2h(*i));
            if (name == "!whileago" || name == "!now")
            {
#ifndef WIN32
            /*  timeval currentTime;
            gettimeofday(&currentTime, NULL);
            long now_stamp = currentTime.tv_sec*1000 + currentTime.tv_usec/1000;*/
			signed long now_stamp = (signed long)getElapsedMillis();
            signed long now_interval = now_interval_len;
#else
            signed long now_stamp = 3000;
            signed long now_interval = 1000;
#endif
            char end_stamp_s[100], begin_stamp_s[100];
            sprintf(begin_stamp_s, "%f", max(0,now_stamp-now_interval));
            sprintf(end_stamp_s, "%ld", now_stamp);

            if (name == "!whileago")
                target.replace(i,
                        Vertex(destTable->addNode(NUMBER_NODE, begin_stamp_s,
                        TruthValue::TRUE_TV(),false)));
//              target.replace(i, Vertex(NewNode(NUMBER_NODE, begin_stamp_s)));
            if (name == "!now")
                target.replace(i,
                        Vertex(destTable->addNode(NUMBER_NODE, end_stamp_s,
                        TruthValue::TRUE_TV(),false)));
//              target.replace(i, Vertex(NewNode(NUMBER_NODE, end_stamp_s)));

                is_changed = true;
            }
        }
        
    return is_changed;
}

boost::shared_ptr<set<BoundVertex> > LookupRule::attemptDirectProduction(meta outh)
{
    vtree target(*outh);

    boost::shared_ptr<set<BoundVertex> > ret(new TableGather(target, destTable));
    
    if (ExpandEvaluationLinks(target, destTable))
    {
        TableGather dynamic_lookup(target, destTable);
        copy(dynamic_lookup.begin(), dynamic_lookup.end(), inserter(*ret, ret->begin()));

//      puts("Lookup macros!");
//      NMPrinter()(NMPrintable(target), -3);
        boost::shared_ptr<set<BoundVertex> > ret(new TableGather(target, destTable));
        
        /*foreach(const BoundVertex& bv, *ret)
        {
            NMPrinter()(v2h(bv.value), -3);
        }
        if (!ret->empty())
        {
            puts("results above!");
            getc(stdin);
        }*/
    }   

    return ret;
}

Rule::setOfMPs LookupRule:: o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    assert(0);
    return Rule::setOfMPs();
}

Handle _v2h(const Vertex& v) { return v2h(v); }

boost::shared_ptr<set<BoundVertex> > HypothesisRule::attemptDirectProduction(meta outh)
{
    AtomSpace *nm = CogServer::getAtomSpace();
    set<BoundVertex>* ret = new set<BoundVertex>;
    
    Type t = nm->getTypeV(*outh);
    bool hyp_link = inheritsType(t, HYPOTHETICAL_LINK);

	if (HYPRULE_MAKES_ZERO_CONFIDENCE_ATOMS || hyp_link)
        if (!hasFW_VAR(*outh))
        {
 			 cprintf(4,"HYP0:\n");
             NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
             printer.print(outh->begin(), 4);

            ret->insert(BoundVertex(destTable->addAtom(*outh, TruthValue::TRIVIAL_TV(), false, true)));

			 cprintf(4,"HYP:\n");
             printer.print(v2h(ret->begin()->value), 4);
		}

    return boost::shared_ptr<set<BoundVertex> >(ret);
}

#if 1
Rule::setOfMPs HypothesisRule:: o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    assert(0);
    return Rule::setOfMPs();
}

    BoundVertex ANDPartitionRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
    {
        const int N = (int)premiseArray.size();
        Handle *hs = new Handle[N];
        
        transform(premiseArray.begin(), premiseArray.end(), &hs[0], GetHandle()); //mem_fun(
//          Concat<DropVertexBindings, GetHandle, BoundVertex, Handle>());

        BoundVertex ret = Vertex(UnorderedCcompute(destTable, AND_LINK, fN, hs,N,CX));
        delete[] hs;
        return ret;
    }

BoundVertex ORPartitionRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
/*  Handle *hs = new Handle[premiseArray.size()];
    transform(premiseArray.begin(), premiseArray.end(), hs[0], DropVertexBindings()); //mem_fun(
    const int n = premiseArray.size();*/

    BoundVertex ret = regularOR->compute(premiseArray,CX);
//  delete[] hs;
    return ret;
}

    ImplicationBreakdownRule::ImplicationBreakdownRule(iAtomTableWrapper *_destTable)
    : Rule(_destTable,false,true,"ImplicationBreakdown")
    {
        inputFilter.push_back(meta(
                new tree<Vertex>(
                mva((Handle)IMPLICATION_LINK,
                    mva((Handle)ATOM),
                    mva((Handle)ATOM)))
            ));
/*      inputFilter.push_back(meta(
                new tree<Vertex>(       
                    mva((Handle)ATOM))
            ));*/
    }
    Rule::setOfMPs ImplicationBreakdownRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        ///haxx:: (restricts internal implications

//      if (inheritsType(nm->getType(v2h(*outh->begin())), IMPLICATION_LINK))
//          return Rule::setOfMPs();

        MPs ret;
        Vertex myvar = CreateVar(destTable);

        // Joel: Wrapped up myvar in vtree to fit functions
        ret.push_back(BBvtree(new BoundVTree(mva((Handle)IMPLICATION_LINK, 
            vtree(myvar),
            *outh))));
//		ret.push_back(meta(new BoundVTree(myvar)));
        
        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
    }
    BoundVertex ImplicationBreakdownRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
    {
        AtomSpace *nm = CogServer::getAtomSpace();
        assert(validate(premiseArray));

//printTree(premiseArray[0],0,1);

        std::vector<Handle> args = nm->getOutgoing(v2h(premiseArray[0]));
        Type T = nm->getType(args[1]);
        std::string pname = nm->getName(args[1]);

        TruthValue* tvs[] = {
            (TruthValue*) &(getTruthValue(v2h(premiseArray[0]))),
            (TruthValue*) &(getTruthValue(args[0])),
            (TruthValue*) &(getTruthValue(args[1]))
        };
        
        TruthValue* retTV =
            ImplicationBreakdownFormula().compute(tvs, 3);

        std::vector<Handle> new_args = nm->getOutgoing(args[1]);

        Handle ret=NULL;

        /*if (inheritsType(T, NODE))
            ret = destTable->addNode(T, pname,
                    *retTV,
//                  true);          
                    false);
        else*/
    
        assert (!(inheritsType(T, NODE)));

        ret = destTable->addLink(T, new_args,
                    *retTV,
                    RuleResultFreshness);   

        delete retTV;

NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
printer.print(ret, 1);

        return Vertex(ret);
    }   

    StrictImplicationBreakdownRule::StrictImplicationBreakdownRule(iAtomTableWrapper *_destTable)
    : Rule(_destTable,false,true,"ModusPonensRule")
    {
        inputFilter.push_back(meta(
                new tree<Vertex>(
                mva((Handle)IMPLICATION_LINK,
                    mva((Handle)ATOM),
                    mva((Handle)ATOM)))
            ));
        inputFilter.push_back(meta(
                new tree<Vertex>(       
                    mva((Handle)ATOM))
            ));
    }

    Rule::setOfMPs StrictImplicationBreakdownRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        ///haxx:: (restricts internal implications

//      if (inheritsType(nm->getType(v2h(*outh->begin())), IMPLICATION_LINK))
//          return Rule::setOfMPs();

        MPs ret;
        Vertex myvar = CreateVar(destTable);

        cprintf(4,"\n\nTo produce\n");
        NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
        printer.print(outh->begin(), 4);

        ret.push_back(BBvtree(new BoundVTree(
                        mva((Handle)IMPLICATION_LINK, vtree(myvar),*outh))));
        cprintf(4,"Need:\n");
		ret.push_back(BBvtree(new BoundVTree(myvar)));
        printer.print(ret[0]->begin(), 4);
        printer.print(ret[1]->begin(), 4);

        cprintf(4,"-----\n");
        
        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
    }
    BoundVertex StrictImplicationBreakdownRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
    {
        AtomSpace *nm = CogServer::getAtomSpace();
        assert(validate(premiseArray));

//printTree(premiseArray[0],0,1);

		cprintf(3,"StrictImplicationBreakdownRule::compute:");

        NMPrinter printer(NMP_ALL);
        for (uint i=0;i<premiseArray.size();i++)
            printer.print(v2h(premiseArray[i]), 3);

//  vtree   vt1(make_vtree(nm->getOutgoing(v2h(premiseArray[0]),0))),
//          vt2(make_vtree(v2h(premiseArray[1])));

    vtree   vt1(make_vtree(v2h(premiseArray[0]))),
            vt2(make_vtree(v2h(premiseArray[1])));

        /** haxx:: \todo Temporarily disabled!
            This check does not hold if one of the args
            has been executed but the other one has not.
            Execution here means that Eval(!now) becomes Eval(35353).
            ! is a hacky shorthand for grounded predicates for now.
            
            The real solution will be tointroduce an equality check which considers
            the unexecuted and executed forms equal.
        */

#if 0
        
    if (make_vtree(nm->getOutgoing(v2h(premiseArray[0]),0))
        != make_vtree(v2h(premiseArray[1])))
    {
        cprintf(0,"StrictImplicationBreakdownRule args fail:\n");
#if 0
        printTree(v2h(premiseArray[0]),0,0);
        printTree(nm->getOutgoing(v2h(premiseArray[0]),0),0,0);
        printTree(v2h(premiseArray[1]),0,0);

//      rawPrint(premiseArray[0], premiseArray[0].begin(), 0);
        rawPrint(vt1, vt1.begin(), 0);
        rawPrint(vt2, vt2.begin(), 0);
#else 
        printer.print(v2h(premiseArray[0]), -10);
        printer.print(nm->getOutgoing(v2h(premiseArray[0]),0), -10);
        printer.print(v2h(premiseArray[1]), -10);

//        printer.print(premiseArray[0].begin());
        printer.print(vt1.begin(), -10);
        printer.print(vt2.begin(), -10);
#endif
        getc(stdin);getc(stdin);
        assert(0);
    }
#endif
    
        std::vector<Handle> args = nm->getOutgoing(v2h(premiseArray[0]));
        Type T = nm->getType(args[1]);
        std::string pname = nm->getName(args[1]);

        TruthValue* tvs[] = {
            (TruthValue*) &(getTruthValue(v2h(premiseArray[0]))),
            (TruthValue*) &(getTruthValue(v2h(premiseArray[1]))),
            (TruthValue*) &(getTruthValue(args[1]))
        };
        
        TruthValue* retTV =
            ImplicationBreakdownFormula().compute(tvs, 3);

        std::vector<Handle> new_args = nm->getOutgoing(args[1]);

        Handle ret=NULL;
    
        assert (!(inheritsType(T, NODE)));

        ret = destTable->addLink(T, new_args,
                    *retTV,
                    RuleResultFreshness);   

        delete retTV;                    

		printer.print(ret, 3);

        return Vertex(ret);
    }   

#if 0
Rule::setOfMPs SimSubstRule1::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
/** For simplicity (but sacrificing applicability),
FW_VARs cannot be replaced with children structs.
Links are assumed not inheritable either.
    */
    
    if (outh->begin().number_of_children() != 2
        ||  inheritsType(nm->getType(v2h(*outh->begin())), FW_VARIABLE_NODE))
        return Rule::setOfMPs();

/*  puts("X1");
    rawPrint(*outh,0,0);
    puts("X2");*/
    
    Rule::setOfMPs ret;


//  set<atom> child_nodes;
//  find_child_nodes(outh, child_nodes);
    
    Vertex child = CreateVar(destTable);
    
    for(tree<Vertex>::pre_order_iterator i = outh->begin(); i != outh->end(); i++)
//  for (set<atom>::iterator i = child_nodes.begin(); i != child_nodes.end(); i++)
    {       
/*      puts("-");
        printAtomTree(outh,0,0);
*/
        Vertex old_i = *i;
        *i = child;
        BBvtree templated_atom1(new BoundVTree(*outh));
        *i = old_i;     
        
        BBvtree inhPattern1(new BoundVTree(mva((Handle)INHERITANCE_LINK,
            mva(child), mva(*i))));
        
        MPs ret1;
        ret1.push_back(inhPattern1);
        ret1.push_back(templated_atom1);
        
/*      puts("X");
        rawPrint(*outh,0,0);
        puts("-");
        rawPrint(*templated_atom1,0,0);
        puts("-");
        rawPrint(*inhPattern1,0,0);
        puts("-");*/
        
        overrideInputFilter = true;
        
        ret.insert(ret1);
    }
    
    
    return ret;
}
#endif


#endif //001

} //namespace reasoning


#if 0
Handle Equi2ImpRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
    const int n = premiseArray.size();
    assert(n==1);
    
    pair<Handle, Handle> p = Equi2ImpLink(premiseArray[0]);
    
    return Join<AND_LINK>(p.first, p.second);
}
#endif

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
            swap<C>(seed[index], seed[i]);

            createPermutation<C>(seed, result, index+1);

            swap<C>(seed[index], seed[i]); //Cancel
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

namespace haxx
{
    /// Must invert the formula I*(I+1)/2 for 2-level incl. excl. in OR rule

    map<int, int> total2level1;
    int contractInclusionExclusionFactorial(int total)
    {
        map<int, int>::iterator T2L = total2level1.find(total);

        if(T2L != total2level1.end())
            return T2L->second;

        int val=0;
        int i=0;
        
        for (i = total2level1.size(); val != total; i++)
            val = total2level1[i] = i*(i+1)/2;

        return i-1;
    }
}

namespace reasoning
{

void pr(pair<Handle, Handle> i);
void pr2(pair<Handle, vtree> i);


template <class InputIterator,
                        class OutputIterator,
                        class UnaryOperation,
                        class UnaryPredicate>
OutputIterator transform_if(InputIterator first,
                                        InputIterator last,
                                        OutputIterator result,
                                        UnaryOperation op,
                                        UnaryPredicate test)
{
        while (first != last) {
                if (test(*first))
                        *result++ = op(*first);
                     else
                                *result++ = *first;
                ++first;
        }
        return result;
}

template<typename T>
struct returnor  : public unary_function<T,T> 
{
    const T& ret;
    returnor(const T& _ret) : ret(_ret) {}
    T operator()(const T& _arg) const { return ret; }
};

template<typename T>
struct converter  : public binary_function<T,T,void> 
{
    T src, dest;
    converter(const T& _src, const T& _dest) : src(_src), dest(_dest) {}
    void operator()(T& _arg) {
        if (_arg == src)
            _arg = dest; }
};

Btr<vtree> convert_all_var2fwvar(vtree vt_const, iAtomTableWrapper* table)
{
    AtomSpace *nm = CogServer::getAtomSpace();

    Btr<vtree> ret(new vtree(vt_const));
/// USE THIS IF YOU WISH TO CONVERT VARIABLE_NODEs to FW_VARIABLE_NODEs!
    vtree::iterator vit = ret->begin();

    while(  (vit =  find_if(ret->begin(), ret->end(),
                                bind(equal_to<Type>(),
                                    bind(getTypeVFun, _1),
                                    (Type)(int)VARIABLE_NODE
                                )
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
        printer.print(v2h(*vit), 4);
        printer.print(ret->begin(), 4);
#endif
    }

    return ret;
}

//Btr<ModifiedVTree> convertToModifiedVTree(Btr<vtree> vt)
Btr<ModifiedVTree> convertToModifiedVTree(Handle h, Btr<vtree> vt)
{
    return Btr<ModifiedVTree>(new ModifiedVTree(*vt, h));
}
/*Btr<ModifiedVTree> convertToModifiedVTree(vtree& vt)
{
    return Btr<ModifiedVTree>(new ModifiedVTree(vt));
}*/

/*template<typename T2, typename T3>
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
};*/

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
    for (bindContainerIterT b = b2start;
        b!= b2end;
        b++)
    {
        bindContainerIterT bit;

        if ((bit = b1.find(b->first)) != b1.end() &&
            !(bit->second == b->second))
        {
            return false;

/*          ///The same var bound different way. First virtualize them:

            vtree binder1(make_vtree(v2h(b->second)));
            vtree binder2(make_vtree(v2h(bit->second)));

            /// Then apply all bindings on both sides to both, to "normalize away" dependencies

            Btr<vtree> binder1A(tree_transform(binder1,   mapper<T1, bindContainerIterT>(b1, b1start, b1end)));
            Btr<vtree> binder1B(tree_transform(*binder1A, mapper<T1, bindContainerIterT>(b2, b2start, b2end)));
            Btr<vtree> binder2A(tree_transform(binder2,   mapper<T1, bindContainerIterT>(b1, b1start, b1end)));
            Btr<vtree> binder2B(tree_transform(*binder2A, mapper<T1, bindContainerIterT>(b2, b2start, b2end)));

            return *binder2B == *binder1B; //Check if it's still inconsistent.*/
        }
    }

    return true;
}

void insert_with_consistency_check_bindingsVTreeT(map<Handle, vtree>& m, map<Handle, vtree>::iterator rstart, map<Handle, vtree>::iterator rend)
{
    if (consistent_bindingsVTreeT<Vertex>(m, rstart, rend))
        m.insert(rstart, rend);
    else
        throw PLNexception("InconsistentBindingException");
}

Btr<set<Handle> > ForAll_handles;

Btr<ModifiedBoundVTree> FindMatchingUniversal(meta target, Handle ForAllLink, iAtomTableWrapper* table)
{
	cprintf(4,"FindMatchingUniversal...");
	
    Btr<ModifiedVTree> candidate = 
        convertToModifiedVTree(
                            ForAllLink,
                            convert_all_var2fwvar(
                                make_vtree(getOutgoingFun()(ForAllLink, 1)),
                                table));

    Btr<bindingsVTreeT> bindsInUniversal    (new bindingsVTreeT),
                                bindsInTarget       (new bindingsVTreeT);

    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(candidate->begin(), 4);
    printer.print(target->begin(), 4);

    if (!unifiesTo(*candidate, *target, *bindsInUniversal, *bindsInTarget, true)) //, VARIABLE_NODE))
        return Btr<ModifiedBoundVTree>();

    printer.print(candidate->begin(), 4);
    printer.print(candidate->original_handle, 4);

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
    insert_with_consistency_check_bindingsVTreeT(*bindsCombined, bindsInTarget->begin(), bindsInTarget->end());

    ///Remove FW_VAR => FW_VAR mappings. WARNING! Potentially goes to infinite loop.
    removeRecursionFromMap<bindingsVTreeT::iterator, vtree::iterator>(bindsCombined->begin(), bindsCombined->end());

	cprintf(4,"\ncombined binds:");
    for_each(bindsCombined->begin(), bindsCombined->end(), pr2);

    /**     /// Previously:
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
    }*/

    printer.print(BoundUniversal->begin(), 4);

    /// Previously:
    /// All recursion was removed from the combined binds, but only the binds pertaining
    /// to the variables that occurred in target are relevant and will hence be included.

    /*          for (bindingsVTreeT::iterator it = bindsCombined->begin();
    it!= bindsCombined->end();
    it++)
    if (STLhas(*bindsInTarget, it->first))
    (*BoundUniversal->bindings)[it->first] = it->second;*/

    /// Now:
    /// I changed the unifiesTo function so that lhs/rhs binding distinction was blurred.
    /// TODO: Verify that this is correct: now including bindings on both sides!

    BoundUniversal->bindings = bindsCombined;

	cprintf(4,"\nresult binds:");
    for_each(BoundUniversal->bindings->begin(), BoundUniversal->bindings->end(), pr2);

    return BoundUniversal;
}

Btr< set<Btr<ModifiedBoundVTree> > > FindMatchingUniversals(meta target, iAtomTableWrapper* table)
{
    DeclareBtr(set<Btr<ModifiedBoundVTree> >, ret);

    if (!ForAll_handles)
    {
            ForAll_handles = table->getHandleSet(FORALL_LINK, "");
        puts("recreated ForAll_handles");
        getc(stdin);
    }

    foreach(Handle h, *ForAll_handles)
    {
        Btr<ModifiedBoundVTree> BoundUniversal = FindMatchingUniversal(target, h, table);
        if (BoundUniversal)
            ret->insert(BoundUniversal);
    }

    return ret;
}

void Rule::CloneArgs(const Rule::MPs& src, Rule::MPs& dest)
{
    foreach(const BBvtree& bbvt, src)
        dest.push_back(Btr<BoundVTree>(bbvt->Clone()));
}

} //namespace reasoning

#if 0
Btr< set<Btr<ModifiedBoundVTree> > > FindMatchingUniversals(meta target, iAtomTableWrapper* table)
{
    DeclareBtr(set<Btr<ModifiedBoundVTree> >, ret);
    Btr< set<Btr<ModifiedVTree> > > ForAllLinkRepo;

    if (!ForAllLinkRepo)
    {
        if (!ForAll_handles)
            ForAll_handles = table->getHandleSet(FORALL_LINK, "");

        ForAllLinkRepo.reset(new set<Btr<ModifiedVTree> >);

        /// Handle => vtree and then convert VariableNodes to FW_VariableNodes

        transform(  ForAll_handles->begin(),
                        ForAll_handles->end(),
                        inserter(*ForAllLinkRepo, ForAllLinkRepo->begin()),
                        bind(&convertToModifiedVTree,
                            _1,
                            bind(&convert_all_var2fwvar,
                                bind(&make_vtree,
                                    bind(getOutgoingFun(), _1, 1)),
                                table))); /// Ignore arg #0 ListLink)

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
        Btr<bindingsVTreeT> bindsInUniversal    (new bindingsVTreeT),
                            bindsInTarget       (new bindingsVTreeT);

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
            insert_with_consistency_check_bindingsVTreeT(*bindsCombined, bindsInTarget->begin(), bindsInTarget->end());

            ///Remove FW_VAR => FW_VAR mappings. WARNING! Potentially goes to infinite loop.
            removeRecursionFromMap<bindingsVTreeT::iterator, vtree::iterator>(bindsCombined->begin(), bindsCombined->end());

            cprintf(4,"\ncombined binds:");
            for_each(bindsCombined->begin(), bindsCombined->end(), pr2);

/**     /// Previously:
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
            }*/

#if 0
            rawPrint(*BoundUniversal, BoundUniversal->begin(), 4);
#else 
            printer.print(BoundUniversal->begin(), 4);
#endif

            /// Previously:
            /// All recursion was removed from the combined binds, but only the binds pertaining
            /// to the variables that occurred in target are relevant and will hence be included.

/*          for (bindingsVTreeT::iterator it = bindsCombined->begin();
                                                    it!= bindsCombined->end();
                                                    it++)
                if (STLhas(*bindsInTarget, it->first))
                    (*BoundUniversal->bindings)[it->first] = it->second;*/

            /// Now:
            /// I changed the unifiesTo function so that lhs/rhs binding distinction was blurred.
            /// TODO: Verify that this is correct: now including bindings on both sides!

            BoundUniversal->bindings = bindsCombined;


            cprintf(4,"\nresult binds:");

            for_each(BoundUniversal->bindings->begin(), BoundUniversal->bindings->end(), pr2);

            ret->insert(BoundUniversal);
        }
    }

    return ret;
}

meta SimSubstRule1::i2oType(const vector<Vertex>& h) const
{
    AtomSpace *nm = CogServer::getAtomSpace();

    Handle h0 = v2h(h[0]);
    Handle h1 = v2h(h[1]);
    
    const int N = h.size();
    assert(2==N);
    assert(nm->getType(h0) == INHERITANCE_LINK);
    
    // ( any, Inh(a,b) )
    

    atom ret(h1);
    
    //assert(ret.hs[1].real == nm->getOutgoing(h[1])[0]);

    vector<Handle> hs = nm->getOutgoing(h0);

    /// subst hs[0] to hs[1] (child => parent):
    ret.hs[0]->substitute(atom(hs[1]), atom(hs[0]));

//printAtomTree(ret,0,0);
    
/*  meta ret(new Tree<Vertex>(mva(nm->getType(v2h(h[0])),
        mva(nm->getOutgoing(v2h(h[1]))[0]),
        mva(nm->getOutgoing(v2h(h[0]))[1])));*/
    
	return BBvtree(new BoundVTree(ret.makeHandletree(destTable)));
}

/*
Rule::setOfMPs SimSubstRule2::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    //      if (outh.hs.size() != 2 || outh.hs[1].T == FW_VARIABLE_NODE || inheritsType(outh.hs[1].T,LINK))
    return Rule::setOfMPs();
    
    string varname1 = ("$"+GetRandomString(10));
    
    Rule::setOfMPs ret(new set<boost::shared_ptr<MPs> >);
    atom child(FW_VARIABLE_NODE, varname1);
    
    boost::shared_ptr<atom> inhPattern2(new atom(INHERITANCE_LINK, 2,
        new atom(child), new atom(outh.hs[1])));
    
    boost::shared_ptr<MPs> ret2(new MPs);
    atom *templated_atom2 = new atom(outh); //RHS template
    templated_atom2->hs[1] = child;
    ret2->push_back(boost::shared_ptr<atom>(templated_atom2));
    ret2->push_back(inhPattern2);
    
    overrideInputFilter = true;
    
    ret->insert(ret2);
    
    return ret;
}*/


#endif
