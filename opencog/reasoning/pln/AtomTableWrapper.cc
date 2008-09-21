#include "AtomTableWrapper.h"

#include "utils/XMLNodeLoader.h"
#include "rules/Rules.h"

#include <SimpleTruthValue.h>
#include <Logger.h>
#include <CogServer.h> // To get access to AtomSpace
#include <tree.h>
#include <utils2.h>
#include <utils.h>

#include  <boost/foreach.hpp>
//boost::variant<int, char> bv; //TODELETE

// Mind Shadow is an atom cache system but no longer used
#define USE_MIND_SHADOW 0
#define ARCHIVE_THEOREMS 1

// MACRO for getting the AtomSpace
// @todo: this should be replaced with initialising the ATW with a AtomSpace
// pointer/reference and using that. This should actually store an interface
// that AtomSpace provides but OpenCog currently doesn't indicate what this
// should be.
#define AS_PTR (CogServer::getAtomSpace())

using namespace std;
using namespace opencog;

// TODELETE Not used anywhere
//#ifndef USE_PSEUDOCORE
//void HandleEntry2HandleSeq(HandleEntry& src, vector<Handle>& dest)
//{ 
//    int array_size=0;
//    Handle *dest_array = new Handle[1000];
//
//    src.toHandleVector(dest_array, array_size);
//    for (int i =0;i<array_size;i++)
//    {
//        dest.push_back(dest_array[i]);
//    }
//
//    delete dest_array;
//}
//#endif
namespace haxx
{
    // TODO: make this accessable as a Singleton from a static member of AtomTableWrapper?
    reasoning::iAtomTableWrapper* defaultAtomTableWrapper;

    // TODELETE, not used
    // set<Handle> atomRemoveSet;
    
    //! childOf records what atoms are connected by inheritance links, 1st is
    //! parent, second is child (I think...)
    multimap<Handle,Handle> childOf;

    //! Whether to stores variable nodes as atoms in the AtomSpace
    bool AllowFW_VARIABLENODESinCore = true;

    //! ?
    map<string,Handle> variableShadowMap;
    bool ArchiveTheorems = true;
    
#if USE_MIND_SHADOW
    vector<Handle> mindShadow;
    map<Type, vector<Handle> > mindShadowMap;
#endif
}

//static map<Handle,vtree> h2vtree_cache;
vtree reasoning::make_vtree(Handle h)
{
    // Makes vtree for internal PLN use, so don't convert to real 
    // AtomSpace Handles
    
    /// @todo haxx:: Re-enable cache. It must simply be updated so that (pseudo)core reset takes it into account.
    /* map<Handle,vtree>::iterator i = h2vtree_cache.find(h);
       if (i != h2vtree_cache.end())
          return i->second;*/
    vtree ret;
    reasoning::makeHandletree(h, true, ret);
    // h2vtree_cache[h] = ret;

    reasoning::printTree(h,0,0);
    rawPrint(ret, ret.begin(), 0);

    return ret;
}

//#ifdef USE_PSEUDOCORE

float ContradictionLimit = 0.1f; //Below the limit, we just revise the differences.

//const TruthValue& AtomTableWrapper::getTV(Handle h)
//{
//  return AS_PTR->getTV(h);
//}

//namespace reasoning
//{
//shared_ptr<set<Handle> > AtomTableWrapper::getHandleSet(Type T, const string& name, bool subclass) const
//{
//    vector<Handle> hs(AS_PTR->getHandleSet(T,name,subclass));
//    return shared_ptr<set<Handle> >(new set<Handle>(hs.begin(), hs.end()));
//}
//
//Handle AtomTableWrapper::getHandle(Type t,const string& str) const
//{
//  AtomSpace *a = AS_PTR;
//  return AS_PTR->getHandle(t, str);
//}

//Handle AtomTableWrapper::getHandle(Type t,const HandleSeq& outgoing) const
//{
//  return AS_PTR->getHandle(t, outgoing);
//}

//}; //namespace reasoning

//#else

namespace reasoning
{

void AtomTableWrapper::HandleEntry2HandleSet(HandleEntry& src, set<Handle>& dest) const
{   
    std::vector<Handle> dest_array;

    src.toHandleVector(dest_array);
    for (unsigned int i =0;i<dest_array.size();i++)
    {
        dest.insert(dest_array[i]);
    }
}

bool AtomTableWrapper::inheritsType(Type T1, Type T2)
{
    return ClassServer::isAssignableFrom(T2, T1);
}


bool AtomTableWrapper::isSubType(Handle h, Type T)
{
    Handle r = fakeToRealHandle(h).first;
    return inheritsType(AS_PTR->getType(r), T);
}

Handle AtomTableWrapper::child(const Handle h, const int i) { 
    vhpair r = fakeToRealHandle(h);
    Handle o = AS_PTR->getOutgoing(r.first, i);
    // will require the child to have a version handle with the same context as
    // h.
    // check that VersionHandle in r.second exists, unless it is
    // NULL_VERSION_HANDLE
    // check a version handle r.second exists in o
    if (hasAppropriateContext(o,r.second)) {
        return realToFakeHandle(o,r.second);
    } else {
        // if not, return a NULL_HANDLE
        throw RuntimeException(TRACE_INFO, "no child with same context at position %d", i);
    }
}

bool AtomTableWrapper::hasAppropriateContext(const Handle o, VersionHandle& vh) const
{
    if (vh == NULL_VERSION_HANDLE || !AS_PTR->getTV(o,vh).isNullTv()) {
        return true;
    } else {
        // else check super/parent contexts
        // TODO: perhaps only the immediate parent context is needed?
        Handle super = vh.substantive;
        while ( (super = getSuperContext(super)) ) {
            VersionHandle vh2(CONTEXTUAL, super);
            if (!AS_PTR->getTV(o,vh2).isNullTv()) {
                vh.substantive = super;
                return true;
            }
        }
    }
    return false;
}

Handle AtomTableWrapper::getSuperContext(const Handle sub) const
{
    // Check if there exists a inheritance link
    HandleEntry *he = TLB::getAtom(sub)->getNeighbors(false, true,
            INHERITANCE_LINK);
    Handle h;
    HandleEntry::filterSet(he,CONCEPT_NODE,false); 
    if (he->getSize() > 1) {
        throw RuntimeException(TRACE_INFO, "multiple super/parent contexts, %d in fact",
                he->getSize());
    }
    if (he->getSize() == 0) {
        return NULL;
    }
    h = he->handle;
    delete he;
    return h;
}

bool AtomTableWrapper::isSubcontextOf(const Handle sub, const Handle super)
{
    // Handles should already be real as they are for contexts
    
    // get the name of the super
    std::string superName = AS_PTR->getName(fakeToRealHandle(super).first);

    // Check if there exists a inheritance link
    HandleEntry *he = TLB::getAtom(sub)->getNeighbors(false, true,
            INHERITANCE_LINK);
    HandleEntry::filterSet(he,superName.c_str(),CONCEPT_NODE,false); 

    // TODO: Do we need to check recursively until there are no more super
    // contexts?

    bool result = he->getSize() ? true : false;
    delete he;
    return result;
}

vhpair AtomTableWrapper::fakeToRealHandle(const Handle h) const
{
    // Don't map Handles that are Types
    if ((long) h <= NOTYPE) {
        return vhpair(h,NULL_VERSION_HANDLE);
    }
    vhmap_t::const_iterator i = vhmap.find(h);
    if (i != vhmap.end()) {
        // return existing fake handle
        // TODO: check that real Handle is still valid
        return i->second;
    } else {
        throw RuntimeException(TRACE_INFO, "Invalid fake handle %u", h);
    }

}

Handle AtomTableWrapper::realToFakeHandle(Handle h, VersionHandle vh)
{
    // check if already exists
    //vhmap_t::left_const_iterator l = vhmap.left.find(vhpair(h,vh));
    vhmap_t::const_iterator i = findValueInMap(vhmap,vhpair(h,vh));
    if (i != vhmap.end()) {
        // return existing fake handle
        return i->first;
    } else {
        // add to vhmap
        Handle fakeHandle = vhmap.size() + mapOffset;
        if (fakeHandle < mapOffset) {
            // Error: too many version to handle mappings!
            Logger().error("too many version-to-handle mappings!");
            exit(-1);
        }
        vhmap.insert( vhmap_pair_t(fakeHandle, vhpair(h,vh)) );
        return fakeHandle;
    }

}

std::vector< Handle > AtomTableWrapper::realToFakeHandle(const Handle h) {
    std::vector< Handle > result;
    result.push_back(realToFakeHandle(h, NULL_VERSION_HANDLE));
    foreach(VersionHandle vh, dummyContexts) {
        // if dummyContext is a VersionHandle for h
        if (AS_PTR->containsVersionedTV(h, vh)) {
            result.push_back( realToFakeHandle(h, vh));
        }
    }
    return result;
}

std::vector< Handle > AtomTableWrapper::realToFakeHandle(std::vector< Handle > hs, bool expand) {
    std::vector<Handle> result;
    foreach (Handle h, hs) {
        mergeCopy(result,realToFakeHandle(h));
    }
    return result;
}

const TruthValue& AtomTableWrapper::getTV(Handle h)
{
    AtomSpace* a = AS_PTR;
    if (h) {
        vhpair r = fakeToRealHandle(h);
        return a->getTV(r.first,r.second);
    } else {
        return TruthValue::TRIVIAL_TV();
    }
}

shared_ptr<set<Handle> > AtomTableWrapper::getHandleSet(Type T, const string& name, bool subclass) 
{
    HandleEntry* result = 
        (name.empty()
            ? AS_PTR->getAtomTable().getHandleSet((Type) T, subclass)
            : AS_PTR->getAtomTable().getHandleSet(name.c_str(), (Type) T, subclass));
    shared_ptr<set<Handle> > ret(new set<Handle>);

    HandleEntry2HandleSet(*result, *ret);
    delete result;

    shared_ptr<set<Handle> > retFake(new set<Handle>);
    foreach (Handle h, *ret) {
        foreach(Handle h, realToFakeHandle(h)) {
            (*retFake).insert(h); 
        }
    }
    return retFake;
}

Handle AtomTableWrapper::getHandle(Type t,const string& name) 
{
    // ARI: Behaviour of getHandle for PLN is undefined, because t and name
    // don't uniquely define an atom.
    return realToFakeHandle(AS_PTR->getAtomTable().getHandle(name.c_str(), (Type)t),
            NULL_VERSION_HANDLE);
}

bool AtomTableWrapper::equal(const HandleSeq& lhs, const HandleSeq& rhs)
{
    size_t lhs_arity = lhs.size();
    if (lhs_arity != rhs.size())
        return false;
        
    for (unsigned int i = 0; i < lhs_arity; i++)
        if (lhs[i] != rhs[i])
            return false;
    return true;            
}

Handle AtomTableWrapper::getHandle(Type t,const HandleSeq& outgoing)
{
    // ARI: Behaviour of getHandle for PLN is undefined, because t and outgoing 
    // don't uniquely define an atom.
    
    //HandleEntry* results = 
    //  AS_PTR->getHandleSet((Type)t, true);
//      while (results && !equal(results->handle->getOutgoingSet(), outgoing))
//          results = results->next;
//      Handle ret = (results ? results->handle : NULL);
//      delete results;
    
    return realToFakeHandle(AS_PTR->getHandle(t,outgoing),NULL_VERSION_HANDLE);
}

void AtomTableWrapper::reset()
{
    dummyContexts.clear();
    vhmap.clear();
    ::haxx::childOf.clear();
    AS_PTR->clear();
}

//#endif

extern int atom_alloc_count;
//void initArbitraryAtoms();
//void printUtable();
//void SetTheoremTest();
//void existence2universality();
//extern bool linkNotifications;
bool linkNotifications=false;

static Handle U;

bool AtomTableWrapper::prepare()
{
    // TODO: Replace srand with opencog::RandGen
    srand(12345678);
    linkNotifications = true;
    return true;
}
bool AtomTableWrapper::loadAxioms(const string& path)
{
    // TODO: check exists works on WIN32
    string fname(path);
    string fname2("../tests/reasoning/" + path);
    if (!exists(fname.c_str())) {
        printf("File %s doesn't exist.\n", fname.c_str());
        fname = fname2;
    }
    if (!exists(fname.c_str())) {
        printf("File %s doesn't exist.\n", fname.c_str());
        return false;
    }
    
    try {
        printf("Loading axioms from: %s \n", fname.c_str());
        U = LoadXMLFile(this, fname);
        loadedFiles.insert(fname);
    } catch(string s) { 
        LOG(0, s); 
        return false; 
    } catch(...) { 
        LOG(0, "UNKNOWN EXCEPTION IN LOADAXIOMS!!!"); 
        return false; 
    }

    return true;
}

bool AtomTableWrapper::loadOther(const string& path, bool ReplaceOld)
{
    string buf;
    LoadTextFile(path, buf);

    vector<string> lines = StringTokenizer(buf, "\n\r").WithoutEmpty();

//  SetUniverseSize(CONST_SIZE, lines.size());

    for (uint i = 0; i < lines.size(); i++)
    {
        vector<string> mainelems = StringTokenizer(lines[i], "(").WithoutEmpty();
        if (mainelems.size()<2)
            continue;

        float percentage = 0.0f;

        percentage = atof(StringTokenizer(mainelems[1], "%")[0].c_str());

        vector<string> elems = StringTokenizer(mainelems[0], "\t ").WithoutEmpty();


        SimpleTruthValue tv(percentage/100.0f, 1);

        if (elems.size() == 1)
            addNode(CONCEPT_NODE, elems[0],
                tv,
                false,
                false);
        else if (!elems.empty())
        {
            vector<Handle> hs;

            for (unsigned int j = 0; j < elems.size(); j++)
                if (!elems[j].empty())
                    hs.push_back(AS_PTR->getHandle(CONCEPT_NODE, elems[j]));
               
            assert (hs.size()>1);

            addLink(AND_LINK, hs, tv, false, false);
        }
    }

    loadedFiles.insert(path);

    return true;
}

/*vector<atom> AtomTableWrapper::LoadAnds(const string& path)
{
    string buf;
    LoadTextFile(path, buf);

    vector<atom> ret;

    puts("Adding the ANDs");

    vector<string> lines = StringTokenizer(buf, "\n\r").WithoutEmpty();
    for (int i = 0; i < lines.size(); i++)
    {
        vector<string> elems = StringTokenizer(lines[i], "\t ").WithoutEmpty();

        if(elems.size() ==1)
        {
            puts("Warning! 1-node link!");
            puts(elems[0].c_str());
        }

        if (elems.size()>=2)
        {
            vector<atom> hs;

            for (int j = 0; j < elems.size(); j++)
                if (!elems[j].empty())
                    hs.push_back(atom(CONCEPT_NODE, elems[j]));

            if (hs.size()==1)
            {
                puts("Warning! 1-node link!");
                printAtomTree(hs[0]);
            }
            else
                ret.push_back(atom(AND_LINK, hs));
        }
        printf("%d\n", i);
    }

    puts("ANDs ok");

    return ret;
}

*/

///////////

#define P_DEBUG 0

template<typename T>
bool cutVector(const vector<T>& src, int index, vector<T>& dest)
{
    dest.clear();

    for (int i = 0; i < src.size(); i++)
        if (i != index)
            dest.push_back(src[i]);

    return index < src.size();
}

Handle NormalizingATW::addNode(Type T, const string& name, const TruthValue& tvn,bool fresh,bool managed)
{
    //Handle ret = a->addNode(T, name, tvn, fresh);
    Handle ret = FIMATW::addNode(T, name, tvn, fresh,managed);

    return ret;
}

int AtomTableWrapper::getFirstIndexOfType(const HandleSeq hs, const Type T) const
{
    AtomSpace *a = AS_PTR;
    for (unsigned int i = 0; i < hs.size(); i++)
        if (a->getType(fakeToRealHandle(hs[i]).first) == T)
            return i;

    return -1;
}
#if 0
void remove_redundant(HandleSeq& hs)
{
char b[200];
sprintf(b, "H size before removal: %d.", hs.size());
LOG(5, b);
    
    for (vector<Handle>::iterator ii = hs.begin(); ii != hs.end(); ii++)
    {
        atom atom_ii(*ii);

        for (vector<Handle>::iterator jj = ii; jj != hs.end();)
            if (++jj != hs.end())
                if (atom(*jj) == atom_ii)
/*              if (TheHandleWrapperFactory.New(*jj)->equalOLD(
                            *TheHandleWrapperFactory.New(*ii)
                  ))*/
                {
#ifndef WIN32
    THE FOLLOWING MAY NOT WORK OUTSIDE WIN32:
#endif

sprintf(b, "Removing %s (%d).", a->getName(*ii).c_str(), nm->getType(*ii));
LOG(5, b);

                    jj = hs.erase(jj);
                    jj--;
                }
    }

sprintf(b, "H size AFTER removal: %d.", hs.size());
LOG(5, b);

/*  for (i = 0; i < hs.size(); i++)
        hws.push_back(TheHandleWrapperFactory.New(hs[i]));

    for (vector<HandleWrapper*>::iterator ii = hws.begin(); ii != hws.end(); ii++)
        for (vector<HandleWrapper*>::iterator jj = ii; jj != hws.end();)
            if (++jj != hws.end())
                if ((*jj)->equalOLD(**ii))
                {
#ifndef WIN32
    THE FOLLOWING MAY NOT WORK OUTSIDE WIN32:
#endif
                    jj = hws.erase(jj);
                    jj--;
                }*/
}
#endif

bool AtomTableWrapper::binaryTrue(Handle h)
{
    const TruthValue& tv = getTV(fakeToRealHandle(h).first);

    return (tv.getMean() > PLN_TRUE_MEAN);
}

bool AtomTableWrapper::symmetricLink(Type T)
{
    // TODO: there is a symmetric link super class...
    // perhaps we should just check for that? Or is it
    // PLN specific.
    return inheritsType(T, AND_LINK) || inheritsType(T, LIST_LINK)
            || inheritsType(T, OR_LINK);
}

bool AtomTableWrapper::is_empty_link(Handle h)
{
    AtomSpace *a = AS_PTR;
    return !inheritsType(a->getType(h), NODE)
            && a->getArity(h) == 0;
}

bool AtomTableWrapper::hasFalsum(HandleSeq hs)
{
    AtomSpace *a = AS_PTR;
//  return false;

    for (vector<Handle>::const_iterator ii = hs.begin(); ii != hs.end(); ii++)
    {
        const Handle key = *ii;

        if (inheritsType(getType(key), FALSE_LINK) ) //Explicit falsum
            return true;

        for (vector<Handle>::const_iterator jj = hs.begin(); jj != hs.end();jj++)
            if (jj != ii)
                if (inheritsType(getType(*jj), NOT_LINK) ) //Contradiction
                {
                    Handle notter = getOutgoing(*jj)[0];
                    if (notter == key)
                        return true;
                }
    }

    return false;
}

bool AtomTableWrapper::containsNegation(Handle ANDlink, Handle h)
{
    HandleSeq hs = getOutgoing(ANDlink);

    hs.push_back(h);

    return hasFalsum(hs);
}

Handle AtomTableWrapper::freshened(Handle h, bool managed)
{
    Type T = getType(h);
    HandleSeq hs = getOutgoing(h);
    string name = getName(h);
    const TruthValue& tv = getTV(h);

    if (inheritsType(T, NODE))
        return addNode(T, name, tv, true,managed);
    else
    {
        for (unsigned int i = 0; i < hs.size(); i++)
            hs[i] = freshened(hs[i], managed);

        return addLink(T, hs, tv, true, managed);
    }
}

//#define BL 2
Handle NormalizingATW::addLink(Type T, const HandleSeq& hs,
        const TruthValue& tvn,bool fresh,bool managed)
{
    AtomSpace *a = AS_PTR;
    Handle ret=0;

    bool ok_forall=false;

    char buf[500];
    sprintf(buf, "Adding link of type %s (%d)", type2name[T].c_str(), T);
    LOG(4, buf);

    if (hs.size() > 7)
    {
        LOG(4, "Adding large-arity link!");
/*      if (TheLog.getLevel()>=5)
        {
            char t[100];
            gets(t);
        }*/
    }

#if 0
    if (T == IMPLICATION_LINK
        && hs.size()==2
        && inheritsType(a->getType(hs[0]), FALSE_LINK))
    {
        assert(hs.size()==2 || hs.empty());
        
        ret = hs[1];
    }
    else if (T == IMPLICATION_LINK
        && hs.size()==2
        && inheritsType(a->getType(hs[1]), FALSE_LINK))
    {
        assert(hs.size()==2 || hs.empty());

        HandleSeq NOTarg;
        NOTarg.push_back(hs[0]);
        
        ret = addLink(NOT_LINK, NOTarg, TruthValue::TRUE_TV(), fresh, managed);
    }
    else if (T == IMPLICATION_LINK                              //Accidentally similar to da above
            && hs.size()==2
            && inheritsType(a->getType(hs[0]), AND_LINK)
            && containsNegation(hs[0], hs[1]))
    {
        HandleSeq NOTarg;
        NOTarg.push_back(hs[0]);

        ret = addLink(NOT_LINK, NOTarg, TruthValue::TRUE_TV(), fresh, managed);
    }
    else if (T == IMPLICATION_LINK
        && !hs.empty()
        && inheritsType(a->getType(hs[1]), AND_LINK))
    {
        assert(hs.size()==2 || hs.empty());

        LOG(BL, "Cut A=>AND(B,C,...) into AND(A=>B, A=>C, ...)");
        
        HandleSeq imps;
        
        HandleSeq hs2 = a->getOutgoing(hs[1]);
        for (int i = 0; i < hs2.size(); i++)
        {
            HandleSeq new_hs;
            new_hs.push_back(hs[0]);
            new_hs.push_back(hs2[i]);
            
            imps.push_back( addLink(T, new_hs, tvn,fresh,managed) );
        }
        
        ret = addLink(AND_LINK,imps,TruthValue::TRUE_TV(),fresh,managed);
    }
    else
#endif
#if 0
    if (T == IMPLICATION_LINK
        && !hs.empty()
        && inheritsType(a->getType(hs[0]), IMPLICATION_LINK))
    {
        assert(hs.size()==2 || hs.empty());

        HandleSeq hs2 = a->getOutgoing(hs[0]);

        Handle c = hs[1];
        Handle a = hs2[0];
        Handle b = hs2[1];

//      LOG(BL, "=>( =>(A,B), C) ) into =>( |(B,~A),C ) into  =>( ~&(~B,A),C ) ");
        LOG(BL, "=>( =>(A,B), C) ) into &( =>( A&B, C), =>(~A, C) )");

        HandleSeq NOTa_args;
        NOTa_args.push_back(a);
        
        HandleSeq AND_args;
        AND_args.push_back(a);
        AND_args.push_back(b);

//      HandleSeq NOTAND_args;

        HandleSeq imps1, imps2;
        imps1.push_back(addLink(AND_LINK, AND_args, TruthValue::TRUE_TV(), fresh,managed));
        imps1.push_back(c);

        imps2.push_back(addLink(NOT_LINK, NOTa_args, TruthValue::TRUE_TV(), fresh,managed));
        imps2.push_back(c);
        
        HandleSeq new_hs;
        new_hs.push_back(addLink(IMPLICATION_LINK, imps1, TruthValue::TRUE_TV(), fresh,managed));
        new_hs.push_back(addLink(IMPLICATION_LINK, imps2, TruthValue::TRUE_TV(), fresh,managed));
        
        ret = addLink(AND_LINK,new_hs,TruthValue::TRUE_TV(),fresh,managed);
    }
#endif
/*  else if (T == AND_LINK
            && hs.size()==2
            && a->getType(hs[1]) != IMPLICATION_LINK)
    {
        LOG(0, "AND => Implication");
        cprintf(0,"%d\n",nm->getType(hs[1]));
        getc(stdin);
        
        TruthValue **tvs = new TruthValue *[3];
        tvs[0] = &tvn;
        tvs[1] = getTV(hs[0]);
        tvs[2] = getTV(hs[1]);
        
        TruthValue *impTV = ImplicationConstructionFormula().compute(tvs,3);

        HandleSeq new_hs;
        new_hs.push_back(addLink(T,hs,tvn,fresh,managed));
        new_hs.push_back(addLink(IMPLICATION_LINK,hs,impTV,fresh,managed));
        
        cprintf(0,"EEE %d\n",nm->getType(new_hs[1]));
        
        ret = addLink(AND_LINK, new_hs, TruthValue::TRUE_TV(), fresh,managed);

        LOG(0, "Adding AND-reformed:");
        printTree(ret, 0, 0);
        getc(stdin);
    }*/
    
#if 0
    else if (T == NOT_LINK
            && !hs.empty()
            && inheritsType(a->getType(hs[0]), NOT_LINK)
            && tvn.getMean() > 0.989
            && binaryTrue(hs[0]) )
    {
        LOG(BL, "~~A <---> A");

        HandleSeq relevant_args = a->getOutgoing(hs[0]);
        assert(relevant_args.size() == 1);
        Type relevant_type = a->getType(relevant_args[0]);

        //if (tvn.getMean() > 0.989 && binaryTrue(relevant_args[0]) )
        ret = a->getOutgoing(hs[0])[0]; //addLink(relevant_type, relevant_args,TruthValue::TRUE_TV(),fresh);
    }
    else if (T == AND_LINK
            && !hs.empty()
            && tvn.getMean() > 0.989
            && getFirstIndexOfType(hs, AND_LINK) >= 0
            && binaryTrue(hs[getFirstIndexOfType(hs, AND_LINK)]) )
            //&& inheritsType(a->getType(hs[0]), AND_LINK))
    {
        LOG(BL, "AND(AND(B), AND(A)) <---> AND(B,A)");

//      bool all_true_and = true;

        HandleSeq new_args;

        for (int ii = 0; ii < hs.size(); ii++)
        {
//          printTree(hs[ii],0,4);

            if (inheritsType(a->getType(hs[ii]), AND_LINK)
                && binaryTrue(hs[ii]))
            {
                HandleSeq args1 = a->getOutgoing(hs[ii]);

                for (int a = 0; a < args1.size(); a++)
                    new_args.push_back(args1[a]);
            }
            else
                new_args.push_back(hs[ii]);
        }

        ret = addLink(AND_LINK, new_args, tvn, fresh,managed);
#if P_DEBUG
        LOG(4, "Adding AND-reformed:");
        printTree(ret, 0, 4);
#endif
    }
    else if (T == AND_LINK
            && hs.size()==1
            && tvn.getMean() > 0.989)
    {
        LOG(BL, "AND(A) <---> A");

        ret = hs[0];
    }
    else if (T == AND_LINK)
    {
//for (int a=0;a<hs.size(); a++)
//  printTree(hs[a],0,4);

        LOG(BL, "&(..., A, ..., ~A, ...) => Falsum");

        if (hasFalsum(hs))
            ret = addNode(FALSE_LINK, "FALSE", tvn, fresh,managed);
        else
        {
            LOG(BL, "AND(A,B,A) <---> AND(A,B)");

            int original_size = hs.size();
            remove_redundant(hs);

            if (hs.size() != original_size) //Changed.
            {
                LOG(5, "CHANGES FROM AND(A,B,A) <---> AND(A,B)");

                ret = addLink(AND_LINK, hs, tvn, fresh,managed);
    
//printTree(ret,0,5);
    
                LOG(5, "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
            }
            else
            {
                LOG(5, "NO CHANGE FROM AND(A,B,A) <---> AND(A,B)");
            }
        }
    }
    else if (T == NOT_LINK
            && !hs.empty()
            && inheritsType(a->getType(hs[0]), IMPLICATION_LINK))
    {
        LOG(BL, "~(A=>B) <---> ~(B | ~A) <---> &(A,~B)");

        HandleSeq IMP_args = a->getOutgoing(hs[0]);
        assert(IMP_args.size() == 2);
    
        HandleSeq not_arg;
        not_arg.push_back(IMP_args[1]);

        HandleSeq new_args;
        new_args.push_back(IMP_args[0]);
        new_args.push_back(addLink(NOT_LINK, not_arg,TruthValue::TRUE_TV(),fresh,managed));

        ret = addLink(AND_LINK, new_args, tvn, fresh,managed);
    }
    else if (T == NOT_LINK
            && !hs.empty()
            && inheritsType(a->getType(hs[0]), AND_LINK))
    {
        LOG(BL, "Cut -AND(B,C,...) into AND(B=>-C, C=>-B, ...)");
        
        HandleSeq imps;
        
        HandleSeq and_args = a->getOutgoing(hs[0]);

        for (int i = 0; i < and_args.size(); i++)
        {
            HandleSeq new_and_args;
            cutVector<Handle>(and_args, i, new_and_args);

            HandleSeq new_imp_args;
            HandleSeq not_arg;
            not_arg.push_back(and_args[i]);

            new_imp_args.push_back(addLink(AND_LINK, new_and_args,TruthValue::TRUE_TV(), fresh,managed));
            new_imp_args.push_back(addLink(NOT_LINK, not_arg, TruthValue::TRUE_TV(), fresh,managed));                       
        
            imps.push_back( addLink(IMPLICATION_LINK, new_imp_args, TruthValue::TRUE_TV(), fresh,managed) );
        }
        
        ret = addLink(AND_LINK, imps, TruthValue::TRUE_TV(), fresh,managed);
    } 
#endif
/*  else if (T == IMPLICATION_LINK
            && hs.size()==2
            && tvn.getMean() > 0.989
            && binaryTrue(hs[1])
            && inheritsType(a->getType(hs[1]), IMPLICATION_LINK))
    {
        LOG(0, "(A=>(B=>C)) --->    ((A&B) => C)");
//exit(0);
        Handle old_head = hs[0];
        HandleSeq old_imp_args = a->getOutgoing(hs[1]);
        Handle tail = old_imp_args[1];

        HandleSeq new_and_args;

        new_and_args.push_back(old_head);
        new_and_args.push_back(old_imp_args[0]);

        HandleSeq new_imp_args;
        new_imp_args.push_back(addLink(AND_LINK, new_and_args, TruthValue::TRUE_TV(), fresh,managed));
        new_imp_args.push_back(tail);
        
        ret = addLink(IMPLICATION_LINK, new_imp_args, TruthValue::TRUE_TV(), fresh,managed);

//      printTree(ret);
    }*/

#if 0
    else if (T == IMPLICATION_LINK
            && !hs.empty()
            && inheritsType(a->getType(hs[0]), AND_LINK)
            && getFirstIndexOfType(a->getOutgoing(hs[0]), IMPLICATION_LINK) >= 0)
    {
        LOG(1, "Cut (A& (B=>C) ) => D   --->    (C & A) => D   &   (~B & A)=>D");

        assert(hs.size()==2);

        HandleSeq old_and_args = a->getOutgoing(hs[0]);
        Handle tail = hs[1];

//printTree(hs[1],0,1);

        int imp_index = getFirstIndexOfType(old_and_args, IMPLICATION_LINK);

/*LOG(1, "Old HS");
for (int a2=0;a2<hs.size(); a2++)
    printTree(hs[a2],0,1);

LOG(1, "Old AND");
for (int a1=0;a1<old_and_args.size(); a1++)
    printTree(old_and_args[a1],0,1);*/

        HandleSeq new_and_args1, new_and_args2;
        cutVector<Handle>(old_and_args, imp_index, new_and_args1);
        new_and_args2 = new_and_args1;

        HandleSeq internal_imp_args = a->getOutgoing(old_and_args[imp_index]);
        HandleSeq not_arg;
        not_arg.push_back(internal_imp_args[0]);
        
        new_and_args1.push_back(internal_imp_args[1]);
        new_and_args2.push_back(addLink(NOT_LINK, not_arg, TruthValue::TRUE_TV(), fresh,managed));

#if P_DEBUG
int a;
LOG(1, "New AND");
for (a=0;a<new_and_args1.size(); a++)
    printTree(new_and_args1[a],0,1);
#endif

        HandleSeq new_imp_args1, new_imp_args2;
        new_imp_args1.push_back(addLink(AND_LINK, new_and_args1, TruthValue::TRUE_TV(), fresh,managed));
        new_imp_args1.push_back(tail);
        new_imp_args2.push_back(addLink(AND_LINK, new_and_args2, TruthValue::TRUE_TV(), fresh,managed));
        new_imp_args2.push_back(tail);

#if P_DEBUG
LOG(1, "New IMP1");
for (a=0;a<new_imp_args1.size(); a++)
    printTree(new_imp_args1[a],0,1);
#endif

#if P_DEBUG
LOG(1, "New IMP2");
for (a=0;a<new_imp_args2.size(); a++)
    printTree(new_imp_args2[a],0,1);
#endif

        HandleSeq new_main_and_args;
        new_main_and_args.push_back(addLink(IMPLICATION_LINK, new_imp_args1, TruthValue::TRUE_TV(), fresh,managed));
        new_main_and_args.push_back(addLink(IMPLICATION_LINK, new_imp_args2, TruthValue::TRUE_TV(), fresh,managed));
        
        ret = addLink(AND_LINK, new_main_and_args, TruthValue::TRUE_TV(), fresh,managed);

#if P_DEBUG
LOG(1, "Collapsed:");
printTree(ret,0,1);
#endif

    }
#endif
    else if (T == EXTENSIONAL_EQUIVALENCE_LINK && hs.size()==2)
    //else if (hs.size()==2)
    {
        // Convert EXTENSIONAL_EQUIVALENCE_LINK into two IMPLICATION_LINKs
        // that are mirrored and joined by an AND_LINK.
        const HandleSeq EquiTarget = hs;
        HandleSeq ImpTarget1, ImpTarget2;
        
        for (HandleSeq::const_iterator i = EquiTarget.begin(); i != EquiTarget.end(); i++)
            ImpTarget1.push_back((*i));

        int zz=ImpTarget1.size();
        assert(ImpTarget1.size()==2);
        
        for (vector<Handle>::const_reverse_iterator j = EquiTarget.rbegin(); j != EquiTarget.rend(); j++)
            ImpTarget2.push_back((*j));
        
        assert(ImpTarget2.size()==2);
        
//      const TruthValue& outerTV = getTV(exL);
        
        HandleSeq ANDargs;

        ANDargs.push_back( addLink(IMPLICATION_LINK, ImpTarget1,
            tvn,
            true,managed));

        ANDargs.push_back( addLink(IMPLICATION_LINK, ImpTarget2,
            tvn,
            true,managed));

//      reverse(ANDargs.begin(), ANDargs.end());

        ret = addLink(AND_LINK, ANDargs, TruthValue::TRUE_TV(), fresh,managed);
    }
    else if (T == FORALL_LINK
            && hs.size() == 2
            && inheritsType(a->getType(hs[1]), AND_LINK)
            && binaryTrue(hs[1])
            && a->getArity(hs[1]) > 1)
    {
        // FORALL quantifier with AND_LINK is expanded into a LIST of FORALL
        // for each component within the AND.
        unsigned int AND_arity = a->getArity(hs[1]);

        HandleSeq fa_list;

        for (unsigned int i = 0; i < AND_arity; i++)
        {
            HandleSeq fora_hs;
            // ARI: How come for all links need the source to be freshened?
            fora_hs.push_back(freshened(hs[0],managed));
            fora_hs.push_back(child(hs[1],i));

            fa_list.push_back( addLink(FORALL_LINK, fora_hs,
                tvn,
                fresh,managed) );
        }

        assert(fa_list.size() == AND_arity);

        ret = addLink(LIST_LINK, fa_list, TruthValue::TRUE_TV(), fresh,managed);

//printTree(ret,0,0);
    }
#if  0
    else if (T==OR_LINK)
//          && tvn.getMean() > 0.989)
    {
        LOG(BL, "OR(A,B) <---> ~(~A AND ~B)");

        HandleSeq and_args;
        for (int i = 0; i < hs.size(); i++)
        {
            HandleSeq not_arg;
            not_arg.push_back(hs[i]);
            and_args.push_back(addLink(NOT_LINK, not_arg,TruthValue::TRUE_TV(),fresh,managed));
        }
    
        HandleSeq not_arg;
        not_arg.push_back(addLink(AND_LINK, and_args, tvn, fresh,managed));

        ret = addLink(NOT_LINK, not_arg,TruthValue::TRUE_TV(),fresh,managed);
    }
/*  else if (T==FORALL_LINK && hs.size()==2)
    {
ok_forall=true;
    }*/

/*  else if (AddToU
            && !inheritsType(T, FORALL_LINK))
    {
        HandleSeq emptys, forall_args;

        forall_args.push_back( addLink(LIST_LINK, emptys, TruthValue::TRUE_TV(), false,managed) );
        forall_args.push_back( addLink(T, hs, tvn, fresh,managed) );

        ret = addLink(FORALL_LINK, forall_args, TruthValue::TRUE_TV(), fresh,managed);
    }*/
#endif
    if (!ret)
    {
//      if (symmetricLink(T))
//          remove_if(hs.begin(), hs.end(), is_empty_link);

/*      if (T==AND_LINK)
        {

            int original_size = hs.size();
            remove_redundant(hs);
            assert (hs.size() == original_size);
        }*/

        LOG(5, "Adding to Core...");

        ret = FIMATW::addLink(T,hs,tvn,fresh,managed);
        //ret = a->addLink(T,hs,tvn,fresh,managed);

        LOG(5, "Added.");

#if P_DEBUG
        if (T == IMPLICATION_LINK && hs.size()==2 && a->getType(hs[1])==AND_LINK )
            printTree(ret, 0, 4);
#endif
    }
    return ret;
}

#if 0

LocalATW::LocalATW()
: capacity(0)
{
puts("Loading classed to LocalATW..."); 
    for (int i = 0; i < NUMBER_OF_CLASSES; i++)
    {
        mindShadowMap[i] = shared_ptr<set<Handle> >(new set<Handle>);
        for (   HandleEntry* e = a->getHandleSet((Type)i, true);
                e && e->handle;
                e = e->next)
        {
        printf(".\n");
            mindShadowMap[i]->insert(e->handle);
        }
            
//      delete e;
    }   
}

shared_ptr<set<Handle> > LocalATW::getHandleSet(Type T, const string& name, bool subclass)
{
    assert(!subclass); ///Not supported (yet)
    return mindShadowMap[T];
}

bool equal_vectors(Handle* lhs, int lhs_arity, const vector<Handle>& rhs)
{
    if (lhs_arity != rhs.size())
        return false;
    for (int i=0; i< lhs_arity; i++)
        if (lhs[i] != rhs[i])
            return false;
        
    return true;
}

bool LocalATW::inHandleSet(Type T, const string& name, shared_ptr<set<Handle> > res, Handle* ret)
{
    for (set<Handle>::iterator i=res->begin(); i != res->end(); i++)
        if (inheritsType((*i)->getType(),NODE)
            &&  ((Node*)(*i))->getName() == name)
        {
            if (ret)
                *ret = *i;
            return true;
        }   

    return false;   
}

void LocalATW::SetCapacity(unsigned long atoms)
{
    capacity = atoms;
}

bool LocalATW::inHandleSet(Type T, const HandleSeq& hs, shared_ptr<set<Handle> > res, Handle* ret)
{
    const int N = hs.size();
    
    for (set<Handle>::iterator i=res->begin(); i != res->end(); i++)
        if ((*i)->getArity() == N &&
             equal_vectors((*i)->getOutgoingSet(),
                            N,
                            hs))
        {
            if (ret)
                *ret = *i;
            return true;
        }   

    return false;   
}

void LocalATW::ClearAtomSpace()
{
    int count=0;
    for (map<Type, shared_ptr<set<Handle> > >::iterator m=mindShadowMap.begin();
        m!= mindShadowMap.end(); m++)
    {
        for (set<Handle>::iterator i = m->second->begin(); i!=m->second->end(); i++)
        {
            delete *i;
            count++;
        }
        m->second->clear();
    }   
cprintf(1, "%d entries freed (%.2f Mb).\n", count, (count * sizeof(Link))/1048576.0 );
}



void LocalATW::DumpCore(Type T)
{
    if (T == 0)
    {
        for (map<Type, shared_ptr<set<Handle> > >::iterator m=mindShadowMap.begin();
            m!= mindShadowMap.end(); m++)
        {   
            if (m->first) //Type #0 is not printed out.
                DumpCore(m->first);
        }
    }
    else
        for (set<Handle>::iterator i = mindShadowMap[T]->begin();
             i!=mindShadowMap[T]->end(); i++)
        {
            printf("[%d]\n", *i);
            printTree(*i,0,0);
        }
}                                   

/// Ownership of tvn is given to this method

Handle LocalATW::addLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh, bool managed)
{
    Handle ret = NULL;
    
    if (fresh || !inHandleSet(T,hs,mindShadowMap[T],&ret))
    {
        if (capacity && mindShadowMap[T]->size() >= capacity && !q[T].empty())
        {
            LOG(2, "Above capacity, removing...");
            
//          set<Handle>::iterator remo = mindShadowMap[T]->begin();
            set<Handle>::iterator remo = q[T].front();
            if (*remo)
                delete *remo;
            mindShadowMap[T]->erase(remo);
            
            q[T].pop();
        }
        
        /// Ownership of tvn is passed forward
        
        Link* link = new Link( T,  hs, tvn);
        pair<set<Handle>::iterator, bool> si = mindShadowMap[T]->insert(link);
        if (managed)
            q[T].push(si.first);
        return link;
    }
    else
    {
        return ret;
    }
}

Handle LocalATW::addNode(Type T, const std::string& name, const TruthValue& tvn, bool fresh, bool managed)
{
LOG(4,"Adding node.."); 

    Handle ret = NULL;
    
    if (fresh || !inHandleSet(T,name,mindShadowMap[T],&ret))
    {
        if (capacity && mindShadowMap[T]->size() >= capacity && !q[T].empty())
        {
            LOG(2, "Above capacity, removing...");

            //set<Handle>::iterator remo = mindShadowMap[T]->begin();
            set<Handle>::iterator remo = q[T].front();
            if (*remo)
                delete *remo;
            mindShadowMap[T]->erase(remo);
            
            q[T].pop();
        }

        Node* node = new Node( T,  name,  tvn); 
        pair<set<Handle>::iterator, bool> si = mindShadowMap[T]->insert(node);
        if (managed)
            q[T].push(si.first);
LOG(4,"Node add ok.");      
        return node;
    }
    else
    {
LOG(4,"Node add ok.");              
        return ret;
    }
}

#endif

DirectATW::DirectATW()
{
}

//: FIMATW(NMCore::instance())
NormalizingATW::NormalizingATW()
{
}

// change to
// Handle AtomTableWrapper::addAtom(vtree& a, const TruthValue& tvn, bool fresh, bool managed, Handle associateWith)
Handle AtomTableWrapper::addAtom(vtree& a, const TruthValue& tvn, bool fresh, bool managed)
{
    return addAtom(a,a.begin(),tvn,fresh,managed);
}

Handle AtomTableWrapper::addAtom(vtree& a, vtree::iterator it, const TruthValue& tvn, bool fresh, bool managed)
{
    AtomSpace *as = AS_PTR;
    cprintf(3,"Handle AtomTableWrapper::addAtom...");
    rawPrint(a,it,3);
    
    HandleSeq handles;
    Handle head_type = boost::get<Handle>(*it);
    set<Handle> contexts;
    
    //assert(haxx::AllowFW_VARIABLENODESinCore || head_type != (Handle)FW_VARIABLE_NODE);
    
    if (isReal(head_type))
    {
        LOG(1, "Warning! Trying to add a real atom with addAtom(vtree& a), returning type!\n");
        return head_type;
    }

    for (vtree::sibling_iterator i = a.begin(it); i!=a.end(it); i++)
    {
        Handle *h_ptr = boost::get<Handle>(&*i);

        // ARI: These additions have false true, and a trivial TruthValue, does
        // this mean that the AtomSpace deals with merging them, but the new
        // TruthValue is heavily discounted?
        Handle added = (h_ptr && isReal(*h_ptr)) ?
            (*h_ptr) : addAtom(a, i, TruthValue::TRIVIAL_TV(), false, managed);
        handles.push_back(added);
        contexts.insert(fakeToRealHandle(added).second.substantive);
    }

/*  /// We cannot add non-existent nodes this way!
    assert (!inheritsType(head_type, NODE));*/
    
    if (contexts.size() > 1) {
        // TODO: uh oh, the contexts of the children are different, have to create a
        // new context that inherits from all of them
        LOG(1, "Contexts are different, implement me!\n");
    }
    // Provide new context to addLink function
    return addLink((Type)(int)head_type, handles, tvn, fresh,managed);
}

Handle AtomTableWrapper::directAddLink(Type T, const HandleSeq& hs, const TruthValue& tvn,
        bool fresh,bool managed)
{
    AtomSpace *a = AS_PTR;
    if (tvn.isNullTv())
    {
        LOG(0, "I don't like FactoryTruthValues, so passing "
                "NULL as TruthValue throws exception in AtomTableWrapper.cc.");
        throw RuntimeException(TRACE_INFO, "NULL TV passed to directAddLink");
    }

    LOG(3, "Directly adding...");

    Handle ret;
    uint arity = hs.size();
    
    if (T == INHERITANCE_LINK && arity==2) {
        // childOf keeps a record of inheritance
        // It was tempting to change childOf to use real handles so that maybe
        // fewer of these pairings need to be stored. But this won't work
        // because each Versioned link is specific.
        haxx::childOf.insert(hpair(hs[1], hs[0]));
    }

    // if we are archiving theorems, and trying to add a implication link
    // composed of AND as a source, and the TruthValue is essentially true
    if (haxx::ArchiveTheorems &&
        T == IMPLICATION_LINK && a->getType(hs[0]) == AND_LINK &&
        tvn.getConfidence() > PLN_TRUE_MEAN) {
            vector<Handle> args = getOutgoing(hs[0]);
            cprintf(-3,"THM for:");

            vtree thm_target(make_vtree(hs[1]));

            rawPrint(thm_target, thm_target.begin(), 3);
            LOG(0,"Takes:");
            
            foreach(Handle arg, args) {
                vtree arg_tree(make_vtree(arg));
                rawPrint(arg_tree, arg_tree.begin(), 0);
                CrispTheoremRule::thms[thm_target].push_back(arg_tree);
            }
            
            ///warning "Return value will be invalid!"
            // ARI: what does the above means?
            // TODO: check the contexts of the linked atoms
            ret = addLinkDC( FALSE_LINK, hs, tvn, fresh, managed);
    } else {
        // TODO: check the contexts of the linked atoms
        ret = addLinkDC( T, hs,  tvn, fresh, managed);
    }

    if (inheritsType(T, LINK) && !arity && T != FORALL_LINK) {
        // Link with no connections?
        printTree(ret,0,1);
        cprintf(1,"inheritsType(T, LINK) && !arity\n");
    }   
    
    if (!haxx::AllowFW_VARIABLENODESinCore) {
        foreach(Handle ch, hs) {

            assert(isReal(ch));
            if (getType(ch) == FW_VARIABLE_NODE) {
                printTree(ret,0,-10);
                cprintf(-10,"ATW: getType(ch) == FW_VARIABLE_NODE!");
                getc(stdin);
            }
            //throw string("a->getType(ch) == FW_VARIABLE_NODE") + i2str(ret);
            //assert(a->getType(ch) != FW_VARIABLE_NODE);
        }
    }
        
#if USE_MIND_SHADOW
    haxx::mindShadow.push_back(ret);
    haxx::mindShadowMap[T].push_back(ret);
#endif
LOG(3, "Add ok.");

/*  if (!tvn.isNullTv())
        if (Abs(a->getTV(ret)->getMean() - tvn.getMean()) > 0.0001)
        {
            printf("ATW: %s / %s\n", a->getTV(ret)->toString().c_str(),
                tvn.toString().c_str());
        }*/
    
//  assert(Abs(a->getTV(ret)->getMean() - tvn.getMean()) < 0.0001);

    return ret;
}

//int AtomTableWrapper::implicationConstruction()
//{
//  return 0;
/*  Btr<set<Handle> > links = getHandleSet(AND_LINK,"");
    foreach(Handle h, *links)
    {
    }*/
//}

Handle DirectATW::addLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh, bool managed)
{
    return directAddLink(T, hs, tvn, fresh,managed);
}

Handle DirectATW::addNode(Type T, const string& name, const TruthValue& tvn, bool fresh,bool managed)
{
    AtomSpace *a = AS_PTR;
    assert(!tvn.isNullTv());
    
    //assert(haxx::AllowFW_VARIABLENODESinCore || T != FW_VARIABLE_NODE);

    // Disable confidence for variables:
#ifdef USE_PSEUDOCORE
    //const TruthValue& mod_tvn = (!inheritsType(T, VARIABLE_NODE)?
    //  tvn : SimpleTruthValue(tvn.getMean(), 0.0f));
    const TruthValue& tv = SimpleTruthValue(tvn.getMean(), 0.0f); 
    const TruthValue& mod_tvn = (!inheritsType(T, VARIABLE_NODE))? tvn : tv;

    return a->addNode( T, name, mod_tvn, fresh,managed);
#else
    LOG(3,  "DirectATW::addNode");
    assert(1);
        
    if (inheritsType(T, FW_VARIABLE_NODE))
    {
        // Safeguard the identity of variables.
        map<string,Handle>::iterator existingHandle = haxx::variableShadowMap.find(name);
        if (existingHandle != haxx::variableShadowMap.end())
            return existingHandle->second;
    }   
    Node node( T,  name,  tvn);
    // TODO: add related context
    Handle ret = addAtomDC(node, fresh, managed);
    LOG(3, "Add ok.");
    
    if (inheritsType(T, FW_VARIABLE_NODE))
        haxx::variableShadowMap[name] = ret;
    
#if USE_MIND_SHADOW
haxx::mindShadow.push_back(ret);    
haxx::mindShadowMap[T].push_back(ret);
#endif
    return ret;
#endif
}

Handle AtomTableWrapper::addLinkDC(Type t, const HandleSeq& hs, const TruthValue& tvn,
        bool fresh, bool managed)
{
    AtomSpace *a = AS_PTR;
    HandleSeq hsReal;
    // Convert outgoing links to real Handles
    foreach(Handle h, hs) {
        hsReal.push_back(fakeToRealHandle(h).first);
    }
    // TODO: check that all vhpairs have the same Context... otherwise, create a
    // new one
    
    // Construct a Link then use addAtomDC
    Link l(t,hsReal);
    return addAtomDC(l, fresh, managed);
}

Handle AtomTableWrapper::addNodeDC(Type t, const string& name, const TruthValue& tvn,
        bool fresh, bool managed)
{
    AtomSpace *a = AS_PTR;
    // Construct a Node then use addAtomDC
    Node n(t, name, tvn);
    return addAtomDC(n, fresh, managed);
}

Handle AtomTableWrapper::addAtomDC(Atom &atom, bool fresh, bool managed)
{
    // ARI: what to do with managed? I can't find documentation in the Novamente
    // code, the header documentation just skips describing it.
    AtomSpace *a = AS_PTR;
    Handle result;
    Handle fakeHandle;
    // check if fresh true
    if (fresh) {
        // yes:
        // See if atom exists already
        HandleSeq outgoing;
        Node *nnn = dynamic_cast<Node *>((Atom *) & atom);
        if (nnn) {
            const Node& node = (const Node&) atom;
            result = getHandle(node.getType(), node.getName());
            if (TLB::isInvalidHandle(result)) {
                // if the atom doesn't exist already, then just add normally
                return realToFakeHandle(a->addNode(node.getType(),
                        node.getName(), node.getTruthValue()),
                        NULL_VERSION_HANDLE);
            }
        } else {
            const Link& link = (const Link&) atom;
            for (int i = 0; i < link.getArity(); i++) {
                outgoing.push_back(TLB::getHandle(link.getOutgoingAtom(i)));
            }
            result = getHandle(link.getType(), outgoing);
            if (TLB::isInvalidHandle(result)) {
                // if the atom doesn't exist already, then just add normally
                return realToFakeHandle(a->addLink(link.getType(),
                        outgoing, link.getTruthValue()),
                        NULL_VERSION_HANDLE);
            }
        }
        // if it does exist, then go through each dummy context until NULL_TV is
        // returned, when NULL_TV is returned, this is a free dummy context. if it
        // isn't returned before running out of dummy contexts, create a new dummy
        // context and use that.

        // result should contain a handle to existing atom
        set<VersionHandle>::iterator dc;
        VersionHandle vh;
        for (dc = dummyContexts.begin(); dc != dummyContexts.end(); dc++) {
            if (a->getTV(result,*dc).isNullTv()) {
                vh = *dc;
                break;
            }
        }
        if (dc == dummyContexts.end()) {
            // reached end of dummyContexts, so we need a new one
            stringstream dcName;
            dcName << contextPrefix << dummyContexts.size();
            // add context node
            Handle contextNode = a->addNode(CONCEPT_NODE,dcName.str());
            // add context link
            // it's possible we might get away without a context link but it
            // breaks the expected structure of the OpenCog graph. Adding the
            // context link to be safe.
            HandleSeq cOut;
            cOut.push_back(contextNode);
            cOut.push_back(result);
            a->addLink(CONTEXT_LINK,cOut);
            
            // add version handle to dummyContexts
            vh = VersionHandle(CONTEXTUAL,contextNode);
        } 
        // vh is now a version handle for a free context
        // for which we can set a truth value
        a->setTV(result, atom.getTruthValue(), vh);

        // Link <handle,vh> to a long int
        fakeHandle = realToFakeHandle(result,vh);
    } else {
        // no:
        // just add it and let AtomSpace deal with merging it
        result = a->addRealAtom(atom);
        // Link <handle, NULL_VERSION_HANDLE> with long int, long int is the
        // vector index.
        fakeHandle = realToFakeHandle(result, NULL_VERSION_HANDLE);
        
    }
    // return version handle instead so that the correct Handle can be retrieved
    // in future?
    return fakeHandle;

}

Handle FIMATW::addNode(Type T, const string& name, const TruthValue& tvn, bool fresh,bool managed)
{
    AtomSpace *a = AS_PTR;
/// The method should be, AFAIK, identical to the one in DirectATW, unless FIM is actually in use.
    
    assert(!tvn.isNullTv());
    
//  assert(haxx::AllowFW_VARIABLENODESinCore || T != FW_VARIABLE_NODE);

    /// Disable confidence for variables:

#ifdef USE_PSEUDOCORE
    const TruthValue& tv = SimpleTruthValue(tvn.getMean(), 0.0f);
    const TruthValue& mod_tvn = (!inheritsType(T, VARIABLE_NODE))? tvn : tv; 

    Handle ret = addNode( T, name, mod_tvn, fresh,managed);
#else
    
LOG(3,"FIMATW::addNode");

    if (inheritsType(T, FW_VARIABLE_NODE))
    {
        /// Safeguard the identity of variables.
    
        map<string,Handle>::iterator existingHandle = haxx::variableShadowMap.find(name);
        if (existingHandle != haxx::variableShadowMap.end())
            return existingHandle->second;
        
    }   

    Node node( T,  name,  tvn);
LOG(3,"FIMATW: addAtomDC"); 
    // fresh bug, done?
    Handle ret = addAtomDC(node,fresh,managed);

#if HANDLE_MANAGEMENT_HACK
LOG(3,"FIMATW: addAtomDC OK, checking for Handle release needed");      
    /// haxx:: // Due to core relic
    if (ret != node)
        delete node;
#endif
    
    LOG(3, "Add ok.");
    
#endif

    if (inheritsType(T, FW_VARIABLE_NODE))
        haxx::variableShadowMap[name] = ret;
    
#if USE_MIND_SHADOW
haxx::mindShadow.push_back(ret);    
haxx::mindShadowMap[T].push_back(ret);
#endif


/*  if (PLN_CONFIG_FIM)
    {
        unsigned int *pat1 = new unsigned int[PLN_CONFIG_PATTERN_LENGTH];
        myfim._copy(pat1, myfim.zeropat);
        pat1[0] = (unsigned int)ret;
        myfim.add(pat1);
    }*/

    return ret;
}

Handle FIMATW::addLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh,bool managed)
{
    Handle ret = directAddLink(T, hs, tvn, fresh,managed);  

#if 0
    if (PLN_CONFIG_FIM)
    {
        atom a(ret);
        unsigned int *pat1 = new unsigned int[PLN_CONFIG_PATTERN_LENGTH];
        try {
            a.asIntegerArray(pat1, PLN_CONFIG_PATTERN_LENGTH, node2pat_id, next_free_pat_id);
        } catch(string s) { LOG(4, s); return ret; }
        myfim.add((unsigned int)ret, pat1);
    }
#endif

    return ret; 

}


Handle AtomTableWrapper::getRandomHandle(Type T)
{
    AtomSpace *a = AS_PTR;
  vector<Handle> handles=a->filter_type(T);

  if (handles.size()==0)
    return Handle(0);

  return handles[rand()%handles.size()];
}

#if 0
unsigned int USize(const set<Handle>& triples, const set<Handle>& doubles,
                   const set<Handle>& singles)
{
    map<Handle, tuple3<Handle> > triple_contents;
    map<Handle, tuple2<Handle> > double_contents;
    
    int i=0;
    float total_n = 0.0f, nA, nB, nC, nAB, nBC, nAC, nABC;

    for (set<Handle>::const_iterator t = triples.begin(); t != triples.end(); t++)
    {
        nABC = TheNM.a->getCount(*t);

        vector<Handle> tc = a->getOutgoing(*t);
        assert(tc.size()==3);

        tuple2<Handle> ab, ac, bc;

        ab.t1 = tc[0];
        ab.t2 = tc[1];
        ac.t1 = tc[0];
        ac.t2 = tc[2];
        bc.t1 = tc[1];
        bc.t2 = tc[2];

        for (set<Handle>::const_iterator d = doubles.begin(); d != doubles.end(); d++)
        {
            vector<Handle> dc = a->getOutgoing(*d);
            assert(dc.size()==2);

            if (dc[0] == ab.t1 && dc[1] == ab.t2)
                nAB = TheNM.a->getCount(*d);
            else if (dc[0] == ac.t1 && dc[1] == ac.t2)
                nAC = TheNM.a->getCount(*d);
            else if (dc[1] == bc.t1 && dc[2] == bc.t2)
                nBC = TheNM.a->getCount(*d);
        }

        nA = TheNM.a->getCount(ab.t1);
        nB = TheNM.a->getCount(ab.t2);
        nC = TheNM.a->getCount(ac.t2);

        total_n += nB + (nA - nAB)*(nC - nBC) / (nAC - nABC);
    }

    return total_n / triples.size();
}
#endif


void AtomTableWrapper::DumpCoreLinks(int logLevel)
{
    AtomSpace *a = AS_PTR;
/*#ifdef USE_PSEUDOCORE
      PseudoCore::LinkMap LM = ((PseudoCore*)nm)->getLinkMap();

      for (PseudoCore::LinkMap::iterator i = LM.begin(); i != LM.end(); i++)
          printTree((Handle)i->first,0,logLevel);
#else
puts("Dump disabled");
#endif*/
  vector<Handle> LM=a->filter_type(LINK);
  for (vector<Handle>::iterator i = LM.begin(); i != LM.end(); i++)
    printTree(*i,0,logLevel);
}

void AtomTableWrapper::DumpCoreNodes(int logLevel)
{
    AtomSpace *a = AS_PTR;
/*#ifdef USE_PSEUDOCORE
      PseudoCore::NodeMap LM = ((PseudoCore*)nm)->getNodeMap();

      for (PseudoCore::NodeMap::iterator i = LM.begin(); i != LM.end(); i++)
          printTree((Handle)i->first,0,logLevel);
#else
puts("Dump disabled");
#endif*/
  vector<Handle> LM=a->filter_type(NODE);
  for (vector<Handle>::iterator i = LM.begin(); i != LM.end(); i++)
    printTree(*i,0,logLevel);
}

void AtomTableWrapper::DumpCore(Type T)
{
    shared_ptr<set<Handle> > fa = getHandleSet(T,"");
    //for_each<TableGather::iterator, handle_print<0> >(fa.begin(), fa.end(), handle_print<0>());
    for_each(fa->begin(), fa->end(), handle_print<0>());
}

Handle singular(HandleSeq hs) {
    assert(hs.size()<=1);
    return !hs.empty() ? hs[0] : NULL;
}

/*Handle list_atom(HandleSeq hs) {  
    return addLink(LIST_LINK, hs, TruthValue::TRUE_TV(),false);
}*/

bool AtomTableWrapper::equal(Handle A, Handle B)
{
    AtomSpace *a = AS_PTR;
    if (a->getType(A) != a->getType(B))
        return false;

    vector<Handle> hsA = a->getOutgoing(A);
    vector<Handle> hsB = a->getOutgoing(B);

    const size_t Asize = hsA.size();

    if (Asize != hsB.size())
        return false;

    for (unsigned int i = 0; i < Asize; i++)
        if (!equal(hsA[i], hsB[i]))
            return false;
    
    return true;
}

Handle AtomTableWrapper::OR2ANDLink(Handle& andL)
{
    return AND2ORLink(andL, OR_LINK, AND_LINK);
}

Handle AtomTableWrapper::AND2ORLink(Handle& andL)
{
    return AND2ORLink(andL, AND_LINK, OR_LINK);
}

Handle AtomTableWrapper::invert(Handle h)
{
    HandleSeq hs;
    hs.push_back(h);
    return addLink(NOT_LINK, hs, TruthValue::TRUE_TV(), true);
}

Handle AtomTableWrapper::AND2ORLink(Handle& andL, Type _ANDLinkType, Type _ORLinkType)
{
    AtomSpace *a = AS_PTR;
    assert(a->getType(andL) == _ANDLinkType);

    HandleSeq ORtarget;
    const vector<Handle> _ANDtargets = a->getOutgoing(andL);

    for (vector<Handle>::const_iterator i = _ANDtargets.begin();
            i != _ANDtargets.end(); i++) {
        ORtarget.push_back(invert(*i));
    }

    const TruthValue& outerTV = getTV(andL);

    HandleSeq NOTarg;
    Handle newORAND = addLink(_ORLinkType, ORtarget,
                outerTV,
                true);

    NOTarg.push_back(newORAND);

puts("---------");
printTree(newORAND,0,0);

    return addLink(NOT_LINK, NOTarg,
                TruthValue::TRUE_TV(),
                true);
}

pair<Handle, Handle> AtomTableWrapper::Equi2ImpLink(Handle& exL)
{
    AtomSpace *a = AS_PTR;
    printf("((%d))\n", a->getType(exL));
    printTree(exL,0,0);

    assert(a->getType(exL) == EXTENSIONAL_EQUIVALENCE_LINK);

    HandleSeq ImpTarget1, ImpTarget2;

    const vector<Handle> EquiTarget = a->getOutgoing(exL);

    for (vector<Handle>::const_iterator i = EquiTarget.begin(); i != EquiTarget.end(); i++)
        ImpTarget1.push_back((*i));

    assert(ImpTarget1.size()==2);

    for (vector<Handle>::const_reverse_iterator j = EquiTarget.rbegin(); j != EquiTarget.rend(); j++)
        ImpTarget2.push_back((*j));

    assert(ImpTarget2.size()==2);

    const TruthValue& outerTV = getTV(exL);

    pair<Handle, Handle> ret;

    ret.first = addLink(IMPLICATION_LINK, ImpTarget1,
                outerTV,
                true);

    ret.second = addLink(IMPLICATION_LINK, ImpTarget2,
                outerTV,
                true);

    return ret;
}

bool AtomTableWrapper::isReal(const Handle h) const
{
    return AS_PTR->isReal(fakeToRealHandle(h).first);
}

const TimeServer& AtomTableWrapper::getTimeServer() const
{
    return AS_PTR->getTimeServer();
}

int AtomTableWrapper::getArity(Handle h) const
{
    // get neighbours
    vhpair v = fakeToRealHandle(h);
    // check neighbours have TV with same VersionHandle
    HandleSeq hs = AS_PTR->getOutgoing(v.first);
    int arity = 0;
    foreach (Handle h, hs) {
        if (!AS_PTR->getTV(v.first,v.second).isNullTv()) {
            arity += 1;
        }
    }
    return arity;

}

HandleSeq AtomTableWrapper::filter_type(Type t) 
{
    HandleSeq s;
    s = AS_PTR->filter_type(t);
    return realToFakeHandle(s, true);


}

std::vector<Handle> AtomTableWrapper::getOutgoing(const Handle h)
{
    vhpair v = fakeToRealHandle(h);
    HandleSeq s1,s2;
    s1 = AS_PTR->getOutgoing(v.first);
    foreach (Handle h1, s1) {
        if (!AS_PTR->getTV(h1,v.second).isNullTv()) {
            s2.push_back(realToFakeHandle(h1,v.second));
        }
    }
    return s2;

}

Handle AtomTableWrapper::getOutgoingAtIndex(const Handle h, const int i) 
{
    return child(h, i);
}

Type AtomTableWrapper::getType(const Handle h) const
{
    return AS_PTR->getType(fakeToRealHandle(h).first);
}

std::string AtomTableWrapper::getName(const Handle h) const
{
    return AS_PTR->getName(fakeToRealHandle(h).first);
}

Type AtomTableWrapper::getTypeV(const tree<Vertex>& _target) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);
    return getType(v2h(*_target.begin()));
}

/*
Handle AtomTableWrapper::Exist2ForAllLink(Handle& exL)
{
    assert(a->getType(exL) == EXIST_LINK);

    HandleSeq ForAllTarget;

    const vector<Handle> ExistTarget = a->getOutgoing(exL);

    for (vector<Handle>::const_iterator i = ExistTarget.begin(); i != ExistTarget.end(); i++)
    {
        ForAllTarget.push_back(invert(*i));
    }

    const TruthValue& outerTV = getTV(exL);

    HandleSeq NOTarg;
    NOTarg.push_back(addLink(FORALL_LINK, ForAllTarget,
                outerTV,
                true));

    return addLink(NOT_LINK, NOTarg,
                TruthValue::TRUE_TV(),
                true);
}
*/
} //~namespace

