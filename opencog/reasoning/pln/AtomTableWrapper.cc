#include "AtomTableWrapper.h"
#include "PLN.h"
#include "utils/XMLNodeLoader.h"
#include "rules/Rules.h"

#include <SimpleTruthValue.h>
#include <CogServer.h> // To get access of AtomSpace
#include <tree.h>
#include <utils2.h>
#include <utils.h>

#if 1 //000

#include  <boost/foreach.hpp>
boost::variant<int, char> bv;

#define USE_MIND_SHADOW 0
#define HANDLE_MANAGEMENT_HACK 0
#define ARCHIVE_THEOREMS 1

#define AS_PTR (CogServer::getAtomSpace())

using namespace std;
using namespace opencog;

bool isSubType(Handle h, Type T)
{
	return inheritsType(AS_PTR->getType(h), T);
}

#ifndef USE_PSEUDOCORE
	void HandleEntry2HandleSeq(HandleEntry& src, vector<Handle>& dest)
	{	
		int array_size=0;
		Handle *dest_array = new Handle[1000];
	
		src.toHandleVector(dest_array, array_size);
		for (int i =0;i<array_size;i++)
		{
			dest.push_back(dest_array[i]);
		}

		delete dest_array;
	}
#endif
	
Handle child(Handle h, int i) { 
	return AS_PTR->getOutgoing(h, i);
}
	
namespace haxx
{
	reasoning::iAtomTableWrapper* defaultAtomTableWrapper;
	set<Handle> atomRemoveSet;
	multimap<Handle,Handle> childOf;
	bool AllowFW_VARIABLENODESinCore = true;
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
/// \todo haxx:: Re-enable cache. It must simply be updated so that (pseudo)core reset takes it into account.
/*    map<Handle,vtree>::iterator i = h2vtree_cache.find(h);
       if (i != h2vtree_cache.end())
               return i->second;*/
       
   vtree ret;
   reasoning::makeHandletree(h, true, ret);
/*    h2vtree_cache[h] = ret;*/

   reasoning::printTree(h,0,0);
   rawPrint(ret, ret.begin(), 0);

   return ret;
}

#ifdef USE_PSEUDOCORE

float ContradictionLimit = 0.1f; //Below the limit, we just revise the differences.

bool inheritsType(Type subT, Type superT) 
{
	return ClassServer::isAssignableFrom(superT, subT);
}

const TruthValue& getTruthValue(Handle h)
{
	return AS_PTR->getTV(h);
}

namespace reasoning
{
	shared_ptr<set<Handle> > AtomTableWrapper::getHandleSet(Type T, const string& name, bool subclass) const
	{
		vector<Handle> hs(AS_PTR->getHandleSet(T,name,subclass));
		return shared_ptr<set<Handle> >(new set<Handle>(hs.begin(), hs.end()));
	}

Handle AtomTableWrapper::getHandle(Type t,const string& str) const
{
	AtomSpace *a = AS_PTR;
	return AS_PTR->getHandle(t, str);
}

Handle AtomTableWrapper::getHandle(Type t,const HandleSeq& outgoing) const
{
	return AS_PTR->getHandle(t, outgoing);
}

}; //namespace reasoning

#else

	void HandleEntry2HandleSet(HandleEntry& src, set<Handle>& dest)
	{	
		std::vector<Handle> dest_array;
	
		src.toHandleVector(dest_array);
		for (unsigned int i =0;i<dest_array.size();i++)
		{
			dest.insert(dest_array[i]);
		}
	}

bool inheritsType(Type T1, Type T2)
{
	return ClassServer::isAssignableFrom(T2, T1);
}

const TruthValue& getTruthValue(Handle h)
{
    AtomSpace* a = AS_PTR;
	return (h ? (a->getTV(h)) : TruthValue::TRIVIAL_TV());
}

namespace reasoning
{
	shared_ptr<set<Handle> > AtomTableWrapper::getHandleSet(Type T, const string& name, bool subclass) const
	{
		HandleEntry* result = 
			(name.empty()
				? AS_PTR->getAtomTable().getHandleSet((Type) T, subclass)
				: AS_PTR->getAtomTable().getHandleSet(name.c_str(), (Type) T, subclass));
		shared_ptr<set<Handle> > ret(new set<Handle>);

		HandleEntry2HandleSet(*result, *ret);

		delete result;
		return ret;
	}

	Handle AtomTableWrapper::getHandle(Type t,const string& name) const
	{
		return AS_PTR->getAtomTable().getHandle(name.c_str(), (Type)t);
	}
	
	bool equal(const HandleSeq& lhs, const HandleSeq& rhs)
	{
		size_t lhs_arity = lhs.size();
		if (lhs_arity != rhs.size())
			return false;
			
		for (unsigned int i = 0; i < lhs_arity; i++)
			if (lhs[i] != rhs[i])
				return false;
		return true;			
	}

	Handle AtomTableWrapper::getHandle(Type t,const HandleSeq& outgoing) const
	{
		//HandleEntry* results = 
		//	AS_PTR->getHandleSet((Type)t, true);
			
//		while (results && !equal(results->handle->getOutgoingSet(), outgoing))
//			results = results->next;
//      Handle ret = (results ? results->handle : NULL);
//		delete results;
		
		return AS_PTR->getHandle(t,outgoing);
	}

    void AtomTableWrapper::reset()
    {
        AS_PTR->clear();
    }
}

#endif

namespace reasoning
{
extern int atom_alloc_count;
void initArbitraryAtoms();
void printUtable();
void SetTheoremTest();
void existence2universality();
extern bool linkNotifications;

static Handle U;

bool AtomTableWrapper::Prepare()
{
	srand(12345678);
	linkNotifications = true;

	return true;
}

bool AtomTableWrapper::LoadAxioms(const string& path)
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
		LoadedFiles.insert(fname);
    } catch(string s) { 
        LOG(0, s); 
        return false; 
    } catch(...) { 
        LOG(0, "UNKNOWN EXCEPTION IN LOADAXIOMS!!!"); 
        return false; 
	}

	return true;
}

bool AtomTableWrapper::LoadOther(const string& path, bool ReplaceOld)
{
	string buf;
	LoadTextFile(path, buf);

	vector<string> lines = StringTokenizer(buf, "\n\r").WithoutEmpty();

//	SetUniverseSize(CONST_SIZE, lines.size());

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

	LoadedFiles.insert(path);

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
	//Handle ret = FIMATW::addNode(T, name, tvn, fresh);

	return ret;
}

int getFirstIndexOfType(HandleSeq hs, Type T)
{
	AtomSpace *a = AS_PTR;
	for (unsigned int i = 0; i < hs.size(); i++)
		if (a->getType(hs[i]) == T)
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
/*				if (TheHandleWrapperFactory.New(*jj)->equalOLD(
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

/*	for (i = 0; i < hs.size(); i++)
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

bool linkNotifications=false;

bool AtomTableWrapper::binary_true(Handle h)
{
	const TruthValue& tv = getTruthValue(h);

	return (tv.getMean() > 0.989);
}

bool symmetricLink(Type T)
{
	return inheritsType(T, AND_LINK) || inheritsType(T, LIST_LINK)
			|| inheritsType(T, OR_LINK);
}

bool is_empty_link(Handle h)
{
	AtomSpace *a = AS_PTR;
	return !inheritsType(a->getType(h), NODE)
			&& a->getArity(h) == 0;
}

bool hasFalsum(HandleSeq hs)
{
	AtomSpace *a = AS_PTR;
//	return false;

	for (vector<Handle>::const_iterator ii = hs.begin(); ii != hs.end(); ii++)
	{
		const Handle key = *ii;

		if (inheritsType(a->getType(*ii), FALSE_LINK) ) //Explicit falsum
			return true;

		for (vector<Handle>::const_iterator jj = hs.begin(); jj != hs.end();jj++)
			if (jj != ii)
				if (inheritsType(a->getType(*jj), NOT_LINK) ) //Contradiction
				{
					Handle notter = a->getOutgoing(*jj)[0];
					if (notter == key)
						return true;
				}
	}

	return false;
}

bool containsNegation(Handle ANDlink, Handle h)
{
	AtomSpace *a =  AS_PTR;
	HandleSeq hs = a->getOutgoing(ANDlink);

	hs.push_back(h);

	return hasFalsum(hs);
}

Handle AtomTableWrapper::freshened(Handle h, bool managed)
{
	AtomSpace *a = AS_PTR;
	Type T = a->getType(h);
	HandleSeq hs = a->getOutgoing(h);
	string name = a->getName(h);
	const TruthValue& tv = getTruthValue(h);

	if (inheritsType(T, NODE))
		return addNode(T, name, tv, true,managed);
	else
	{
		for (unsigned int i = 0; i < hs.size(); i++)
			hs[i] = freshened(hs[i], managed);

		return addLink(T, hs, tv, true, managed);
	}
}

#define BL 2

Handle NormalizingATW::addLink(Type T, const HandleSeq& hs, const TruthValue& tvn,bool fresh,bool managed)
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
/*		if (TheLog.getLevel()>=5)
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
	else if (T == IMPLICATION_LINK								//Accidentally similar to da above
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

//		LOG(BL, "=>( =>(A,B), C) ) into =>( |(B,~A),C ) into  =>( ~&(~B,A),C ) ");
		LOG(BL, "=>( =>(A,B), C) ) into &( =>( A&B, C), =>(~A, C) )");

		HandleSeq NOTa_args;
		NOTa_args.push_back(a);
		
		HandleSeq AND_args;
		AND_args.push_back(a);
		AND_args.push_back(b);

//		HandleSeq NOTAND_args;

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
/*	else if (T == AND_LINK
			&& hs.size()==2
			&& a->getType(hs[1]) != IMPLICATION_LINK)
	{
		LOG(0, "AND => Implication");
		cprintf(0,"%d\n",nm->getType(hs[1]));
		getc(stdin);
		
		TruthValue **tvs = new TruthValue *[3];
		tvs[0] = &tvn;
		tvs[1] = getTruthValue(hs[0]);
		tvs[2] = getTruthValue(hs[1]);
		
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
			&& binary_true(hs[0]) )
	{
		LOG(BL, "~~A <---> A");

		HandleSeq relevant_args = a->getOutgoing(hs[0]);
		assert(relevant_args.size() == 1);
		Type relevant_type = a->getType(relevant_args[0]);

		//if (tvn.getMean() > 0.989 && binary_true(relevant_args[0]) )
		ret = a->getOutgoing(hs[0])[0]; //addLink(relevant_type, relevant_args,TruthValue::TRUE_TV(),fresh);
	}
	else if (T == AND_LINK
			&& !hs.empty()
			&& tvn.getMean() > 0.989
			&& getFirstIndexOfType(hs, AND_LINK) >= 0
			&& binary_true(hs[getFirstIndexOfType(hs, AND_LINK)]) )
			//&& inheritsType(a->getType(hs[0]), AND_LINK))
	{
		LOG(BL, "AND(AND(B), AND(A)) <---> AND(B,A)");

//		bool all_true_and = true;

		HandleSeq new_args;

		for (int ii = 0; ii < hs.size(); ii++)
		{
//			printTree(hs[ii],0,4);

			if (inheritsType(a->getType(hs[ii]), AND_LINK)
				&& binary_true(hs[ii]))
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
//	printTree(hs[a],0,4);

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
/*	else if (T == IMPLICATION_LINK
			&& hs.size()==2
			&& tvn.getMean() > 0.989
			&& binary_true(hs[1])
			&& inheritsType(a->getType(hs[1]), IMPLICATION_LINK))
	{
		LOG(0, "(A=>(B=>C))	--->	((A&B) => C)");
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

//		printTree(ret);
	}*/

#if 0
	else if (T == IMPLICATION_LINK
			&& !hs.empty()
			&& inheritsType(a->getType(hs[0]), AND_LINK)
			&& getFirstIndexOfType(a->getOutgoing(hs[0]), IMPLICATION_LINK) >= 0)
	{
		LOG(1, "Cut (A& (B=>C) ) => D	--->	(C & A) => D   &   (~B & A)=>D");

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
		const vector<Handle> EquiTarget = hs;
		HandleSeq ImpTarget1, ImpTarget2;
		
		for (vector<Handle>::const_iterator i = EquiTarget.begin(); i != EquiTarget.end(); i++)
			ImpTarget1.push_back((*i));
int zz=ImpTarget1.size();
		assert(ImpTarget1.size()==2);
		
		for (vector<Handle>::const_reverse_iterator j = EquiTarget.rbegin(); j != EquiTarget.rend(); j++)
			ImpTarget2.push_back((*j));
		
		assert(ImpTarget2.size()==2);
		
//		const TruthValue& outerTV = getTruthValue(exL);
		
		HandleSeq ANDargs;

		ANDargs.push_back( addLink(IMPLICATION_LINK, ImpTarget1,
			tvn,
			true,managed));

		ANDargs.push_back( addLink(IMPLICATION_LINK, ImpTarget2,
			tvn,
			true,managed));

//		reverse(ANDargs.begin(), ANDargs.end());

		ret = addLink(AND_LINK, ANDargs, TruthValue::TRUE_TV(), fresh,managed);
	}
	else if (T == FORALL_LINK
			&& hs.size() == 2
			&& inheritsType(a->getType(hs[1]), AND_LINK)
			&& binary_true(hs[1])
			&& a->getArity(hs[1]) > 1)
	{
		unsigned int AND_arity = a->getArity(hs[1]);

		HandleSeq fa_list;

		for (unsigned int i = 0; i < AND_arity; i++)
		{
			HandleSeq fora_hs;
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
//			&& tvn.getMean() > 0.989)
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
/*	else if (T==FORALL_LINK && hs.size()==2)
	{
ok_forall=true;
	}*/

/*	else if (AddToU
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
//		if (symmetricLink(T))
//			remove_if(hs.begin(), hs.end(), is_empty_link);

/*		if (T==AND_LINK)
		{

			int original_size = hs.size();
			remove_redundant(hs);
			assert (hs.size() == original_size);
		}*/

		LOG(5, "Adding to Core...");

        // TODO: OpenCog port: remove FIM
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
		for (	HandleEntry* e = a->getHandleSet((Type)i, true);
				e && e->handle;
				e = e->next)
		{
		printf(".\n");
			mindShadowMap[i]->insert(e->handle);
		}
			
//		delete e;
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
			&&	((Node*)(*i))->getName() == name)
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
			
//			set<Handle>::iterator remo = mindShadowMap[T]->begin();
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

Handle AtomTableWrapper::addAtom(tree<Vertex>& a, const TruthValue& tvn, bool fresh, bool managed)
{
	return addAtom(a,a.begin(),tvn,fresh,managed);
}

Handle AtomTableWrapper::addAtom(tree<Vertex>& a, tree<Vertex>::iterator it, const TruthValue& tvn, bool fresh, bool managed)
{
	AtomSpace *as = AS_PTR;
	cprintf(3,"Handle AtomTableWrapper::addAtom...");
	rawPrint(a,it,3);
	
	vector<Handle> handles;
	Handle head_type = boost::get<Handle>(*it);
	
	//assert(haxx::AllowFW_VARIABLENODESinCore || head_type != (Handle)FW_VARIABLE_NODE);
	
	if (as->isReal(head_type))
	{
		LOG(1, "Warning! Adding a real atom with addAtom(tree<Vertex>& a)!\n");
		return head_type;
	}

	for (tree<Vertex>::sibling_iterator i = a.begin(it); i!=a.end(it); i++)
	{
		Handle *h_ptr = boost::get<Handle>(&*i);

		handles.push_back((h_ptr && as->isReal(*h_ptr))
								? (*h_ptr)
								: addAtom(a, i, TruthValue::TRIVIAL_TV(), false, managed));
	}

/*	/// We cannot add non-existent nodes this way!
	assert (!inheritsType(head_type, NODE));*/
	
	return addLink((Type)(int)head_type, handles, tvn, fresh,managed);
}

Handle directAddLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh,bool managed)
{
	AtomSpace *a = AS_PTR;
	if (tvn.isNullTv())
	{
		LOG(0, "I don't like FactoryTruthValues, so passin NULL as TruthValue causes exit in AtomTableWrapper.cc.");
		exit(0);
	}

LOG(3, "Directly adding...");
assert(1);

	uint arity = hs.size();
	
    if (T == INHERITANCE_LINK && arity==2)
        haxx::childOf.insert(pair<Handle,Handle>(hs[1], hs[0]));

	Handle ret;

	if (haxx::ArchiveTheorems &&
		T == IMPLICATION_LINK && a->getType(hs[0]) == AND_LINK && tvn.getConfidence() > 0.98999f)
		{
			vector<Handle> args = a->getOutgoing(hs[0]);
			printf("THM for:");

			vtree thm_target(make_vtree(hs[1]));

			rawPrint(thm_target, thm_target.begin(), 3);
			LOG(0,"Takes:");
			
			foreach(Handle arg, args)
			{
				vtree arg_tree(make_vtree(arg));
				rawPrint(arg_tree, arg_tree.begin(), 0);
				CrispTheoremRule::thms[thm_target].push_back(arg_tree);
			}
			
			///warning "Return value will be invalid!"
            // TODO: fresh bug
			ret = a->addLink( FALSE_LINK, hs, tvn);
		}
	else	
        // TODO: fresh bug
		ret = a->addLink( T, hs,  tvn);

	if (inheritsType(T, LINK) && !arity && T != FORALL_LINK)
	{
		printTree(ret,0,1);
		cprintf(1,"inheritsType(T, LINK) && !arity\n");
	}	
	
	if (!haxx::AllowFW_VARIABLENODESinCore)
		foreach(Handle ch, hs)
		{
			assert(a->isReal(ch));
			if (a->getType(ch) == FW_VARIABLE_NODE)
			{
				printTree(ret,0,-10);
				cprintf(-10,"ATW: a->getType(ch) == FW_VARIABLE_NODE!");
				getc(stdin);getc(stdin);
			}
//				throw string("a->getType(ch) == FW_VARIABLE_NODE") + i2str(ret);
			//assert(a->getType(ch) != FW_VARIABLE_NODE);
		}
		
#if USE_MIND_SHADOW
	haxx::mindShadow.push_back(ret);
	haxx::mindShadowMap[T].push_back(ret);
#endif
LOG(3, "Add ok.");

/*	if (!tvn.isNullTv())
		if (Abs(a->getTruthValue(ret)->getMean() - tvn.getMean()) > 0.0001)
		{
			printf("ATW: %s / %s\n", a->getTruthValue(ret)->toString().c_str(),
				tvn.toString().c_str());
		}*/
	
//	assert(Abs(a->getTruthValue(ret)->getMean() - tvn.getMean()) < 0.0001);

	return ret;
}

int AtomTableWrapper::ImplicationConstruction()
{
	return 0;
/*	Btr<set<Handle> > links = getHandleSet(AND_LINK,"");
	foreach(Handle h, *links)
	{
	}*/
}

Handle DirectATW::addLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh, bool managed)
{
	return directAddLink(T, hs, tvn, fresh,managed);
}

Handle DirectATW::addNode(Type T, const string& name, const TruthValue& tvn, bool fresh,bool managed)
{
	AtomSpace *a = AS_PTR;
	assert(!tvn.isNullTv());
	
//	assert(haxx::AllowFW_VARIABLENODESinCore || T != FW_VARIABLE_NODE);

	/// Disable confidence for variables:

#ifdef USE_PSEUDOCORE
	//const TruthValue& mod_tvn = (!inheritsType(T, VARIABLE_NODE)? tvn : SimpleTruthValue(tvn.getMean(), 0.0f));
        const TruthValue& tv = SimpleTruthValue(tvn.getMean(), 0.0f); 
	const TruthValue& mod_tvn = (!inheritsType(T, VARIABLE_NODE))? tvn : tv;

	return a->addNode( T, name, mod_tvn, fresh,managed);
#else
LOG(3,	"DirectATW::addNode");
assert(1);
		
	if (inheritsType(T, FW_VARIABLE_NODE))
	{
		/// Safeguard the identity of variables.
	
		map<string,Handle>::iterator existingHandle = haxx::variableShadowMap.find(name);
		if (existingHandle != haxx::variableShadowMap.end())
			return existingHandle->second;
		
	}	
	Node* node = new Node( T,  name,  tvn);
    //TODO: fresh = true bug
	//Handle ret = MindDBProxy::getInstance()->add(node, fresh);
	Handle ret = a->addRealAtom(*node);
	
#if HANDLE_MANAGEMENT_HACK	
	/// haxx:: // Due to core relic
	if (ret != node)
		delete node;
#endif	
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

Handle FIMATW::addNode(Type T, const string& name, const TruthValue& tvn, bool fresh,bool managed)
{
	AtomSpace *a = AS_PTR;
/// The method should be, AFAIK, identical to the one in DirectATW, unless FIM is actually in use.
	
	assert(!tvn.isNullTv());
	
//	assert(haxx::AllowFW_VARIABLENODESinCore || T != FW_VARIABLE_NODE);

	/// Disable confidence for variables:

#ifdef USE_PSEUDOCORE
    const TruthValue& tv = SimpleTruthValue(tvn.getMean(), 0.0f);
    const TruthValue& mod_tvn = (!inheritsType(T, VARIABLE_NODE))? tvn : tv; 

	Handle ret = a->addNode( T, name, mod_tvn, fresh,managed);
#else
	
LOG(3,"FIMATW::addNode");

	if (inheritsType(T, FW_VARIABLE_NODE))
	{
		/// Safeguard the identity of variables.
	
		map<string,Handle>::iterator existingHandle = haxx::variableShadowMap.find(name);
		if (existingHandle != haxx::variableShadowMap.end())
			return existingHandle->second;
		
	}	

	Node* node = new Node( T,  name,  tvn);
LOG(3,"FIMATW: AtomSpace->add");	
    // TODO: fresh bug
    Handle ret = AS_PTR->addRealAtom(*node);

#if HANDLE_MANAGEMENT_HACK
LOG(3,"FIMATW: AtomSpace->add OK, checking for Handle release needed");		
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


/*	if (PLN_CONFIG_FIM)
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


Handle AtomTableWrapper::GetRandomHandle(Type T)
{
	AtomSpace *a = AS_PTR;
/*#ifdef USE_PSEUDOCORE	
	int choose_node = 0;
	
	if (inheritsType(T, NODE))
		choose_node = 1;
	else if (inheritsType(T, LINK))
		choose_node = 0;
	else
		choose_node = (rand()%2);

	PseudoCore* pcore = (PseudoCore*)(nm);

	if (!pcore)
		throw string("PSEUDOCORE REQUIRED");
	
	int i=0;
	
	if (choose_node)
	{
		int index = rand()%pcore->_nodes.size();
		
		for (PseudoCore::NodeMap ::const_iterator it=pcore->_nodes.begin();it!=pcore->_nodes.end();++it,i++)
			if (i >= index)
				if (pcore->inheritsType(it->second.type, T))
					return (Handle)it->first;
				
				return (Handle)(pcore->_nodes.begin()->first); //If no good...
	}
	else
	{
		int index = rand()%pcore->_links.size();
		
		for (PseudoCore::LinkMap::const_iterator it=pcore->_links.begin();it!=pcore->_links.end();++it,i++)
			if (i >= index)
				if (pcore->inheritsType(it->second.type, T))
					return (Handle)it->first;
				
				return (Handle)(pcore->_links.begin()->first); //If no good...
	}
#else
	
	#if USE_MIND_SHADOW
	return haxx::mindShadow[random() % haxx::mindShadow.size()];
	#else
	puts("Must enable USE_MIND_SHADOW!");
	#endif

#endif*/

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

bool equal(Handle A, Handle B)
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

Handle AtomTableWrapper::OR2ANDLink(Handle& andL) { return AND2ORLink(andL, OR_LINK, AND_LINK); }
Handle AtomTableWrapper::AND2ORLink(Handle& andL) { return AND2ORLink(andL, AND_LINK, OR_LINK); }

Handle AtomTableWrapper::Invert(Handle h)
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

	for (vector<Handle>::const_iterator i = _ANDtargets.begin(); i != _ANDtargets.end(); i++)
	{
		ORtarget.push_back(Invert(*i));
	}

	const TruthValue& outerTV = getTruthValue(andL);

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

	const TruthValue& outerTV = getTruthValue(exL);

	pair<Handle, Handle> ret;

	ret.first = addLink(IMPLICATION_LINK, ImpTarget1,
				outerTV,
				true);

	ret.second = addLink(IMPLICATION_LINK, ImpTarget2,
                outerTV,
				true);

	return ret;
}
/*
Handle AtomTableWrapper::Exist2ForAllLink(Handle& exL)
{
	assert(a->getType(exL) == EXIST_LINK);

	HandleSeq ForAllTarget;

   	const vector<Handle> ExistTarget = a->getOutgoing(exL);

	for (vector<Handle>::const_iterator i = ExistTarget.begin(); i != ExistTarget.end(); i++)
	{
		ForAllTarget.push_back(Invert(*i));
	}

	const TruthValue& outerTV = getTruthValue(exL);

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

#endif //000
