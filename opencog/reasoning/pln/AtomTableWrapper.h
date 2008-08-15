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
enum USizeMode { CONST_SIZE, REAL_SIZE };

tree<Vertex> make_vtree(Handle h);

/** @class AtomTableWrapper
	\brief The bridge between atom tables (PseudoCore) and PTL.
*/

class AtomTableWrapper : public iAtomTableWrapper
{
	uint USize;
	USizeMode Usizemode;
	Handle addAtom(tree<Vertex>&, tree<Vertex>::iterator, const TruthValue& tvn, bool fresh, bool managed);
public:
	set<std::string> LoadedFiles;
	
	void DumpCoreNodes(int logLevel);
	void DumpCoreLinks(int logLevel);
	void DumpCore(Type T);

	unsigned int GetUniverseSize() const
	{
		return USize;
	}
	void SetUniverseSize(USizeMode _Usizemode, unsigned int _USize)
	{
		assert(_Usizemode == CONST_SIZE);
		Usizemode = _Usizemode;
		USize = _USize;
	}

	virtual Btr<set<Handle> > getHandleSet(Type T, const string& name, bool subclass = false) const;
	Handle getHandle(Type t,const std::string& str) const;
    Handle getHandle(Type t,const HandleSeq& outgoing) const;

    void reset();

	AtomTableWrapper() : USize(800) {}
	virtual ~AtomTableWrapper() {}
//	combo::NMCore* getCore() const { return core; }

	bool LoadAxioms(const string& path);
	bool LoadOther(const std::string& path, bool ReplaceOld);
//	std::vector<atom> LoadAnds(const std::string& path);

	/// Makes sure that the loaded stuff is in a correct normal form etc.
	bool Prepare();
		
	int ImplicationConstruction();

	Handle addAtom(tree<Vertex>&, const TruthValue& tvn, bool fresh, bool managed);
	virtual Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh, bool managed=true)=0;
	virtual Handle addNode(Type T, const std::string& name, const TruthValue& tvn, bool fresh, bool managed=true)=0;

    bool HasFalsum(float minAllowedError = 0.5)
	{
		return false;
	}

	Handle GetRandomHandle(Type T);
	Handle freshened(Handle h, bool managed);
	bool binary_true(Handle h);

	Handle Invert(Handle h);
	Handle AND2ORLink(Handle& andL, Type _ANDLinkType, Type _ORLinkType);
	pair<Handle, Handle> Equi2ImpLink(Handle& exL);
	Handle Exist2ForAllLink(Handle& exL);
	Handle OR2ANDLink(Handle& andL);
	Handle AND2ORLink(Handle& andL);
	
/*	void VariableMPforms(const atom& src, set<atom, lessatom_ignoreVarNameDifferences>& res,
					 set<subst>* forbiddenBindings);				 */
};

/** @class FIMATW
	Passes the atoms via FIM analyzer. To turn this off, set FIM=0 in Config.
*/

class FIMATW : public AtomTableWrapper
{
//	std::map<atom,int,lessatom> node2pat_id;
	fim::pat_id next_free_pat_id;
public:
	/// Semi-haxx::
	fim::grim myfim;

//	FIMATW(combo::NMCore* core) : AtomTableWrapper(core), next_free_pat_id(30001) {}
	FIMATW() : next_free_pat_id(30001) {}
	virtual ~FIMATW() {}

	Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh, bool managed=true);
	Handle addNode(Type T, const std::string& name, const TruthValue& tvn, bool fresh, bool managed=true);
};

/** @class NormalizingATW
	Normalizes the atom before passing forward
*/

class NormalizingATW : public FIMATW
{
    NormalizingATW();
public:
	virtual ~NormalizingATW() {}
       
    static NormalizingATW& getInstance() {
        static NormalizingATW* instance = new NormalizingATW();
		return *instance;
    }
    
	Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh, bool managed=true);
	Handle addNode(Type T, const std::string& name, const TruthValue& tvn, bool fresh, bool managed=true);
	Btr<set<Handle> > getHandleSet(Type,const string&,bool = false);

};

/** @class DirectATW
	Forwards the requests without normalizing */

class DirectATW : public AtomTableWrapper
{
	DirectATW();
public:
	virtual ~DirectATW() { }

    static DirectATW& getInstance() {
        static DirectATW* instance = new DirectATW();
		return *instance;
    }
    
	Handle addLink(Type T, const HandleSeq& hs, const TruthValue& tvn, bool fresh, bool managed=true);
	Handle addNode(Type T, const std::string& name, const TruthValue& tvn, bool fresh, bool managed=true);
	Btr<set<Handle> > getHandleSet(Type, const string&,bool = false);

};

using namespace boost;

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
};


} //~namespace reasoning

#endif
