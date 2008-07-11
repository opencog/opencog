#ifndef PTLATOM_H
#define PTLATOM_H

#define MetaPredicate atom

// TODO: PTL-specific types. They should be declared at src/common/type.script, 
// which is a script for creating automatically (and so, in a more safer way) 
// integer codes and string representation of all NM atom types 

#define RESTRICTOR			        158
#define INSTANCEOF_R				159
#define HAS_R			                160
#define IN_R			                161

#define TYPED			                162

#define __EQUALS				163
#define __EQUALS_N				164
#define __INSTANCEOF_N				165
#define __INDEXER				166
#define __INDEX1				167
#define __INDEX2				168
#define __INDEX3				169
#define __INDEX4				170

#define __OR					171
#define __AND					172
#define __NOT					173

//#include "PLNUtils.h"
//#include "Rule.h" 
#include "iAtomTableWrapper.h"
#include <boost/smart_ptr.hpp>

namespace reasoning
{
struct atom;
typedef pair<std::string, atom> subst;
struct lessatom;

struct less_subst;

struct atom //: public BoundHandle
{
public:
	mutable Handle real;
	mutable std::map<std::string,atom>* bindings;

	Type T;
	int arity;
	std::string name;
	Handle cx;

	std::vector<boost::shared_ptr<atom> > hs;
	
	mutable set<subst>* forbiddenBindings;

	bool operator<(const atom& rhs) const;
	bool operator!=(const atom& rhs) const { return (*this)<rhs || rhs<(*this); }

	void setHandle(Handle h);	

	explicit atom(Handle h);
	atom();
	atom(const atom& rhs);
	atom(Type _T, std::string _name);
	atom(Type _T, int _arity, ...);
	atom(Type _T, std::vector<boost::shared_ptr<atom> > hs);

	/** Create from an integer pattern of a pre-defined length. Doesn't change the ownership
		of the array.
	*/

	//atom(unsigned int*);
	~atom();

	bool well_formed() const;

	/// Probably expensive operation.

	void SetOutgoing(std::vector<Handle> _hs);

	/// Create a copy of this wrapper into the core, regardless whether this
	/// was created from a core handle.

	Handle attach(iAtomTableWrapper* core) const {
		return cx;
	};
	void detach() const;

	/// Copy this as an integer array into the dest array

	int asIntegerArray(unsigned int* dest, unsigned int patlen, std::map<atom,int,lessatom>& node2pat_id,
						unsigned int& next_free_pat_id, int index = 0) const;


	MetaPredicate* getMeta() const;
	Handle getHandle() const;

	bool operator==(const atom& rhs) const;

	Type execType() const;
	std::vector<boost::shared_ptr<atom> > execOutTree() const;
	bool matchType(const atom& rhs) const;
	bool matchType(Type rhsT) const;
	std::string execName() const { return name; }

	bool containsVar() const;
	bool containsFWVar() const;

	bool forbidLastSubstitution() const;

	/** Metapredicate operation */

	bool operator()(Handle h) const;

	static bool ValidMetaPredicate(Type T);

	static std::vector<Handle> convertVector(const std::vector<boost::shared_ptr<atom> >& hs,
										iAtomTableWrapper* table);

	void substitute(Handle rhs, std::string varname);
	void substitute(Handle dest, atom src);
	void substitute(const atom& dest, const atom& src);
	void substitute(const atom& rhs, std::string varname);
	
	/// TODO: Write proper visitor object!
	tree< boost::shared_ptr<atom> > maketree() const; //tree<atom>& dest);
	
	tree<Vertex> makeHandletree(iAtomTableWrapper* table, bool fullVirtual=false) const; //tree<atom>& dest);

	/// Actualize the substitution from bindings, delete forbiddenBindings, on new instance..

	void getWithActualizedSubstitutions(atom& target) const;
	Handle bindHandle(iAtomTableWrapper* table) const;

	void extractVars(set<std::string>& vars) const;
	void extractFWVars(set<std::string>& vars) const;

	atom(const tree<boost::shared_ptr<atom> >& a, tree<boost::shared_ptr<atom> >::iterator parent_node, bool root = true);
	atom(const tree<Vertex>& a, tree<Vertex>::iterator parent_node, bool root = true);
};
/*
struct less_subst: public binary_function<subst, subst, bool>
{
    bool operator()(const subst& lhs, const subst& rhs) const
    {
    	return lhs.first < rhs.first; // Only compare the std::string keys
    }
};*/

struct lessatom : public binary_function<atom, atom, bool>
{
    bool operator()(const atom& lhs, const atom& rhs) const;
};

struct lessatom_ignoreVarNameDifferences : public binary_function<atom, atom, bool>
{
    bool operator()(const atom& lhs, const atom& rhs) const
	{
		int diff = nodedifference(lhs, rhs);

		if (diff < 0)
			return true;
		else
			return false;
	}

	int nodedifference(const atom& lhs, const atom& rhs) const
	{
		//hack: ATOM type equivals "ANY"
		if (  (lhs.T == ATOM && rhs.T != FW_VARIABLE_NODE)
			||(rhs.T == ATOM && lhs.T != FW_VARIABLE_NODE))
			return 0;

		int lhsize = (int)lhs.hs.size();
		int rhsize = (int)rhs.hs.size();

		if (lhs.T < rhs.T)
			return -1;
		if (lhs.T > rhs.T)
			return 1;


		if (lhsize < rhsize)
			return -1;
		if (lhsize > rhsize)
			return 1;
			
		if ((lhs.T != VARIABLE_NODE) && rhs.T != FW_VARIABLE_NODE)
		{
			if (lhs.name < rhs.name)
				return -1;
			if (lhs.name > rhs.name)
				return 1;
		}

		if (lhsize == 0)
			return 0;

		/// Otherwise equal as nodes, with same arity. Then, check outgoing set:

		for (int i = 0; i < lhsize; i++)
		{
			int diff = nodedifference(*lhs.hs[i], *rhs.hs[i]);

			if (diff < 0)
				return -1;
			if (diff > 0)
				return 1;
		}

		return 0;
	}
};

void printAtomTree(const atom& a, int level = 0, int LogLevel = 5);

#define equal_atom_ignoreVarNameDifferences(a,b) \
	(!lessatom_ignoreVarNameDifferences()(a,b) && \
	!lessatom_ignoreVarNameDifferences()(b,a))

template<int L>
class atom_print
{
public:
	void operator()(const atom& rhs) { printAtomTree(rhs, 0,L); }
};


template<int L>
class handle_print
{
public:
	void operator()(Handle rhs) { 
	//	printTree(rhs, 0,L); 
	}
};

typedef set<atom, lessatom> atomset;

void getAtomTreeString(const atom& a, std::string& outbuf);

void VariableMPforms(const atom& src, set<atom, lessatom_ignoreVarNameDifferences>& res,
					 set<subst>* forbiddenBindings);
bool getLargestIntersection2(const set<atom,lessatom>& keyelem_set,
							const std::vector<Handle>& link_set, std::vector<boost::shared_ptr<atom> >& result);

atom* neBoundVertexWithNewType(Handle h, Type T);

} //namespace

//#include "iAtomTableWrapper.h"

#endif
