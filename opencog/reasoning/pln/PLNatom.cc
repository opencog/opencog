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

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/utils.h>

#include "PLN.h"
#include "AtomSpaceWrapper.h"
#include "PLNatom.h"

using namespace std;

namespace test
{
	extern int attachs;
}

namespace reasoning
{
int atom_alloc_count=0;
int inode_alloc_count=0;
	
extern std::map<int, string> type2name;
	
bool existMPin(const vector<Btr<atom> >& hs);

bool atom::ValidMetaPredicate(Type T)
{
	return GET_ATW->inheritsType(T, RESTRICTOR) ||
		GET_ATW->inheritsType(T, AND_LINK) ||
		GET_ATW->inheritsType(T, __OR) ||
		GET_ATW->inheritsType(T, NOT_LINK);
}

bool lessatom::operator()(const atom& lhs, const atom& rhs) const
{
    if (currentDebugLevel==5)
    {
        printAtomTree(lhs,0,0);
        printAtomTree(rhs,0,0);
    }
    if (lhs.T < rhs.T)
        return true;
    if (lhs.T > rhs.T)
        return false;

    if (lhs.hs.size() < rhs.hs.size())
        return true; 
    if (lhs.hs.size() > rhs.hs.size())
        return false;

    if (lhs.name < rhs.name)
        return true;
    if (lhs.name > rhs.name)
        return false;

    for (unsigned int i = 0; i < lhs.hs.size(); i++)
        if (lessatom()(*lhs.hs[i], *rhs.hs[i]))
            return true;
        else
            if (lessatom()(*rhs.hs[i], *lhs.hs[i]))
                return false;

    return false;
}

//bool MetaPredicate::operator()(HandleType h) const
bool atom::operator()(Handle h) const
{
	uint s=0;

bool echo=false;
#define p(str) if (echo) puts(str);

	/// haxx:
	if (T == ATOM)
		return true;
	
	if (GET_ATW->inheritsType(T, RESTRICTOR))
	{
		bool unnormed_ret = false;
		bool normal_form = (T == __INSTANCEOF_N || T == __EQUALS_N);
		
		switch(T)
		{
		case __INSTANCEOF_N:
			unnormed_ret = GET_ATW->inheritsType(GET_ATW->getType(h), hs[0]->T);
			
			break;
			
		case __EQUALS_N:
			
			unnormed_ret = (atom(h) == *this->hs[0]);
			
			break;
		}
		
		if (!unnormed_ret && normal_form)
		{
			if (GET_ATW->inheritsType(GET_ATW->getType(h), FORALL_LINK))
				return (*this)(GET_ATW->getOutgoing(h)[1]);
			else if (GET_ATW->inheritsType(hs[0]->T, FORALL_LINK))
				return ( *((MetaPredicate*)hs[1].get()) )(h);
			else
				return false;
		}
		else if (normal_form)
			return true;

		assert(0);
	}
	
	if (T == AND_LINK) {
        p("AND");
		for (s = 0; s < hs.size(); s++)
			if (!(*((MetaPredicate*)(hs[s].get())))(h))
			{
				p("no");
				return false;
			}
			p("yes");
			return true;
    } else if (T == OR_LINK || T == __OR) {
        p("OR");
		for (s = 0; s < hs.size(); s++)
			if ((*((MetaPredicate*)(hs[s].get())))(h))
			{
				p("yes");
				return true;
			}
			p("no");
			return false;
    } else if (T == NOT_LINK) { 
        for (s = 0; s < hs.size(); s++) //Actually this is "NOT AND"
						if (!(*((MetaPredicate*)(hs[s].get())))(h))
						{
							p("yes");
							return true;
						}
						p("no");
						return false;
	}

	return (this->real == h || (*this) == atom(h));
}
	
atom::~atom()
{
	atom_alloc_count--;
}

atom::atom()
:real(0), bindings(0), T(ATOM), arity(0), forbiddenBindings(0)
{
	atom_alloc_count++;
}

atom::atom(Type _T, vector<Btr<atom> > _hs)
: real(0), bindings(0), T(_T), hs(_hs), forbiddenBindings(0)
{
	atom_alloc_count++;
}

atom::atom(Handle h)
: bindings(0), forbiddenBindings(0)
{
	setHandle(h);
	atom_alloc_count++;
}

atom::atom(const atom& rhs)
:   real(rhs.real), bindings(rhs.bindings), T(rhs.T), arity(rhs.arity),
    name(rhs.name), cx(rhs.cx), hs(rhs.hs), 
//,bindings(0), forbiddenBindings(0)
forbiddenBindings(rhs.forbiddenBindings)
{
	atom_alloc_count++;
}

atom::atom(Type _T, string _name)
: real(0),bindings(0), T(_T), arity(0), name(_name), 
forbiddenBindings(0)
{
	atom_alloc_count++;
	//printf("Node: %d / %d\n", T, arity);
}

/**
	If the atom has 1 child with substitutions and doesn't itself contain substs,
	it takes it's child's substs. Otherwise, behaviour is undefined.
*/

atom::atom(Type _T, int _arity, ...)
: real(0), bindings(0), T(_T), arity(_arity), forbiddenBindings(0)
{
LOG(5, "Variable arity argument list processing...");
	try
	{
		va_list vars;
		
		va_start(vars, _arity);
		
		//		printf("Link: %d / %d\n", T, _arity);
		
		for (int i = 0; i < _arity; i++)
		{
/// haxx:: with pointers / automatic variables
			atom* next_atom_ptr = va_arg(vars, atom*);
			hs.push_back(Btr<atom>(new atom(*next_atom_ptr)));
			delete next_atom_ptr;
			Btr<atom> next_atom  = hs.back();
			
			if (next_atom->bindings && !next_atom->bindings->empty())
			{
				if (this->bindings)
				{
					assert (this->bindings->empty());

					*this->bindings = *next_atom->bindings;
				}
				else
					this->bindings = next_atom->bindings;
			}
			if (next_atom->forbiddenBindings && !next_atom->forbiddenBindings->empty())
			{
				if (this->forbiddenBindings)
				{
					assert (this->forbiddenBindings->empty());

					*this->forbiddenBindings = *next_atom->forbiddenBindings;
				}
				else
					this->forbiddenBindings = next_atom->forbiddenBindings;
			}

			//			printf("\t%d / %d\n", next.T, next.arity);
		}
		
		va_end(vars);
	  } catch(...) { puts("atom() exception."); int d; scanf("%d",&d); }

LOG(5, "Variable arity argument list ok.");

	atom_alloc_count++;
}

void atom::setHandle(Handle h)
{
/*
    if ((int)h < NUMBER_OF_CLASSES) {
    printf("GOT AN INVALID HANDLE (%d) TO BUILD A reasoning::atom object!!!\n", h); 
            T = (Type)(int)h;
    real = 0;
            return;
    }
*/
	T=GET_ATW->getType(h);
	name=GET_ATW->getName(h);
	HandleSeq _hs = GET_ATW->getOutgoing(h);
//	arity= _hs.size();
	cx=Handle::UNDEFINED;

	hs.clear();
	for (unsigned int i = 0; i < _hs.size(); i++)
		hs.push_back(Btr<atom>(new atom(_hs[i])));

	real = h;
	//SetOutgoing(_hs);
}


Handle atom::bindHandle(iAtomSpaceWrapper* table) const
{
	atom target(*this);

	if (bindings)
		for (map<string,atom>::const_iterator i = bindings->begin(); i != bindings->end();
			i++)
			target.substitute(i->second, i->first);

	return target.attach(table);
}

void atom::getWithActualizedSubstitutions(atom& target) const
{
	target = atom(*this);

	if (bindings)
		for (map<string,atom>::const_iterator i = bindings->begin(); i != bindings->end();
			i++)
		{
			target.substitute(i->second, i->first);
			target.forbiddenBindings = this->forbiddenBindings;
		}
	target.bindings = this->bindings;

	target.real = this->real;

	if (target.real != Handle::UNDEFINED )
	{
		LOG(4, "Warning! Substituted the atom though it's attached to Core!");
	}
}


void atom::detach() const
{
	real = Handle::UNDEFINED;

	for (unsigned int i = 0; i < hs.size(); i++)
		hs[i]->detach();
}

bool atom::well_formed() const
{
	bool arity2 = (hs.size() == 2);

	if (!arity2 && (T == FORALL_LINK || T == IMPLICATION_LINK || T == INHERITANCE_LINK ))
		return false;

	if (T == FORALL_LINK && hs[0]->T != LIST_LINK && hs[0]->T != FW_VARIABLE_NODE)
		return false;

	return true;
}

#if 0
int atom::asIntegerArray(unsigned int* dest, unsigned int patlen, map<atom,int,lessatom>& node2pat_id,
							unsigned int& next_free_pat_id, unsigned int index) const
{
	if (index >= patlen)
		throw string("Warning: Too long pattern for the pre-defined matrix!");

	assert(index < patlen);
	bool firstElem = (index==0);

	if (inheritsType(T, NODE))
	{
		map<atom,int,lessatom>::iterator my_pat_id = node2pat_id.find(*this);
		if (my_pat_id == node2pat_id.end())
		{
			fim::pat_id pid = 300+(real ? (unsigned int)real : next_free_pat_id++);

			dest[index++] = pid;
			node2pat_id[*this] = pid;
		}
		else
			dest[index++] = my_pat_id->second;
	}
	else
	{
		dest[index++] = 2+T;

		unsigned int i=0;
		
		for (i = 0; i < hs.size(); i++)
			index = hs[i].asIntegerArray(dest, patlen, node2pat_id, next_free_pat_id, index);
		for (; i < 3; i++, index++)
			dest[index] = 1;
	}

	if (firstElem)
	{
		for (unsigned int i = index; i < patlen; i++)
			dest[i] = 1;
	}

	return index;
}
#endif

Handle atom::attach(iAtomSpaceWrapper* core) const
{
//printf("atom::attach()\n");
	::test::attachs++;

    AtomSpaceWrapper* at = (AtomSpaceWrapper*) core;
	SimpleTruthValue tvn(1.0, 0.0);

LOG(4, "Attaching...");

//	assert (!inheritsType(T, FW_VARIABLE_NODE));

	if (GET_ATW->inheritsType(T, NODE))
	{
//printf("atom::attach: it's a node: type: %d, name: %s, core: %p\n", T, name.c_str(), core);
        real = at->getHandle(T, name);
//printf("atom::attach: real = %p\n", real);
		if (real == Handle::UNDEFINED)
		{
//printf("atom::attach: real is null => adding node\n");
            real = at->addNode(T, name, tvn, true);
cprintf(4, "Added node as NEW: %s / [%d]\n", name.c_str(), real.value());
		}
	}
	else
	{
		vector<Handle> outg;

		for (unsigned int i = 0; i < hs.size(); i++)
		{		
			outg.push_back(hs[i]->attach(core));
		}
cprintf(4, "Attaching %d entries...\n", outg.size());
		
        if ((real = at->getHandle(T, outg))==Handle::UNDEFINED)
		{
cprintf(4, "Not exist.\n");
            real = at->addLink(T, outg, tvn, true);
		}
	}
	
LOG(4, "Attached.");

	assert(real != Handle::UNDEFINED);

	return real;
}

void atom::SetOutgoing(HandleSeq _hs)
{
	for (unsigned int i = 0; i < _hs.size(); i++)
		hs.push_back(Btr<atom>(new atom(_hs[i])));
}

bool atom::operator==(const atom& rhs) const
{
	return !lessatom_ignoreVarNameDifferences()(*this, rhs)
		&& !lessatom_ignoreVarNameDifferences()(rhs, *this);
}

MetaPredicate* atom::getMeta() const
{
	assert(MetaPredicate::ValidMetaPredicate(T));

	return (MetaPredicate*)this;
}

Type atom::execType() const
{
//	if (inheritsType(T, __INDEXER) && hs.size() > (T - __INDEX1))
//		return hs[T - __INDEX1].execType();
	if (T == INSTANCEOF_R || T == __INSTANCEOF_N)
		return hs[0]->T;
	if ((T == AND_LINK || T == OR_LINK) && existMPin(hs))
		return hs[0]->execType();

	return T;
}

vector<Btr<atom> > atom::execOutTree() const
{
//	if (inheritsType(T, __INDEXER) && hs.size() > (T - __INDEX1))
//		return hs[T - __INDEX1].execOutTree();
	if (GET_ATW->inheritsType(T, RESTRICTOR))
		return vector<Btr<atom> >();

	return hs;
}

bool atom::matchType(const atom& rhs) const
{
	return matchType(rhs.execType());
}

bool atom::matchType(Type rhsT) const
{
	Type lhsT = this->execType();

	return lhsT == rhsT || GET_ATW->inheritsType(lhsT, rhsT)
        || GET_ATW->inheritsType(rhsT, lhsT);
}

bool atom::operator<(const atom& rhs) const
{
	if (T < rhs.T || arity < rhs.arity || name < rhs.name)
		return true;
	for (unsigned int i = 0; i < hs.size(); i++)
		if (*hs[i] < *rhs.hs[i])
			return true;

	return false;
}

void atom::substitute(const atom& dest, const atom& src)
{
	if (!lessatom()(src,*this) && !lessatom()(*this,src)) //(this->attach()))
		*this = dest; 
	else
	{
		for (unsigned int i = 0; i < hs.size(); i++)
			hs[i]->substitute(dest, src);
	}

		/// Behold: the Performance Hog!

	if (bindings)
		for (map<string, atom>::iterator	s = bindings->begin();
											s!= bindings->end(); s++)
			s->second.substitute(dest, src);

}

void atom::substitute(const atom& rhs, string varname)
{
	substitute(rhs, atom(VARIABLE_NODE, varname));
	substitute(rhs, atom(FW_VARIABLE_NODE, varname));
}

void atom::substitute(Handle dest, atom src)
{
	if (src == *this) //(this->attach()))
		setHandle(dest);
	else
	{
		for (unsigned int i = 0; i < hs.size(); i++)
			hs[i]->substitute(dest, src);
	}

	if (bindings)
		for (map<string, atom>::iterator	s = bindings->begin();
											s!= bindings->end(); s++)
			s->second.substitute(dest, src);
}

void atom::substitute(Handle rhs, string varname)
{
	substitute(rhs, atom(VARIABLE_NODE, varname));
	substitute(rhs, atom(FW_VARIABLE_NODE, varname));
}

vector<Handle> atom::convertVector(const vector<Btr<atom> >& hs, iAtomSpaceWrapper* table)
{
	vector<Handle> ret;		

	for (unsigned int i = 0; i < hs.size(); i++)
		ret.push_back(hs[i]->real != Handle::UNDEFINED? hs[i]->real : hs[i]->attach(table));

	return ret;

}

/// Remove the explicit outgoingset (hs) and replace it with a tree structure.

void prn(tree< Btr<atom> >& tr)
{
	tree< Btr<atom> >::sibling_iterator sib=tr.begin();
      while(sib!=tr.end()) {
         cout << type2name[(*sib)->T] << endl;
         ++sib;
         }
      cout << endl;
      tree< Btr<atom> >::iterator sib2=tr.begin();
      tree< Btr<atom> >::iterator end2=tr.end();
      while(sib2!=end2) {
         for(int i=0; i<tr.depth(sib2); ++i) 
            cout << " ";
         cout << type2name[(*sib2)->T] << endl;
         ++sib2;
      }
}
/*
void makeHandletree(Handle real, iAtomSpaceWrapper* table, bool fullVirtual, tree<Vertex>& ret) const
{
	Handle top=(Handle)0;
	Type T=GET_ATW->getType(real);

	if (!fullVirtual || inheritsType(T, NODE))
		ret.set_head(real);
	else //Virtual
	{
		ret.set_head((Handle)T);
	
		HandleSeq _hs = GET_ATW->getOutgoing(real);
		foreach(Handle child_h, hs)
		{
			tree<Vertex> child = makeHandletree(child_h, table, fullVirtual);
			ret.append_children(ret.begin(), child, child);
		}
	}
}
*/

void expandHandletree(bool fullVirtual, vtree& ret, tree<Vertex>::iterator ret_top);
void makeHandletree(Handle real, bool fullVirtual, tree<Vertex>& ret)
{
	ret.set_head(real);
	expandHandletree(fullVirtual, ret, ret.begin());
}

void expandHandletree(bool fullVirtual, vtree& ret, tree<Vertex>::iterator ret_top)
{
    // Although it's called real, it's a fake PLN handle
	Handle real = v2h(*ret_top);
	Type T=GET_ATW->getType(real);

	/// If virtual link, then we keep expanding
	if (fullVirtual && !GET_ATW->inheritsType(T, NODE))
	{
		*ret_top = Vertex((Handle)T);
	
		HandleSeq _hs = GET_ATW->getOutgoing(real);
		foreach(Handle child_h, _hs)
		{
			tree<Vertex>::iterator next_i = ret.append_child(ret_top, child_h);
			expandHandletree(fullVirtual, ret, next_i);
		}
	}
}

tree<Vertex> atom::makeHandletree(iAtomSpaceWrapper* table, bool fullVirtual) const
{
	tree<Vertex> ret;
	Handle top = Handle::UNDEFINED;
	
	if (real != Handle::UNDEFINED && (!fullVirtual || GET_ATW->inheritsType(GET_ATW->getType(real), FW_VARIABLE_NODE)))
		top = real;
	else
		top = (GET_ATW->inheritsType(T, NODE)
					? attach(table)
					: (Handle)T);

	ret.set_head(top);
	
	for (unsigned int i = 0; i < this->hs.size(); i++)
	{
		tree<Vertex> children = this->hs[i]->makeHandletree(table, fullVirtual);

		ret.append_children(ret.begin(), children.begin(), children.end());
	}

	return ret;
}

tree< Btr<atom> > atom::maketree() const //tree<Btr<atom> >& dest)
{
	Btr<atom> pseudo_atom(new atom(this->T, this->name));
	pseudo_atom->real = this->real;
	tree< Btr<atom> > ret(pseudo_atom);

	for (unsigned int i = 0; i < this->hs.size(); i++)
	{
		tree< Btr<atom> > children = this->hs[i]->maketree();

		ret.append_children(ret.begin(), children.begin(), children.end());
	}

	return ret;
}

///hacky

atom::atom(const tree<Btr<atom> >& a, tree<Btr<atom> >::iterator parent_node, bool root)
: bindings(0), forbiddenBindings(0)
{
	if (root)
		parent_node = a.begin();

	T = (*parent_node)->T;
	name = (*parent_node)->name;
	real = (*parent_node)->real;

	if (!GET_ATW->inheritsType(T, NODE)) //nodes have no children...
		for (tree<Btr<atom> >::sibling_iterator s = a.begin(parent_node);  s != a.end(parent_node); ++s)
		{	
			Btr<atom> b(new atom);
			
			if (s.number_of_children() > 0 && !GET_ATW->inheritsType((*s)->T, NODE) )
				b = Btr<atom>(new atom(a, s, false)); //a.child(s, 0), &this->hs);
			
			b->T = (*s)->T;
			b->name = (*s)->name;
			b->real = (*s)->real;
			
			hs.push_back(b);
		}
}


///hacky

atom::atom(const tree<Vertex>& a, tree<Vertex>::iterator parent_node, bool root)
: bindings(0), forbiddenBindings(0)
{
	if (root)
		parent_node = a.begin();
	
	if (parent_node == a.end())
		return;

	T = GET_ATW->getType(boost::get<Handle>(*parent_node));
	name = GET_ATW->getName(boost::get<Handle>(*parent_node));
	real = boost::get<Handle>(*parent_node);
	
//cprintf(4,"REAL: %d\n", real);
	
	if (!GET_ATW->inheritsType(T, NODE)) //nodes have no children...
		for (tree<Vertex>::sibling_iterator s = a.begin(parent_node);  s != a.end(parent_node); ++s)
		{	
			Btr<atom> b(new atom);
			
			if (s.number_of_children() > 0 && !GET_ATW->inheritsType(GET_ATW->getType(boost::get<Handle>(*s)), NODE) )
				b = Btr<atom>(new atom(a, s, false)); //a.child(s, 0), &this->hs);

			b->T = GET_ATW->getType(boost::get<Handle>(*s));
			b->name = GET_ATW->getName(boost::get<Handle>(*s));
			b->real = boost::get<Handle>(*s);
//cprintf(4,"REAL SUB: %d\n", b.real);			
			hs.push_back(b);
		}
}

bool atom::containsVar() const
{ 
	if (!GET_ATW->inheritsType(T, NODE))
	{
		for (unsigned int i=0; i<hs.size(); ++i)
			if (hs[i]->containsVar())
				return true;
			return false;
	}
	return GET_ATW->inheritsType(T, VARIABLE_NODE);
}


bool atom::containsFWVar() const
{ 
	if (!GET_ATW->inheritsType(T, NODE))
	{
		for (unsigned int i=0; i<hs.size(); ++i)
			if (hs[i]->containsFWVar())
				return true;
			return false;
	}
	return GET_ATW->inheritsType(T, FW_VARIABLE_NODE);
}

const int MAX_VARIABLE_NUMBER = 5;

void addatomcopy(const tree<Btr<atom> >& a, set<atom, lessatom_ignoreVarNameDifferences>& res)
{
	res.insert( atom(a, a.begin()) );
}

void atom::extractVars(set<string>& vars) const
{
	if (GET_ATW->inheritsType(T, VARIABLE_NODE))
	{
		vars.insert(name);
	}
	else
		for (unsigned int i = 0; i < hs.size(); i++)
			hs[i]->extractVars(vars);
}

void atom::extractFWVars(set<string>& vars) const
{
	if (GET_ATW->inheritsType(T, FW_VARIABLE_NODE))
	{
		vars.insert(name);
	}
	else
		for (unsigned int i = 0; i < hs.size(); i++)
			hs[i]->extractFWVars(vars);
}

/*atom varFormula2ForAll(const atom& a)
{
	set<string> vars;

	a.extractVars(vars);

	atom* varlist = new atom(LIST_LINK,0);
	for (set<string>::iterator i = vars.begin(); i != vars.end(); i++)
		varlist->hs.push_back(atom(VARIABLE_NODE, *i));

	/// Transfer ownership of varlist forward:

	return atom(FORALL_LINK, 2,
				varlist,
				new atom(a));			
}*/

bool atom::forbidLastSubstitution() const
{
	if (!bindings || bindings->empty())
		return false;
	if (!forbiddenBindings)
		forbiddenBindings = new set<subst>;

	map<string,atom>::reverse_iterator endi
		= bindings->rbegin();
	string rkey = endi->first;
	
	forbiddenBindings->insert(*endi);
	bindings->erase(rkey);

	return true;
}


} //~namespace reasoning

