/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#include "PLN.h"

#include "PLNRules/Rule.h"
#include "Rules.h"
#include "AtomSpaceWrapper.h"
#include "PLNatom.h"

namespace opencog { namespace pln {

bool substitutableTo(atom& from, atom& to,
						map<string,atom>& bindings,
						const set<subst>& forbiddenBindings,
					 pair<string,atom>** restart_from,
					 pair<string,atom>** restart_to);
}

namespace haxx
{
	static int localcount=0;
	using namespace opencog { namespace pln {;

	bool getHandleSet(Type T, const string& name, AtomLookupProvider* aprovider, vector<Handle>& res);
	
	void dovar3(tree<Btr<atom> >& ta,
		set<atom,lessatom_ignoreVarNameDifferences>& res,
		set<subst>* forbiddenBindings,
		iAtomSpaceWrapper* table)
	{
		vector<Handle> candidates;
		atom unsubsted_target(ta, ta.begin());

///LOG(4, "dovar3: unsubsted_target:");
///printAtomTree(unsubsted_target,0,4);

		// haxx::
//		getHandleSet(FORALL_LINK, "", table, candidates);
		Btr<set<Handle> > scandidates = table->getHandleSet(FORALL_LINK, "");
		for (set<Handle>::iterator j=scandidates->begin();j!=scandidates->end();j++)
			candidates.push_back(*j);
		
		for (uint i = 0; i < candidates.size(); i++)
		{
printf("dovar3: Next candidate...");
			atom target(unsubsted_target); //'target' will experiece substs
			if (asw->getArity(candidates[i]) < 2)
				continue;

			atom ForAll_candidate(asw->getOutgoing(candidates[i],1));
///LOG(4, "dovar3: candidate[i]:");
///printAtomTree(ForAll_candidate,0,4);

			///haxx:: (ForAll_candidate contains ImpLink but points to the ForAllLink...
			ForAll_candidate.real = candidates[i];

			//assert(!a.bindings || a.bindings->empty());

			/// Each candidate is independent of the others.
			/// Since forbiddens are governed from higher level, they must point to the
			/// same copy of the set.

			ForAll_candidate.bindings = new map<string,atom>;
			ForAll_candidate.forbiddenBindings = new set<subst>(*forbiddenBindings);
	
			pair<string,atom>* restart_from = NULL;
			pair<string,atom>* restart_to = NULL;

			/// Save this one. ForAll_candidate will be substed.

			atom unsubsted_ForAll_candidate(ForAll_candidate);
printf("dovar3: substitutableTo:");
			/**
				ForAll_candidate.bindings stores the substitutions of FW variables
				that are found in target. Substs of the BW variables in ForAll_cand are
				not stored, since they're only used to find out the potential.. This is
				actually waste of resources, since (some of) these must be re-computed.
			*/

			/// Whether we can create 'a' from ForAll_candidate by substitutions.

			while (substitutableTo(	ForAll_candidate, target,
									*ForAll_candidate.bindings,
									*ForAll_candidate.forbiddenBindings,
									&restart_from, &restart_to))
			{
				/// Only gets here if subst was ok.

				assert(!restart_from || !restart_to); // Can't have both

				if (restart_from)	/// But if restart asked for, then we must try again
				{
///					LOG(4, "Substituting for " + restart_from->first);
///					printAtomTree(restart_from->second,0,4);

					ForAll_candidate.substitute(restart_from->second, restart_from->first);
					target.substitute(restart_from->second, restart_from->first);

					delete restart_from;

//					printAtomTree(ForAll_candidate,0,1);
//					printAtomTree(a,0,1);

					restart_from = NULL;
				}
				else if (restart_to)	/// But if restart asked for, then we must try again
				{
///					LOG(4, "Substituting for " + restart_to->first);
///					printAtomTree(restart_to->second,0,4);

					ForAll_candidate.substitute(restart_to->second, restart_to->first);
					target.substitute(restart_to->second, restart_to->first);

					delete restart_to;

//					printAtomTree(ForAll_candidate,0,1);
//					printAtomTree(a,0,1);

					restart_to = NULL;
				}
				else			/// Otherwise, we're clear.
				{
				
/*					unsubsted_ForAll_candidate.T = FORALL_LINK;
					unsubsted_ForAll_candidate.hs[0] = atom(asw->getOutgoing(candidates[i],0));				
					
					/// This will return a description of ForAllLink which !exists.
					unsubsted_ForAll_candidate.hs[1] = unsubsted_target;
					
					/// But we will pass the correct ForAll atom in the 'real' var.
					unsubsted_ForAll_candidate.real = candidates[i];
*/					
					/// Replace remaining (free) variables with FW_Variables.

					set<string> remainingVariables;
					ForAll_candidate.extractVars(remainingVariables);
					for(set<string>::iterator	v = remainingVariables.begin();
												v != remainingVariables.end(); v++)
					{
						pHandle FW_var = _v2h(CreateVar(table));
						unsubsted_ForAll_candidate.substitute(
							atom(FW_var),
							*v);
					}
					
					vtree vt(unsubsted_ForAll_candidate.makeHandletree(ASW(), true));
					//vtree vt(ForAll_candidate.makeHandletree(ASW(), true));
					
					rawPrint(vt, vt.begin(), 4);
//					getc(stdin);
/*
	if (ForAll_candidate.bindings)
		for (map<string, atom>::iterator	s = ForAll_candidate.bindings->begin();
											s!= ForAll_candidate.bindings->end(); s++)
			printAtomTree(s->second,0,1);
*/
/*	if (unsubsted_ForAll_candidate.bindings)
		for (map<string, atom>::iterator	s = unsubsted_ForAll_candidate.bindings->begin();
											s!= unsubsted_ForAll_candidate.bindings->end(); s++)
											{
			LOG(1, s->first);
			printAtomTree(s->second,0,1);
											}

printAtomTree(unsubsted_ForAll_candidate,0,1);*/
localcount++;
					res.insert(unsubsted_ForAll_candidate);
					break;
				}
//				printAtomTree(ForAll_candidate,0,0);
//				printAtomTree(target,0,0);
			}	
		}
	}
}

namespace opencog { namespace pln {
{
void VariableMPforms(const atom& src, set<atom, lessatom_ignoreVarNameDifferences>& res,
					 set<subst>* forbiddenBindings);

Rule::setOfMPs CrispUnificationRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
	if (asw->isSubType(_v2h(*outh->begin()), FORALL_LINK) ||
		asw->isSubType(_v2h(*outh->begin()), FW_VARIABLE_NODE))
		return Rule::setOfMPs();
	
	set<atom, lessatom_ignoreVarNameDifferences> varforms;
	
	/// Gather hints:
printf("VariableMPforms...");
//printAtomTree(outh, 0,0);
	//asw->VariableMPforms(atom(*outh,outh->begin()), varforms, outh.forbiddenBindings);
	
	set<subst> unused_forbiddenBindings;
	
	VariableMPforms(atom(*outh,outh->begin()), varforms,&unused_forbiddenBindings);
printf("VariableMPforms was ok.");	
 	if (varforms.empty())
		return Rule::setOfMPs();
printf("VariableMPforms was non-empty.");
	setOfMPs ret;
	
	for (set<atom,lessatom_ignoreVarNameDifferences>::iterator i = varforms.begin(); i != varforms.end(); i++)
	{
		MPs ret1;
	
		boost::shared_ptr<atom> Andform(new atom(i->real));
printf("And formm:\n");		
		printAtomTree(*Andform,0,3);
		ret1.push_back(BBvtree(new BoundVTree(Andform->makeHandletree(destTable))));
		
//		ret1.push_back(BBvtree(new tree<Vertex>(mva(i->real))));
		printTree(i->real,0,3);

		boost::shared_ptr<atom> rootAtom(new atom(HYPOTHETICAL_LINK, 1, new atom()));
		
		/// Remove OLD free FW variables from the topohypo:
		atom tempa(*outh, outh->begin());
		tempa.bindings = i->bindings;
		//i->getWithActualizedSubstitutions(rootAtom->hs[0]);
		tempa.getWithActualizedSubstitutions(rootAtom->hs[0]);

printf("Root atom:\n");
		printAtomTree(*rootAtom,0,3);
printf("Partial CrispU args:\n");

/*		vtree root_children(rootAtom->hs[0].makeHandletree(asw));
		for (vtree::sibling_iterator p = root_children.begin(root_children.begin());
									p != root_children.end(root_children.begin());
									p++)
		{
			rawPrint(vtree(p,p),0,0);			
		}*/
		vector<atom>& parent_hs = *rootAtom; //->hs[0].hs;
		for (vector<atom>::iterator p=parent_hs.begin(); p != parent_hs.end(); p++)
		{
//			if (!p->real)
		//vector<atom>::iterator p=rootAtom.begin(); 
			set<string> vars;
			p->extractFWVars(vars);
			if (!vars.empty())
			{
				boost::shared_ptr<atom> new_h(new atom(*p));

				new_h->detach();

				ret1.push_back(BBvtree(new BoundVTree(new_h->makeHandletree(asw))));

				printAtomTree(*new_h,0,3);
			}
		}
        
		rootAtom->detach();

		ret1.push_back(BBvtree(new BoundVTree(rootAtom->makeHandletree(asw))));

        // Debug printing
        MPs::iterator m = ret1.begin();
        for(; m != ret1.end(); m++)
            if (*m && (*m)->begin() != (*m)->end())
                rawPrint(**m,(*m)->begin(),3);
        //getc(stdin);
		
        printf("Crispu vector size=%u\n", (unsigned int) ret1.size());
		ret.insert(ret1);
	}
	
	/*		atom* Andform = new atom(AND_LINK, 2,
	new atom(__INSTANCEOF_N, 1, new atom(FORALL_LINK,0)),
	new atom(__INDEX2, 1, Orform)
	);*/

	overrideInputFilter = true;
	
	return ret;
}

void VariableMPforms(const atom& src, set<atom, lessatom_ignoreVarNameDifferences>& res,
					 set<subst>* forbiddenBindings)
{
	tree<Btr<atom> > tsrc = src.maketree();

	atom a(tsrc, tsrc.begin());

	printAtomTree(src,0,4);

	haxx::dovar3(tsrc, res, forbiddenBindings, ASW());

printf("dovar3 ok.");

if (!res.empty())
{
map<string,atom>* bindings = res.begin()->bindings;
}
//	dovar(tsrc.begin(), tsrc, res);

	for_each(res.begin(), res.end(), atom_print<4>());
}

}} //namespace opencog { namespace pln {

