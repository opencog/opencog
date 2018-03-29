/*
 * XPatternMinerSCM.cc
 *
 * Copyright (C) 2018 OpenCog Foundation
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com> 
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

#ifdef HAVE_GUILE

#include <opencog/guile/SchemeModule.h>

namespace opencog {

class XPatternMinerSCM : public ModuleWrap
{
protected:
	virtual void init();

	/**
	 * Given a pattern and a texts concept, return all shallow
	 * abstractions, pre-wrapped in Lists ready to be applied to the
	 * pattern.
	 *
	 * For instance, given
	 *
	 * pattern = (Lambda X Y (Inheritance X Y))
	 * texts  = { (Inheritance A B), (Inheritance A C) }
	 *
	 * returns
	 *
	 * Set
	 *   List A Y
	 *   List Y Y
	 *   List X B
	 *   List X C
	 *
	 * TODO: we really only want to return shallow abtractions that
	 * would reach minimum support.
	 */
	Handle do_shallow_abstract(Handle pattern,
	                           Handle texts);

public:
	XPatternMinerSCM();
};

} /*end of namespace opencog*/

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemePrimitive.h>

#include <opencog/learning/xpattern-miner/c++-based/XPatternMiner.h>

using namespace opencog;

XPatternMinerSCM::XPatternMinerSCM() : ModuleWrap("opencog xpattern-miner") {}

/// This is called while (opencog xpattern-miner) is the current
/// module.  Thus, all the definitions below happen in that module.
void XPatternMinerSCM::init(void)
{
	define_scheme_primitive("cog-shallow-abstract",
		&XPatternMinerSCM::do_shallow_abstract, this, "xpattern-miner");
}

Handle XPatternMinerSCM::do_shallow_abstract(Handle pattern, Handle texts)
{
	AtomSpace *as = SchemeSmob::ss_get_env_as("cog-shallow-abstract");

	// Fetch all texts
	HandleSet texts_set;
	IncomingSet member_links = texts->getIncomingSetByType(MEMBER_LINK);
	for (const LinkPtr l : member_links) {
		Handle member = l->getOutgoingAtom(0);
		if (member != texts)
			texts_set.insert(member);
	}

	// Generate all shallow abstractions
	HandleSetSeq shabs_per_var =
		XPatternMiner::shallow_abstract(pattern, texts_set);

	// Turn that sequence of handle sets into a set of ready to be
	// applied shallow abstractions
	const Variables& vars = XPatternMiner::get_variables(pattern);
	HandleSet sa_lists;
	unsigned vari = 0;         // Index of the variable
	for (const HandleSet& shabs : shabs_per_var) {
		for (const Handle& sa : shabs) {
			HandleSeq sa_list = vars.varseq;
			sa_list[vari] = sa;
			sa_lists.insert(as->add_link(LIST_LINK, sa_list));
		}
		vari++;
	}

	return as->add_link(SET_LINK, HandleSeq(sa_lists.begin(), sa_lists.end()));
}

extern "C" {
void opencog_xpatternminer_init(void);
};

void opencog_xpatternminer_init(void)
{
    static XPatternMinerSCM xpattern_miner;
    xpattern_miner.module_init();
}

#endif // HAVE_GUILE
