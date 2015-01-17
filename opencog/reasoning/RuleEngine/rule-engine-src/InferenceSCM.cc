/*
 * InferenceSCM.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  Sept 2014
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
#include "InferenceSCM.h"

#include <opencog/guile/SchemePrimitive.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/reasoning/RuleEngine/rule-engine-src/pln/ForwardChainer.h>
#include <opencog/reasoning/RuleEngine/rule-engine-src/pln/BackwardChainer.h>
#include <opencog/atomspace/AtomSpace.h>

using namespace opencog;

InferenceSCM* InferenceSCM::_inst = NULL;

InferenceSCM::InferenceSCM() {
	if (NULL == _inst) {
		_inst = this;
		init();
	}
}

InferenceSCM::~InferenceSCM() {

}

void InferenceSCM::init(void) {
	_inst = new InferenceSCM();
#ifdef HAVE_GUILE
	//all commands for invoking the rule engine from scm shell should be declared here
	define_scheme_primitive("cog-fc", &InferenceSCM::do_forward_chaining,
			_inst); //eg. from scm shell (cog-fc (InheritanceLink (ConceptNode "cat")(ConceptNode "animal"))
	define_scheme_primitive("cog-bc", &InferenceSCM::do_backward_chaining,
			_inst); //backward chaining
#endif
}

Handle InferenceSCM::do_forward_chaining(Handle h) {
#ifdef HAVE_GUILE
	AtomSpace *as = SchemeSmob::ss_get_env_as("cog-fc");
	ForwardChainer fc(as);
	fc.do_chain(h); //START FORWARD CHAINING
	HandleSeq result = fc.get_chaining_result();
	return as->addLink(LIST_LINK, result, TruthValue::DEFAULT_TV());
#else
	return Handle::UNDEFINED;
#endif
}

Handle InferenceSCM::do_backward_chaining(Handle h) {
#ifdef HAVE_GUILE
	AtomSpace *as = SchemeSmob::ss_get_env_as("cog-bc");
	BackwardChainer bc(as);
	bc.do_chain(h);
	map<Handle, HandleSeq> soln = bc.get_chaining_result();
	HandleSeq soln_list_link;
	for (auto it = soln.begin(); it != soln.end(); ++it) {
		HandleSeq hs;
		hs.push_back(it->first);
		hs.insert(hs.end(), it->second.begin(), it->second.end());
		soln_list_link.push_back(
				as->addLink(LIST_LINK, hs, TruthValue::DEFAULT_TV()));
	}
	return as->addLink(LIST_LINK, soln_list_link, TruthValue::DEFAULT_TV());
#else
	return Handle::UNDEFINED;
#endif
}
