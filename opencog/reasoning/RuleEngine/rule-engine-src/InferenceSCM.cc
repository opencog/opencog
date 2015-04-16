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
#include <opencog/reasoning/RuleEngine/rule-engine-src/pln/DefaultForwardChainerCB.h>
#include <opencog/reasoning/RuleEngine/rule-engine-src/pln/BackwardChainer.h>
#include <opencog/atomspace/AtomSpace.h>

using namespace opencog;

InferenceSCM* InferenceSCM::_inst = NULL;

InferenceSCM::InferenceSCM()
{
    if (NULL == _inst) {
        _inst = this;
        init();
    }
}

InferenceSCM::~InferenceSCM()
{

}

void InferenceSCM::init(void)
{
    _inst = new InferenceSCM();
#ifdef HAVE_GUILE
    //all commands for invoking the rule engine from scm shell should be declared here
    define_scheme_primitive("cog-fc", &InferenceSCM::do_forward_chaining,
                            _inst); //eg. from scm shell (cog-fc (InheritanceLink (ConceptNode "cat")(ConceptNode "animal"))
    define_scheme_primitive("cog-bc", &InferenceSCM::do_backward_chaining,
                            _inst); //backward chaining
#endif
}

Handle InferenceSCM::do_forward_chaining(Handle h)
{
#ifdef HAVE_GUILE
    AtomSpace *as = SchemeSmob::ss_get_env_as("cog-fc");
    DefaultForwardChainerCB dfc(as);
    ForwardChainer fc(as);
    /**
     * Parse (cog-fc ListLink()) as forward chaining with Handle::UNDEFINED which  does
     * pattern matching on the atomspace using the rules declared in the config.A similar
     * functionality with the python version of the  forward chainer.
     */
    if (h->getType() == LIST_LINK and as->getIncoming(h).empty())
        fc.do_chain(dfc, Handle::UNDEFINED);
    else
        /** Does variable fulfillment forward chaining or forward chaining based on
         *  target node @param h.
         *  example (cog-fc (InheritanceLink (VariableNode "$X") (ConceptNode "Human")))
         *  finds all the matches for $X by first finding matching rules and then applying
         *  all of them using the pattern matcher.
         *  and (cog-fc (ConceptNode "Human")) will start forward chaining on the concept Human
         *  trying to generate inferences associated only with the conceptNode Human.
         */
        fc.do_chain(dfc, h);

    HandleSeq result = fc.get_chaining_result();
    return as->addLink(LIST_LINK, result);
#else
    return Handle::UNDEFINED;
#endif
}

Handle InferenceSCM::do_backward_chaining(Handle h)
{
#ifdef HAVE_GUILE
    AtomSpace *as = SchemeSmob::ss_get_env_as("cog-bc");

    JsonicControlPolicyParamLoader cpolicy_loader(JsonicControlPolicyParamLoader(as, "reasoning/RuleEngine/default_cpolicy.json"));
    cpolicy_loader.load_config();

    std::vector<Rule> rules;

    for (Rule* pr : cpolicy_loader.get_rules())
        rules.push_back(*pr);

    BackwardChainer bc(as, rules);
	bc.set_target(h);

	logger().debug("[BackwardChainer] Before do_chain");

    bc.do_full_chain();

	logger().debug("[BackwardChainer] After do_chain");
    map<Handle, UnorderedHandleSet> soln = bc.get_chaining_result();

    HandleSeq soln_list_link;
    for (auto it = soln.begin(); it != soln.end(); ++it) {
        HandleSeq hs;
        hs.push_back(it->first);
        hs.insert(hs.end(), it->second.begin(), it->second.end());

        soln_list_link.push_back(as->addLink(LIST_LINK, hs));
    }

    return as->addLink(LIST_LINK, soln_list_link);
#else
    return Handle::UNDEFINED;
#endif
}
