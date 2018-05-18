/*
 * OpenPsiImplicator.cc
 *
 * Copyright (C) 2017 MindCloud
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

#include <opencog/atoms/proto/NameServer.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/execution/Instantiator.h>

#include "OpenPsiImplicator.h"
#include "OpenPsiRules.h"

using namespace opencog;

OpenPsiImplicator::OpenPsiImplicator(AtomSpace* as) :
  InitiateSearchCB(as),
  DefaultPatternMatchCB(as),
  Satisfier(as)
{
  _action_executed = _as->add_node(PREDICATE_NODE, "action-executed");
}

bool OpenPsiImplicator::grounding(const HandleMap &var_soln,
                                  const HandleMap &term_soln)
{
  // TODO: Saperated component patterns aren't handled by this function
  // as PMCGroundings is used instead. Update to handle such cases.

  // The psi-rule weight calculations could be done here.
  _result = TruthValue::TRUE_TV();

  if (0 < var_soln.size()) {
    for( auto it = var_soln.begin(); it != var_soln.end(); ++it )
    {
      if(nameserver().isA(VARIABLE_NODE, (it->second)->get_type())) {
        return false;
      }
    }

    // TODO: If we are here it means the suggested groundings doesn't have
    // VariableNodes, and can be cached. This doesn't account for terms
    // that are under QuoteLink, or other similar type links. How should
    // such cases be handled?

    // Store the result in cache.
    _satisfiability_cache[_pattern_body] = var_soln;

    // NOTE: If a single grounding is found then why search for more? If there
    // is an issue with instantiating the implicand then there is an issue with
    // the declard relationship between the implicant and the implicand, aka
    // user-error.
    return true;
  } else if (not _have_variables) {
    // This happens when InitiateSearchCB::no_search has groundings -- when
    // there is only constant (probably evaluatable) but no variable in the
    // pattern
    _satisfiability_cache[_pattern_body] = var_soln;
    return true;
  } else {
    // TODO: This happens when InitiateSearchCB::no_search has groundings.
    // Cases for when this happens hasn't been tested yet. Explore the
    // behavior and find a better solution. For now, log it and continue
    // searching.
    logger().info("In %s: the following _pattern_body triggered "
      "InitiateSearchCB::no_search \n %s", __FUNCTION__ ,
      _pattern_body->to_string().c_str());

    return false;
  }
}

TruthValuePtr OpenPsiImplicator::check_satisfiability(const Handle& rule,
    OpenPsiRules& opr)
{
  // TODO:
  // Solve for multithreaded access. Create a rule class and lock
  // the rule when updating the cache.

  PatternLinkPtr query = opr.get_query(rule);
  Handle query_body = query->get_pattern().body;

  // Always update cache to clear any previous result.
  // TODO: Add cache per atomspace.
  _satisfiability_cache.erase(query_body);
  _pattern_seen.insert(query_body);

  query->satisfy(*this);

  // The boolean returned by query->satisfy isn't used because all
  // type of contexts haven't been handled by this callback yet.
  if (_satisfiability_cache.find(query_body) != _satisfiability_cache.end()) {
    return TruthValue::TRUE_TV();
  } else {
    return TruthValue::FALSE_TV();
  }
}

Handle OpenPsiImplicator::imply(const Handle& rule, OpenPsiRules& opr)
{
  PatternLinkPtr query = opr.get_query(rule);
  Handle query_body = query->get_pattern().body;

  if (_pattern_seen.find(query_body) == _pattern_seen.end())
  {
    throw RuntimeException(TRACE_INFO, "The openpsi rule should be checked "
      "for satisfiablity first." );
  }

  auto it = _satisfiability_cache.find(query_body);
  if (it != _satisfiability_cache.end())
  {
    Instantiator inst(_as);

    Handle result = \
      inst.instantiate(opr.get_action(rule), it->second, true);
    rule->setValue(_action_executed, ProtoAtomCast(TruthValue::TRUE_TV()));

    return result;
  } else {
    // NOTE: Trying to check for satisfiablity isn't done because it
    // is the responsibility of the action-selector for determining
    // what action is to be taken.
    rule->setValue(_action_executed, ProtoAtomCast(TruthValue::FALSE_TV()));
    return Handle::UNDEFINED;
  }
}

TruthValuePtr OpenPsiImplicator::was_action_executed(const Handle rule)
{
  return TruthValueCast(rule->getValue(_action_executed));
}

OpenPsiImplicator& opencog::openpsi_implicator(AtomSpace* as)
{
  static OpenPsiImplicator implicator(as);
  return implicator;
}
