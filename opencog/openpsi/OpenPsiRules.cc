/*
 * OpenPsiRules.cc
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

#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>

#include "OpenPsiRules.h"

using namespace opencog;

OpenPsiRules::OpenPsiRules(AtomSpace* as): _as(as)
{
  _action_executed = _as->add_node(PREDICATE_NODE, "action-executed");
}

Handle OpenPsiRules::add_rule(const HandleSeq& context, const Handle& action,
  const Handle& goal, const TruthValuePtr stv)
{
  // Add an AndLink of context and action which forms the
  // implicant of the psi-rule.
  HandleSeq temp_c =  context;
  temp_c.push_back(action);
  Handle hca = _as->add_link(AND_LINK, temp_c);

  // Add the psi-rule, set the truthvalue and default record of whether
  // the action of the rule was executed.
  Handle rule = _as->add_link(IMPLICATION_LINK, hca, goal);

  // No need to recreate the cache entry if it already exists.
  if (_psi_rules.count(rule)) return rule;

  rule->setTruthValue(stv);
  rule->setValue(_action_executed, ProtoAtomCast(TruthValue::FALSE_TV()));

  // Construct the query atom that is used to check satisfiablity. This is
  // done here for performance. If context has only one atom and it is a
  // SatisfactionLink(e.g. a ghost context) then it cast it because the
  // cast will be valid; else construct a PatternLink wrapping the context
  // in an AndLink.
  if ((1 == context.size()) and
    classserver().isA(SATISFACTION_LINK, context[0]->get_type())) {
      // This is for ghost.

      // Add to the index of rules.
      PatternLinkPtr query = PatternLinkCast(context[0]);
      _psi_rules[rule] = std::make_tuple(context, action, goal, query);
  } else {
    // This is for backward compatability.
    // TODO: Test thoroughly, or develop an alternative. See discussion
    // @ https://github.com/opencog/opencog/pull/2899 for what the
    // alternative might be.

    // Add to the index of rules.
    PatternLinkPtr query_body = \
      createPatternLink(Handle(createLink(context, AND_LINK)));
    _psi_rules[rule] = std::make_tuple(context, action, goal, query_body);
  }

  return rule;
}

bool OpenPsiRules::is_rule(const Handle& rule)
{
  return _psi_rules.count(rule);
}

Handle OpenPsiRules::add_category(const Handle& new_category)
{
  _as->add_link(INHERITANCE_LINK, new_category, _psi_category);
  if(not(_category_index.count(new_category))) {
    _category_index[new_category] = {};
  }

  return new_category;
}

Handle OpenPsiRules::add_to_category(const Handle& rule, const Handle& category)
{
  _as->add_link(MEMBER_LINK, rule, category);
  // Add the category just in case it hasn't been declared.
  // TODO But why make the add_category public then?
  add_category(category);
  _category_index[category].insert(rule);

  return rule;
}

HandleSeq& OpenPsiRules::get_categories()
{
  HandleSeq* categories = new HandleSeq();
  for(auto i : _category_index) {
    categories->emplace_back(i.first);
  }

  // TODO: Should this be a shared ptr to avoid memory leak?
  return *categories;
}

HandleSeq& OpenPsiRules::get_context(const Handle rule)
{
  if(_psi_rules.count(rule)) {
    return std::get<0>(_psi_rules[rule]);
  } else {
    // TODO: Should this be a shared ptr to avoid memory leak?
    HandleSeq* hs = new HandleSeq();
    return *hs;
  }
}

Handle OpenPsiRules::get_action(const Handle rule)
{
  if(_psi_rules.count(rule)) {
    return std::get<1>(_psi_rules[rule]);
  } else {
    return Handle::UNDEFINED;
  }
}

Handle OpenPsiRules::get_goal(const Handle rule)
{
  if(_psi_rules.count(rule)) {
    return std::get<2>(_psi_rules[rule]);
  } else {
    return Handle::UNDEFINED;
  }
}

PatternLinkPtr OpenPsiRules::get_query(const Handle rule)
{
  if(_psi_rules.count(rule)) {
    return std::get<3>(_psi_rules[rule]);
  } else {
    return PatternLinkCast(Handle::UNDEFINED);
  }
}

OpenPsiRules& opencog::openpsi_cache(AtomSpace* as)
{
  // To handle multiple atomspaces maybe a static vector of OpenPsiRules
  //  maybe used.
  static OpenPsiRules cache(as);
  return cache;
}
