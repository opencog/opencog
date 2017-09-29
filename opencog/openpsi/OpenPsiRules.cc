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

#include <opencog/atoms/base/Node.h>

#include "OpenPsiRules.h"

using namespace opencog;

std::map<Handle, OpenPsiRules::PsiTuple> OpenPsiRules::_psi_rules = {};
Handle OpenPsiRules::_psi_action = \
  Handle(createNode(CONCEPT_NODE, "OpenPsi: action"));
Handle OpenPsiRules::_psi_goal = \
  Handle(createNode(CONCEPT_NODE, "OpenPsi: goal"));
Handle OpenPsiRules::_psi_demand = \
  Handle(createNode(CONCEPT_NODE, "OpenPsi: demand"));

OpenPsiRules::OpenPsiRules(AtomSpace* as): _as(as)
{}

Handle OpenPsiRules::add_rule(const HandleSeq& context, const Handle& action,
  const Handle& goal, const TruthValuePtr stv, const Handle& demand)
{
  // Declare as an openpsi action if it hasn't already been declared.
  _as->add_link(MEMBER_LINK, action, _psi_action);

  // Add a SequentialAndLink of context and action which forms the
  // implicant of the psi-rule.
  HandleSeq temp_c =  context;
  temp_c.push_back(action);
  Handle hca = _as->add_link(SEQUENTIAL_AND_LINK, temp_c);

  // Add the psi-rule, set the truthvalue and add to a demand.
  Handle rule = _as->add_link(IMPLICATION_LINK, hca, goal);
  rule->setTruthValue(stv);
  _as->add_link(MEMBER_LINK, rule, demand);

  // Add to the index of rules.
  _psi_rules[rule] = std::make_tuple(context, action, goal);

  return rule;
}

Handle OpenPsiRules::add_tag(const Handle tag_type_node,
                             const std::string& name)
{
  Handle tag = _as->add_node(CONCEPT_NODE, name);
  _as->add_link(INHERITANCE_LINK, tag , tag_type_node);
  return tag;
}

HandleSeq OpenPsiRules::psi_get_context(const Handle rule)
{
  if(_psi_rules.count(rule)) {
    return std::get<0>(_psi_rules[rule]);
  } else {
    return {};
  }
}

Handle OpenPsiRules::psi_get_action(const Handle rule)
{
  if(_psi_rules.count(rule)) {
    return std::get<1>(_psi_rules[rule]);
  } else {
    return Handle::UNDEFINED;
  }
}

Handle OpenPsiRules::psi_get_goal(const Handle rule)
{
  if(_psi_rules.count(rule)) {
    return std::get<2>(_psi_rules[rule]);
  } else {
    return Handle::UNDEFINED;
  }
}
