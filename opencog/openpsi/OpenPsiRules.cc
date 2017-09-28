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

  return rule;
}
