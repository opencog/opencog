/*
 * OpenPsiRules.h
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

#ifndef _OPENCOG_OPENPSI_RULES_H
#define _OPENCOG_OPENPSI_RULES_H

#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{

class OpenPsiRules
{
public:
  OpenPsiRules(AtomSpace* as);

  /**
   * Add a rule to the atomspace and the psi-rule index.
   * @return An ImplicationLink that forms a psi-rule. The structure
   *  of the rule is
   *    (ImplicationLink TV
   *      (SequentialAndLink
   *        context
   *        action)
   *      goal)
   */
  Handle add_rule(const HandleSeq& context, const Handle& action,
    const Handle& goal, const TruthValuePtr stv, const Handle& demand);

private:
  /**
   * The structure of the tuple is (context, action, goal).
   */
  typedef std::tuple<HandleSeq, Handle, Handle> PsiTuple;

  /**
   * This is a index with the keys being the psi-rules and the corresponding
   * value being a tuple of its three components. The intention is to minimize
   * the computing required for getting the different component of a rule.
   */
  static std::map<Handle, PsiTuple> _psi_rules;

  // TODO: Using names that are prefixed with "OpenPsi: " might be a bad idea,
  // because it might hinder interoperability with other components that
  // expect an explicit ontological representation. For historic reasons we
  // continue using such convention but should be replaces with graph that
  // represent the relationships. That way it would be possible to answer
  // questions about the system the nlp pipeline.

  /**
   * Node used to declare an action.
   */
  static Handle _psi_action;

  /**
   * Node used to declare a goal.
   */
  // static Handle _psi_goal;

  AtomSpace* _as;
};

} // namespace opencog

#endif  // _OPENCOG_OPENPSI_RULES_H
