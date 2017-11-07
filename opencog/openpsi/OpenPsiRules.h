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

#include <opencog/atoms/pattern/PatternLink.h>
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
    const Handle& goal, const TruthValuePtr stv);

  /**
   * @param rule A psi-rule.
   * @return Context of the given psi-rule.
   */
  static HandleSeq& get_context(const Handle rule);

  /**
   * @param rule A psi-rule.
   * @return Action of the given psi-rule.
   */
  static Handle get_action(const Handle rule);

  /**
   * @param rule A psi-rule.
   * @return Goal of the given psi-rule.
   */
  static Handle get_goal(const Handle rule);

  /**
   * @param rule A psi-rule.
   * @return Query atom used to check if the context of the given psi-rule is
   *  satisfiable or not.
   */
  static PatternLinkPtr get_query(const Handle rule);

  /**
   * Declare a new category by adding the following structured atom into the
   * atomspace
   *      (InheritanceLink (ConceptNode "name") category_type_node)
   *
   * @param tag_type_node The Node from which the new tag node inherites from.
   * @param name The name of the ConceptNode that is going to be added
   * @return ConceptNode that represents the category.
   */
   // TODO: Should the categories be saved so as to check when using
   // add_to_category?
  Handle add_category(const Handle tag_type_node, const std::string& name);

  /**
   * Add a node to a category. The representation is as follows
   *    (MemberLink node category_node)
   * Having this enables the possiblity of easily redefining the
   * representation.
   *
   * @param node A node to be categorized.
   * @param category An atom that reprsents the category.
   */
  void add_to_category(const Handle node, const Handle category);

private:
  /**
   * The structure of the tuple is (context, action, goal, query),
   * where queryis a PatternLink that isn't added to the atomspace, and
   * is used to check if the rule is satisfiable.
   */
  // TODO Should these entries be a member of Rules class?
  typedef std::tuple<HandleSeq, Handle, Handle, PatternLinkPtr> PsiTuple;

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

  AtomSpace* _as;
};

} // namespace opencog

#endif  // _OPENCOG_OPENPSI_RULES_H
