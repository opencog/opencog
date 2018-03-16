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
   *      (AndLink
   *        context
   *        action)
   *      goal)
   */
  Handle add_rule(const HandleSeq& context, const Handle& action,
    const Handle& goal, const TruthValuePtr stv);

  /**
   * It checks if the rule passed is cached in the index. A valid
   * structured rule declared in the atomspace but not indexed will
   * not be considered as a rule.
   *
   * @return true if the rule is in the index, false other wise.
   */
   // NOTE: An approach where in first the rules are declared then indexed
   // by searching the atomspace, similar to opencog::UREConfig, can
   // be followed. But, this way the developer/agents/modules will have
   // to make the choice of which declarations to process using this module,
   // there by possibly helping in performance.
  bool is_rule(const Handle& rule);

  /**
   * Returns all the categories that were added using add_to_category.
   *
   * @return A vector of Handles that represent the categories.
   */
  HandleSeq& get_categories();

  /**
   * @param rule A psi-rule.
   * @return Context of the given psi-rule.
   */
  HandleSeq& get_context(const Handle rule);

  /**
   * @param rule A psi-rule.
   * @return Action of the given psi-rule.
   */
  Handle get_action(const Handle rule);

  /**
   * @param rule A psi-rule.
   * @return Goal of the given psi-rule.
   */
  Handle get_goal(const Handle rule);

  /**
   * @param rule A psi-rule.
   * @return Query atom used to check if the context of the given psi-rule is
   *  satisfiable or not.
   */
  PatternLinkPtr get_query(const Handle rule);

  /**
   * Declare a new category by adding the following structured atom into the
   * atomspace
   *      (Inheritance new_category (Concept "OpenPsi: category"))
   *
   * Such categorization is helpful in defining custom behaviors per category.
   *
   * @param new_category The node reprsenting the new category.
   * @return ConceptNode that represents the category.
   */
   // TODO:add predicate to check for membership of category.
  Handle add_category(const Handle& new_category);

  /**
   * Add a node to a category. The representation is as follows
   *    (MemberLink rule category)
   * Having this enables the possiblity of easily redefining the
   * representation.
   *
   * @param rule A rule to be categorized.
   * @param category An atom that represents the category.
   * @return The rule that was passed in.
   */
  Handle add_to_category(const Handle& rule, const Handle& category);

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
  std::map<Handle, PsiTuple> _psi_rules;

  // TODO: Using names that are prefixed with "OpenPsi: " might be a bad idea,
  // because it might hinder interoperability with other components that
  // expect an explicit ontological representation. For historic reasons we
  // continue using such convention but should be replaced with graph that
  // represent the relationships. That way it would be possible to answer
  // questions about the system using the nlp pipeline.

  /**
   * Maps from category nodes to Set of rules in that category.
   * It is static because the assumption is the recategorization of rules
   * doesn't happen dynamically, for now, i.e., when there is no learning
   * taking place.
   */
  std::map<Handle, HandleSet> _category_index;

  /**
   * Node used to declare a category.
   */
  Handle _psi_category;

  // Predicate used to set a value on whether an action was executed or not
  Handle _action_executed;

  AtomSpace* _as;
};

// This function is used to create a single static instance
OpenPsiRules& openpsi_cache(AtomSpace* as);

} // namespace opencog

#endif  // _OPENCOG_OPENPSI_RULES_H
