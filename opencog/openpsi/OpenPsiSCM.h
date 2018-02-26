/*
 * OpenPsiSCM.h
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

#ifndef _OPENCOG_OPENPSI_SCM_H
#define _OPENCOG_OPENPSI_SCM_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/TruthValue.h>

namespace opencog
{

class OpenPsiSCM
{
public:
  OpenPsiSCM();

private:
  Handle add_category(const Handle& new_category);

  /**
   * Add psi-rule.
   *
   * @return An implication link which is a psi-rule.
   */
  Handle add_rule(const HandleSeq& context, const Handle& action,
    const Handle& goal, const TruthValuePtr stv, const Handle& category);

  Handle add_to_category(const Handle& rule, const Handle& category);

  /**
   * Get the action of the given rule.
   *
   * @param rule A psi-rule.
   * @return A handle of the action atom.
   */
  Handle get_action(const Handle& rule);

  /**
   * A wrapper around OpenPsiRules::get_categories
   *
   * @return A vector of Handles that represent the categories.
   */
  HandleSeq& get_categories();

  /**
   * Get the context of the given rule.
   *
   * @param rule A psi-rule.
   * @return A vector of atoms that form the context of the given rule.
   */
  HandleSeq& get_context(const Handle& rule);

  /**
   * Get the goal of the given rule.
   *
   * @param rule A psi-rule.
   * @return A handle of the goal atom.
   */
  Handle get_goal(const Handle& rule);

  /**
   * Instantiates the action of the psi-rule if their is an entry in
   * the cache for the groundings of its context.
   *
   * @param rule A psi-rule.
   */
  Handle imply(const Handle& rule);

  /**
   * Check if the instation of the acation worked on the last run of
   * imply(rule).
   *
   * @param rule A psi-rule
   */
   TruthValuePtr was_action_executed(const Handle& rule);

  /**
   * A wrapper around OpenPsiRules::is_rule.
   *
   * @return true if the rule is in the index, false other wise.
   */
  bool is_rule(const Handle& rule);

  /**
   * Returns TRUE_TV or FALSE_TV depending on whether the context of the
   * given psi-rule is satisfiable or not.
   *
   * @param rule A psi-rule.
   */
  TruthValuePtr is_satisfiable(const Handle& rule);

  // ========================================================
  // Boilerplate code.
  // ========================================================
  /**
   * Init function for using with scm_with_guile. It creates the
   * openpsi scheme module and uses it by default.
   *
   * @param self pointer to the OpenPsiSCM object
   */
  static void* init_in_guile(void*);

  /**
   * The main function for defining stuff in the openpsi scheme module.
   *
   * @param data  pointer to the OpenPsiSCM object
   */
  static void init_in_module(void*);

  /**
   * The main init function for the OpenPsiSCM object.
   */
  void init(void);
};

} // namespace opencog

extern "C" {
void opencog_openpsi_init(void);
};

#endif  // _OPENCOG_OPENPSI_SCM_H
