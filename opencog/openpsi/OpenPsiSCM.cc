/*
 * OpenPsiSCM.cc
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

#include <opencog/guile/SchemePrimitive.h>

#include "OpenPsiImplicator.h"
#include "OpenPsiRules.h"

#include "OpenPsiSCM.h"

using namespace opencog;

OpenPsiSCM::OpenPsiSCM()
{
  static bool is_init = false;
  if (is_init) return;
  is_init = true;
  scm_with_guile(init_in_guile, this);
}

void OpenPsiSCM::init()
{
  define_scheme_primitive("psi-add-category", &OpenPsiSCM::add_category,
    this, "openpsi");

  define_scheme_primitive("psi-add-to-category", &OpenPsiSCM::add_to_category,
    this, "openpsi");

  define_scheme_primitive("psi-categories", &OpenPsiSCM::get_categories,
    this, "openpsi");

  define_scheme_primitive("psi-get-action", &OpenPsiSCM::get_action,
    this, "openpsi");

  define_scheme_primitive("psi-get-context", &OpenPsiSCM::get_context,
    this, "openpsi");

  define_scheme_primitive("psi-get-goal", &OpenPsiSCM::get_goal,
    this, "openpsi");

  define_scheme_primitive("psi-imply", &OpenPsiSCM::imply,
    this, "openpsi");

  define_scheme_primitive("psi-rule", &OpenPsiSCM::add_rule,
    this, "openpsi");

  define_scheme_primitive("psi-rule?", &OpenPsiSCM::is_rule,
    this, "openpsi");

  define_scheme_primitive("psi-satisfiable?", &OpenPsiSCM::is_satisfiable,
    this, "openpsi");
}

Handle OpenPsiSCM::add_category(const Handle& new_category)
{
  AtomSpace* as = SchemeSmob::ss_get_env_as("psi-add-category");
  OpenPsiRules rule_constructor(as);
  return rule_constructor.add_category(new_category);
}

Handle OpenPsiSCM::add_rule(const HandleSeq& context, const Handle& action,
  const Handle& goal, const TruthValuePtr stv, const Handle& category)
{
  AtomSpace* as = SchemeSmob::ss_get_env_as("psi-rule");
  // TODO: Should this be a singleton? What could be the issues that need
  // to be handled? How to handle multiple atomspace, maybe a singleton per
  // atomspace?
  OpenPsiRules rule_constructor(as);
  Handle rule = rule_constructor.add_rule(context, action, goal, stv);
  // TODO: Add to multiple categories using scheme rest list.
  rule_constructor.add_to_category(rule, category);
  return rule;
}

Handle OpenPsiSCM::add_to_category(const Handle& rule, const Handle& category)
{
  AtomSpace* as = SchemeSmob::ss_get_env_as("psi-add-to-category");
  OpenPsiRules rule_constructor(as);
  return rule_constructor.add_to_category(rule, category);
}

Handle OpenPsiSCM::get_action(const Handle& rule)
{
  return OpenPsiRules::get_action(rule);
}

HandleSeq& OpenPsiSCM::get_categories()
{
  return OpenPsiRules::get_categories();
}

HandleSeq& OpenPsiSCM::get_context(const Handle& rule)
{
  return OpenPsiRules::get_context(rule);
}

Handle OpenPsiSCM::get_goal(const Handle& rule)
{
  return OpenPsiRules::get_goal(rule);
}

Handle OpenPsiSCM::imply(const Handle& rule)
{
  AtomSpace* as = SchemeSmob::ss_get_env_as("psi-imply");
  OpenPsiImplicator implicator(as);
  return implicator.imply(rule);
}

bool OpenPsiSCM::is_rule(const Handle& rule)
{
  return OpenPsiRules::is_rule(rule);
}

TruthValuePtr OpenPsiSCM::is_satisfiable(const Handle& rule)
{
  AtomSpace *as = SchemeSmob::ss_get_env_as("psi-satisfiable?");
  OpenPsiImplicator implicator(as);
  return implicator.check_satisfiability(rule);
}

// ========================================================
// Boilerplate code.
// ========================================================
void* OpenPsiSCM::init_in_guile(void* self)
{
  scm_c_define_module("opencog openpsi", init_in_module, self);
  scm_c_use_module("opencog openpsi");
  return NULL;
}

void OpenPsiSCM::init_in_module(void* data)
{
  OpenPsiSCM* self = (OpenPsiSCM*) data;
  self->init();
}

void opencog_openpsi_init(void)
{
  static OpenPsiSCM openpsi;
}
