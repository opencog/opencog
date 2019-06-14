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

#include <opencog/atoms/atom_types/NameServer.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/execution/Instantiator.h>

#include "OpenPsiImplicator.h"
#include "OpenPsiSatisfier.h"
#include "OpenPsiRules.h"

using namespace opencog;

OpenPsiImplicator::OpenPsiImplicator(AtomSpace* as)
{
  _as = as;
  _action_executed = _as->add_node(PREDICATE_NODE, "action-executed");
}


TruthValuePtr OpenPsiImplicator::check_satisfiability(const Handle& rule,
    OpenPsiRules& opr)
{

  OpenPsiSatisfier sater(_as, this);
  return sater.check_satisfiability(rule, opr);
}

Handle OpenPsiImplicator::imply(const Handle& rule, OpenPsiRules& opr)
{
  OpenPsiSatisfier sater(_as, this);
  return sater.imply(rule, opr);
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
