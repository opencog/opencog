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

#include <opencog/atoms/execution/Instantiator.h>

#include "OpenPsiImplicator.h"

using namespace opencog;

std::map<Handle, HandleMap> OpenPsiImplicator::_satisfiability_cache = {};

OpenPsiImplicator::OpenPsiImplicator(AtomSpace* as) :
  InitiateSearchCB(as),
  DefaultPatternMatchCB(as),
  Satisfier(as)
{}

bool OpenPsiImplicator::grounding(const HandleMap &var_soln,
                                  const HandleMap &term_soln)
{
  // The psi-rule weight calculations could be done here.
  _result = TruthValue::TRUE_TV();

  // Store the result in cache.
  _satisfiability_cache[_pattern_body] = var_soln;

  // NOTE: If a single grounding is found then why search for more? If there
  // is an issue with instantiating the implicand then there is an issue with
  // the implicand. Satisfier::grounding does exaustive search so this should
  // theoretically speed psi-loop.
  return true;
}

TruthValuePtr OpenPsiImplicator::check_satisfiability(
  const Handle& himplication)
{
  // TODO: Replace this with the context, as psi-rule is
  // (context + action ->goal)
  Handle context = himplication->getOutgoingAtom(0);

  // TODO: How to prevent stale cache?
  if (_update_cache) {
    PatternLinkPtr plp =  createPatternLink(context);
    plp->satisfy(*this);
  }

  if (_satisfiability_cache.count(context)) {
    return TruthValue::TRUE_TV();
  } else {
    return TruthValue::FALSE_TV();
  }
}

Handle OpenPsiImplicator::imply(const Handle& himplication)
{
  // TODO: Replace this with the action, as psi-rule is
  // (context + action ->goal)
  Handle context = himplication->getOutgoingAtom(0);
  Instantiator inst(_as);

  if (_satisfiability_cache.count(context)) {
    return inst.instantiate(himplication->getOutgoingAtom(1),
              _satisfiability_cache.at(context), true);
  } else {
    // NOTE: Trying to check for satisfiablity isn't done because it
    // is the responsibility of the action-selector for determining
    // what action is to be taken.
    return Handle::UNDEFINED;
  }
}
