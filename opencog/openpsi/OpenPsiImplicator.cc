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


#include "OpenPsiImplicator.h"

using namespace opencog;

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

  // NOTE: If a single grounding is found then why search for more? If there
  // is an issue with instantiating the implicand then there is an issue with
  // the implicand. Satisfier::grounding does exaustive search so this should
  // theoretically speed psi-loop.
  return true;
}

TruthValuePtr OpenPsiImplicator::check_satisfiability(
  const Handle& himplication)
{
  PatternLinkPtr plp =  createPatternLink(himplication->getOutgoingAtom(0));
  plp->satisfy(*this);

  return _result;
}
