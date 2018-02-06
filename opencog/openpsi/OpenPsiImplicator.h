/*
 * OpenPsiImplicator.h
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


#ifndef _OPENCOG_OPENPSI_IMPLICATOR_H
#define _OPENCOG_OPENPSI_IMPLICATOR_H

#include <opencog/atoms/pattern/PatternLink.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/openpsi/OpenPsiRules.h>

#include <opencog/query/Satisfier.h>

class OpenPsiImplicatorUTest;

namespace opencog
{

class OpenPsiImplicator: public virtual Satisfier
{
  // Needed for resetting private cache.
  // TODO Why would one need to reset during psi-loop?
  friend class ::OpenPsiImplicatorUTest;

public:
  OpenPsiImplicator(AtomSpace* as);

  /**
   * Return true if a single grounding has been found.
   */
  bool grounding(const HandleMap &var_soln,
                 const HandleMap &term_soln);

  /**
   * Returns TRUE_TV if there is grounding else returns FALSE_TV. If the
   * cache has entry for the context then TRUE_TV is returned.
   *
   * @param rule An openpsi rule.
   */
  TruthValuePtr check_satisfiability(const Handle& rule, OpenPsiRules& opr);

  /**
   * Instantiate the action of the given openpsi rule.
   *
   * @param rule An openpsi rule.
   * @return The handle to the grounded atom.
   */
  Handle imply(const Handle& rule, OpenPsiRules& opr);

private:
  /**
   * Cache used to store context with the variable groundings. Values
   * are not used to associate the variable groundings(the HandleMap) with
   * the query PatternLink, because doing so would require extra
   * computation that doesn't add any value.
   */
  std::map<Handle, HandleMap> _satisfiability_cache;

  /**
   * An empty map used for clearing cache entries, or to denote absence
   * of groundings.
   */
  const HandleMap _EMPTY_HANDLE_MAP;

  // Because two of the ancestor classes that this class inherites
  // from have _as variable.
  using DefaultPatternMatchCB::_as;
};

// This function is used to create a single static instance
OpenPsiImplicator& openpsi_implicator(AtomSpace* as);

}; // namespace opencog

#endif // _OPENCOG_OPENPSI_IMPLICATOR_H
