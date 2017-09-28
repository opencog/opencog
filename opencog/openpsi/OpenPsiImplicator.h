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
   */
  TruthValuePtr check_satisfiability(const Handle& himplication);

  /**
   * Instantiate the implicand of the given ImplicationLink.
   *
   * @param himplication Handle to the ImplicationLink.
   * @param vars Map from variables to their groundings.
   * @return The handle to the grounded atom.
   */
  Handle imply(const Handle& himplication);

private:
  /**
   * Cache used to store context with the variable groundings.
   */
  static std::map<Handle, HandleMap> _satisfiability_cache;

  /**
   * Used to signal whether cache should be updated or not. By default the
   * cache is updated. It isn't static because one might only want to
   * use the cache for a subset of contexts.
   */
  bool _update_cache = true;

  // Because two of the ancestor classes that this class inherites
  // from have _as variable.
  using DefaultPatternMatchCB::_as;
};

}; // namespace opencog

#endif // _OPENCOG_OPENPSI_IMPLICATOR_H
