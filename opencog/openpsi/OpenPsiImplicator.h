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

namespace opencog
{

class OpenPsiImplicator: public virtual Satisfier
{
public:
  OpenPsiImplicator(AtomSpace* as);

  /**
   * Return true if a single grounding has been found.
   */
  bool grounding(const HandleMap &var_soln,
                 const HandleMap &term_soln);

  /**
   * Returns TRUE_TV if there is grounding else returns FALSE_TV.
   */
  TruthValuePtr check_satisfiability(const Handle& himplication);
};

}; // namespace opencog

#endif // _OPENCOG_OPENPSI_IMPLICATOR_H
