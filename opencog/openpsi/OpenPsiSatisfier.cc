/*
 * OpenPsiSatisfier.cc
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

OpenPsiSatisfier::OpenPsiSatisfier(AtomSpace* as,
                                   OpenPsiImplicator* implicator) :
  Satisfier(as)
{
  _implicator = implicator;
}

bool OpenPsiSatisfier::grounding(const HandleMap &var_soln,
                                  const HandleMap &term_soln)
{
  // TODO: Saperated component patterns aren't handled by this function
  // as PMCGroundings is used instead. Update to handle such cases.

  // The psi-rule weight calculations could be done here.
  _result = true;

  if (0 < var_soln.size()) {
    for( auto it = var_soln.begin(); it != var_soln.end(); ++it )
    {
      if(nameserver().isA((it->second)->get_type(), VARIABLE_NODE)) {
        return false;
      }
    }

    // TODO: If we are here it means the suggested groundings doesn't have
    // VariableNodes, and can be cached. This doesn't account for terms
    // that are under QuoteLink, or other similar type links. How should
    // such cases be handled?

    // Store the result in cache.
    _implicator -> _satisfiability_cache[_pattern_body] = var_soln;

    // NOTE: If a single grounding is found then why search for more? If there
    // is an issue with instantiating the implicand then there is an issue with
    // the declard relationship between the implicant and the implicand, aka
    // user-error.
    return true;
  } else if (not _have_variables) {
    // This happens when InitiateSearchCB::no_search has groundings -- when
    // there is only constant (probably evaluatable) but no variable in the
    // pattern
    _implicator -> _satisfiability_cache[_pattern_body] = var_soln;
    return true;
  } else {
    // TODO: This happens when InitiateSearchCB::no_search has groundings.
    // Cases for when this happens hasn't been tested yet. Explore the
    // behavior and find a better solution. For now, log it and continue
    // searching.
    logger().info("In %s: the following _pattern_body triggered "
      "InitiateSearchCB::no_search \n %s", __FUNCTION__ ,
      _pattern_body->to_string().c_str());

    return false;
  }
}
