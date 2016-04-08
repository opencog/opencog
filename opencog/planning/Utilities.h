/*
 * @file opencog/planning/Utilities.h
 *
 * Copyright (C) 2016 OpenCog Foundation
 * All Rights Reserved
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

#ifndef _OPENCOG_PLANNING_UTILITIES_H
#define _OPENCOG_PLANNING_UTILITIES_H

#include <opencog/atoms/base/Handle.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{
/** \addtogroup planning
 * @{
 */

    /**
     * Gets the atoms that inherit from (ConceptNode "opencog: action") and
     *
     * @param as The atomspace that is to be searched for actions.
     * @param rbs The rulebase
     * @return HandleSeq of actions.
     */
    HandleSeq fetch_actions(AtomSpace& as);

/** @}*/
} // namespace opencog

#endif // _OPENCOG_PLANNING_UTILITIES_H
