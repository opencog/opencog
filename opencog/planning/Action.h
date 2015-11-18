/*
 * @file opencog/planning/Action.h
 * @author Amen Belayneh <amenbelayneh@gmail.com> November 2015
 *
 * Copyright (C) 2015 OpenCog Foundation
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

#ifndef _OPENCOG_PLANNING_ACTION_H
#define _OPENCOG_PLANNING_ACTION_H

#include <opencog/rule-engine/Rule.h>

namespace opencog {
/** \addtogroup planning
 * @{
 */
class Action : public Rule
{
public:
    Action(Handle member_l);
    ~Action();

private:

    // The implicant of the pattern, which is also called  "context"
    // is divided into two
    // terms that have virtual clauses and terms that don't.
    HandleSeq _virtual_terms ;
    HandleSeq _fixed_terms;

    // Function for helping construct
    void init();

}; // class Action

/** @}*/
} // namespace opencog

#endif // _OPENCOG_PLANNING_ACTION_H
