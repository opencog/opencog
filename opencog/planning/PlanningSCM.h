/*
 * @file opencog/planning/PlanningSCM.h
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

#ifndef _OPENCOG_PLANNING_PLANNINGSCM_H
#define _OPENCOG_PLANNING_PLANNINGSCM_H

#include <opencog/guile/SchemeModule.h>
#include <opencog/guile/SchemePrimitive.h>


namespace opencog
{
/** \addtogroup planning
 * @{
 */

class PlanningSCM : public ModuleWrap
{
public:
    PlanningSCM();

protected:
    virtual void init();
    HandleSeq select_actions(Handle rbs);
};

/** @}*/
} // namespace opencog

extern "C" {
// This function will be called to initialize the module.
void opencog_planning_init(void);
};

#endif // _OPENCOG_PLANNING_PLANNINGSCM_H
