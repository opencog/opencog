/*
 * PlanningHeaderFiles.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * Written by Shujing KE
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

// This file includes all the header files involve planning
// This file is to be included by the files invove planning to avoid the header files including each other
#ifndef _PLANNING_HEADER_FILES_H
#define _PLANNING_HEADER_FILES_H

#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParameter.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParamType.h>

namespace opencog { namespace oac {

typedef opencog::pai::ActionParameter StateVariable;
typedef opencog::pai::ActionParamType StateValuleType;
typedef opencog::pai::ParamValue StateValue;

}}
#endif
