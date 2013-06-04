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
#include <set>

#define ACCESS_DISTANCE "2.1"
#define CLOSED_DISTANCE "0.9"
#define PARAMETER_NUM 7

using namespace std;

namespace opencog {

extern opencog::pai::ParamValue opencog::pai::UNDEFINED_VALUE;

namespace oac {

typedef opencog::pai::ActionParameter StateVariable;
typedef opencog::pai::ActionParamType StateValuleType;
typedef opencog::pai::ParamValue StateValue;

     // define the variables for rules
     // we have 6 kinds of typedef variant<string, Rotation, Vector, Entity, fuzzyInterval, fuzzyIntFloatInterval > StateValue
     /*
     BOOLEAN_CODE,
     INT_CODE,
     FLOAT_CODE,
     STRING_CODE,
     VECTOR_CODE,
     ROTATION_CODE,
     ENTITY_CODE,
     FUZZY_INTERVAL_INT_CODE,
     FUZZY_INTERVAL_FLOAT_CODE,*/

     const StateValue access_distance;
     const StateValue SV_TRUE;
     const StateValue SV_FALSE;

     const string bool_var[PARAMETER_NUM];
     const string str_var[PARAMETER_NUM];
     const string int_var[PARAMETER_NUM];
     const string float_var[PARAMETER_NUM];
     const pai::Vector vector_var[PARAMETER_NUM];
     const pai::Entity entity_var[PARAMETER_NUM];

     bool isAVariableNumeric(std::string var);

}}
#endif
