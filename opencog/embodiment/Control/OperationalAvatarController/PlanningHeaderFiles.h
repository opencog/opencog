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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110$1301 USA.
 */

// This file includes all the header files involve planning
// This file is to be included by the files invove planning to avoid the header files including each other
#ifndef _PLANNING_HEADER_FILES_H
#define _PLANNING_HEADER_FILES_H

#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParameter.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParamType.h>
#include <set>

#define ACCESS_DISTANCE "1.0"
#define CLOSED_DISTANCE "1.0"
#define PARAMETER_NUM 7

using namespace std;
using namespace opencog::pai;

namespace opencog {

extern opencog::pai::ParamValue opencog::pai::UNDEFINED_VALUE;

namespace oac {

//typedef opencog::pai::ActionParameter ActionParameter;
//typedef opencog::pai::ActionParamType ActionParamType;
//typedef opencog::pai::ParamValue ParamValue;

     // define the variables for rules
     // we have 6 kinds of typedef variant<string, Rotation, Vector, Entity, fuzzyInterval, fuzzyIntFloatInterval > ParamValue
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

//     const ParamValue access_distance;
//     const ParamValue SV_TRUE;
//     const ParamValue SV_FALSE;

//     const string bool_var[PARAMETER_NUM];
//     const string str_var[PARAMETER_NUM];
//     const string int_var[PARAMETER_NUM];
//     const string float_var[PARAMETER_NUM];
//     const pai::Vector vector_var[PARAMETER_NUM];
//     const pai::Entity entity_var[PARAMETER_NUM];
//     const pai::Entity entity_vartest;

     const string access_distance = ACCESS_DISTANCE;
     const string SV_TRUE = "true";
     const string SV_FALSE = "false";

      const string bool_var[] = { "$bool_var0","$bool_var1", "$bool_var2","$bool_var3", "$bool_var4","$bool_var5","$bool_var6"};
      const  string  str_var[] = { "$str_var0","$str_var1", "$str_var2","$str_var3", "$str_var4","$str_var5","$str_var6" };
      const  string  int_var[] = { "$int_var0","$int_var1", "$int_var2", "$int_var3", "$int_var4","$int_var5","$int_var6"};
      const  string  float_var[] = { "$float_var0","$float_var1","$float_var2","$float_var3","$float_var4","$float_var5","$float_var6"};

      const  opencog::pai::Vector vector_var[] = {
         opencog::pai::Vector(999999.00000,999999.00000,999999.00000),
         opencog::pai::Vector(999998.00000,999999.00000,999999.00000),
         opencog::pai::Vector(999997.00000,999999.00000,999999.00000),
         opencog::pai::Vector(999996.00000,999999.00000,999999.00000),
         opencog::pai::Vector(999995.00000,999999.00000,999999.00000),
         opencog::pai::Vector(999994.00000,999999.00000,999999.00000),
         opencog::pai::Vector(999993.00000,999999.00000,999999.00000) };

      const  opencog::pai::Entity entity_var[] = {
        opencog::pai::Entity("$entity_var0", "undefined"),
        opencog::pai::Entity("$entity_var1", "undefined"),
        opencog::pai::Entity("$entity_var2", "undefined"),
        opencog::pai::Entity("$entity_var3", "undefined"),
        opencog::pai::Entity("$entity_var4", "undefined"),
        opencog::pai::Entity("$entity_var5", "undefined"),
        opencog::pai::Entity("$entity_var6", "undefined"),
        };

     bool isAVariableNumeric(std::string var);

     opencog::pai::ActionParamTypeCode GetVariableType(std::string var);

}}
#endif
