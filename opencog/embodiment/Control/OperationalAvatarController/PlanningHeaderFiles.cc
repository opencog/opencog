/*
 * PlanningHeaderFiles.cc
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

#include "PlanningHeaderFiles.h"


using namespace opencog::oac;

const StateValue access_distance = ACCESS_DISTANCE;
const StateValue SV_TRUE = "true";
const StateValue SV_FALSE = "false";

 const string bool_var[] = { "$bool_var0","$bool_var1", "$bool_var2","$bool_var3", "$bool_var4","$bool_var5","$bool_var6"};
 const  string  str_var[] = { "$str_var0","$str_var1", "$str_var2","$str_var3", "$str_var4","$str_var5","$str_var6" };
 const  string  int_var[] = { "$int_var0","$int_var1", "$int_var2", "$int_var3", "$int_var4","$int_var5","$int_var6"};
 const  string  float_var[] = { "$float_var0","$float_var1","$float_var2","$float_var3","$float_var4","$float_var5","$float_var6"};
 const  opencog::pai::Vector vector_var[] = {
    opencog::pai::Vector(999999.00000,999999.00000,999999.00000),
    opencog::pai::Vector(999999.01000,999999.01000,999999.01000),
    opencog::pai::Vector(999999.02000,999999.02000,999999.02000),
    opencog::pai::Vector(999999.03000,999999.03000,999999.03000),
    opencog::pai::Vector(999999.04000,999999.04000,999999.04000),
    opencog::pai::Vector(999999.05000,999999.05000,999999.05000),
    opencog::pai::Vector(999999.06000,999999.06000,999999.06000) };

 const  opencog::pai::Entity entity_var[] = {
   opencog::pai::Entity("$entity_var0", "undefined"),
   opencog::pai::Entity("$entity_var1", "undefined"),
   opencog::pai::Entity("$entity_var2", "undefined"),
   opencog::pai::Entity("$entity_var3", "undefined"),
   opencog::pai::Entity("$entity_var4", "undefined"),
   opencog::pai::Entity("$entity_var5", "undefined"),
   opencog::pai::Entity("$entity_var6", "undefined"),
   };

 bool opencog::oac::isAVariableNumeric(std::string var)
 {
     int pos = var.find("$int_var");
     if (pos != std::string::npos)
         return true;

     pos = var.find("$float_var");
     if (pos != std::string::npos)
         return true;

     pos = var.find('(');
     if (pos != std::string::npos)
     {
         pos = var.find("$entity_var");
         if (pos == std::string::npos)
             return true;
         else
             return false;
     }

     return false;

 }

