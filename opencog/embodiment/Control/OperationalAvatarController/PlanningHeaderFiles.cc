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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110$1301 USA.
 */

#include "PlanningHeaderFiles.h"


using namespace opencog::oac;


 bool opencog::oac::isAVariableNumeric(std::string var)
 {
     std::size_t pos = var.find("$int_var");
     if ((size_t)pos != std::string::npos)
         return true;

     pos = var.find("$float_var");
     if ((size_t)pos != std::string::npos)
         return true;

     pos = var.find('(');
     if ((size_t)pos != std::string::npos)
     {
         pos = var.find("$entity_var");
         if ((size_t)pos == std::string::npos)
             return true;
         else
             return false;
     }

     return false;

 }

 opencog::pai::ActionParamTypeCode opencog::oac::GetVariableType(std::string var)
 {
     std::size_t pos = var.find("$int_var");
     if ((size_t)pos != std::string::npos)
         return opencog::pai::INT_CODE;

     pos = var.find("$float_var");
     if ((size_t)pos != std::string::npos)
         return opencog::pai::FLOAT_CODE;

     pos = var.find("$bool_var");
     if ((size_t)pos != std::string::npos)
         return opencog::pai::BOOLEAN_CODE;

     pos = var.find("$str_var");
     if ((size_t)pos != std::string::npos)
         return opencog::pai::STRING_CODE;

     pos = var.find('(');
     if ((size_t)pos != std::string::npos)
     {
         pos = var.find("$entity_var");
         if ((size_t)pos != std::string::npos)
             return opencog::pai::ENTITY_CODE;
         else
             return opencog::pai::VECTOR_CODE;

         // todo:currently we don't really try fuzzy_int, fuzzy_float and Rotation
     }

     return opencog::pai::NUMBER_OF_ACTION_PARAM_TYPES;
 }

