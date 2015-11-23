/*
 * @file opencog/planning/ActionSelector.h
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

 #ifndef _OPENCOG_PLANNING_ACTION_SELECTOR_H
 #define _OPENCOG_PLANNING_ACTION_SELECTOR_H

 #include <vector>


 #include "Action.h"

 namespace opencog
 {

 class ActionSelector
 {
 public:
     ActionSelector(AtomSpace& as, Handle rbs);
     ~ActionSelector();

     static const std::string action_rbs_name;

 private:
     AtomSpace& _as;

     // An action is a URE rule that inherits from
     // (ConceptNode "OpenCog: Action").
     std::vector<Action> _actions;

     // Initial rulebase
     Handle _rbs;
     HandleSeq fetch_actions();
 }; // class ActionSelector

 } // namespace opencog

 #endif  // _OPENCOG_PLANNING_UTILITIES_H
