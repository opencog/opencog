/*
 * opencog/comboreduct/combo/procedure_repository.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _COMBO_PROCEDURE_REPOSITORY_H
#define _COMBO_PROCEDURE_REPOSITORY_H

#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

#include "procedure_call.h"
#include "vertex.h"

#include <set>
#include <map>

#define LINE_CHAR_MAX 4096

namespace opencog { namespace combo {

  typedef std::set<const procedure_call_base*> procedure_call_set;
  typedef procedure_call_set::const_iterator procedure_call_set_const_it;
  typedef procedure_call_set::iterator procedure_call_set_it;
  
  //def for procedure_repository

  //the following map associates procedure name and index in the proc_repo
  typedef std::map<std::string, procedure_call_base*> str_proc_map;
  typedef str_proc_map::const_iterator str_proc_map_const_it;
  typedef str_proc_map::iterator str_proc_map_it;

  //that vector contains an ordered set of strongly connected components
  //ordered according to procedure_call dependency
  //for instance if A uses B, and A and C are mutually dependent then
  //they have the following strongly_connected_list :
  //[{B}, {A, C}]
  //So the type checker follows this order to infer the types and uses
  //contextual inference when there is multual dependency
  typedef std::vector<procedure_call_set> strongly_connected_components;
  typedef strongly_connected_components::iterator
    strongly_connected_components_it;
  typedef strongly_connected_components::const_iterator
    strongly_connected_components_const_it;

  class procedure_repository {
  protected:
    str_proc_map _repo;

    strongly_connected_components _ordered_scc;

    //for all definite_objects in tr, check if the string matches the name
    //of a procedure_call and replace it by it if so
    void instantiate_procedure_calls(combo_tree& tr,
                                     bool warn_on_definite_object=false) const;

    //return a set of all procedure_call contained in a given procedure_call
    std::set<const procedure_call_base*> procedure_call_dependencies(const procedure_call_base* pc) const;

    //return a set of all procedure_call contained in a given set of 
    //procedure_call. It does not return reflexive dependency
    //that is the given procedure_calls are substracted from the result
    std::set<const procedure_call_base*> procedure_call_dependencies(const std::set<const procedure_call_base*>& pcs) const;

    //build the graph of dependency of the repo and generate the
    //associated strongly_connected_components
    void generate_and_order_strongly_connected_components();

  public:
    //each new procedure instanciated using procedure_call_base constructor
    //is added into the repository
    //If a procedure with same name is already present and is different in
    //content then an assert is raised
    void add(procedure_call_base* pc);

    //remove the procedure of a given name if any
    //WARNING : if other procedure were dependent on it
    //then they would now point toward an invalid procedure_call
    //that maybe a source of bug
    void remove(const std::string& name);

    //function allowing to retreive a procedure based on it's name
    //if no procedure corresponds to that name then it returns a NULL pointer
    procedure_call instance(const std::string& name) const;

    //return true if the repository contains a procedure with the given name
    bool does_contain(const std::string& name) const;

    //apply instantiate_procedure_calls on the entire repository
    void instantiate_procedure_calls(bool warn_on_definite_object=false);

    //check and infer the types of all procedure in the repository,
    //returns false if exists type error
    //the procedures with type error will have ill_formed types
    bool infer_types_repo();

    //clear
    void clear();

    //output stream method
    std::ostream& toStream(std::ostream& out, bool with_type = false) const;

    //for debugging print the repository on the standard output
    void print(bool with_type = false) const;
  };
  
template<class BUILTIN_ACTION,
	 class PERCEPTION,
	 class ACTION_SYMBOL,
	 class INDEFINITE_OBJECT>
unsigned int load_procedure_repository(std::istream& in,
                                       combo::procedure_repository& pr,
                                       bool type_checking = false) {
    unsigned int n = 0;
    
    while (in.good()) {
        while (in.peek()==' ' || in.peek()=='\n' || in.peek()=='\t')
            in.get();
        
        if(in.peek()=='#') { // a comment line
            char tmp[LINE_CHAR_MAX];
            in.getline(tmp,LINE_CHAR_MAX);
            continue;
        }
        
        procedure_call pc = load_procedure_call<BUILTIN_ACTION, PERCEPTION,
                                                ACTION_SYMBOL, INDEFINITE_OBJECT>(in, false);
    
        if (!in.good()){
            break;
        }
    
        if(pc) {
            pr.add(const_cast<procedure_call_base*>(pc));
            ++n;
            logger().fine("procedure_repository - Loaded '%s' with arity '%d'.", 
                          pc->get_name().c_str(), pc->arity());
            
        } else {
            logger().error("procedure_repository - Error parsing combo function.");
        }
    }
    //doing the resolution and type checking here
    //allows mutual recursion amongst functions to be defined in the input
    pr.instantiate_procedure_calls(true);
    if(type_checking) {
        bool type_check_success = pr.infer_types_repo();
        if(!type_check_success) {
            logger().error("procedure_repository - Error type checking.");      
        }
    }
    return n;
}

std::ostream& operator<<(std::ostream& out, procedure_repository pr);

}} // ~namespaces combo opencog

#endif

