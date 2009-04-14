/*
 * opencog/learning/moses/moses/representation.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Predrag Janicic
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
#include <opencog/util/lazy_random_selector.h>

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/reduct/meta_rules.h>
#include <opencog/comboreduct/reduct/logical_rules.h>
#include <opencog/comboreduct/reduct/general_rules.h>

#include "using.h"
#include "representation.h"
#include "build_knobs.h"

namespace moses {

  representation::representation(const reduct::rule& simplify,
                                 const combo_tree& exemplar_,
				 const type_tree& t, 
                                 opencog::RandGen& _rng,
                                 const operator_set* os,
                                 const combo_tree_ns_set* perceptions,
                                 const combo_tree_ns_set* actions) : _exemplar(exemplar_), rng(_rng), _simplify(&simplify) {

#ifdef DEBUG_INFO
    std::cout << "Start building from exemplar: " << _exemplar << std::endl;
#endif

    //build the knobs
    build_knobs(rng,_exemplar,t,*this,os,perceptions,actions);
    
    //handle knob merging

    //convert the knobs into a field specification
    std::multiset<field_set::spec> tmp;
    foreach(const disc_map::value_type& v,disc)
      tmp.insert(v.first);
    foreach(const contin_map::value_type& v,contin)
      tmp.insert(v.first);
    _fields=field_set(tmp.begin(),tmp.end());
            
    std::cout << "#knobs " << disc.size() << " + " << contin.size() << std::endl;
  }

  void representation::transform(const instance& inst) {
    contin_map::iterator ckb=contin.begin();
    for (field_set::const_contin_iterator ci=_fields.begin_contin(inst);
	 ci!=_fields.end_contin(inst);++ci,++ckb) {
      ckb->second.turn(*ci);
      //_exemplar.validate();
    }

    //need to add first onto & then contin
    //cout << _fields.stream(inst) << endl;
    disc_map::iterator dkb=disc.begin();
    for (field_set::const_disc_iterator di=_fields.begin_disc(inst);
	 di!=_fields.end_disc(inst);++di,++dkb) {
      dkb->second->turn(*di);
      //_exemplar.validate();
    }
    for (field_set::const_bit_iterator bi=_fields.begin_bits(inst);
	 bi!=_fields.end_bits(inst);++bi,++dkb) {
      dkb->second->turn(*bi);
    }
    //cout << _exemplar << endl;
    // std::cout << "New exemplar (after build): " << _exemplar << std::endl;

  }

  void representation::clear_exemplar() {
    foreach(disc_v& v,disc)
      v.second->clear_exemplar();
    foreach(contin_v& v,contin)
      v.second.clear_exemplar();
  }


  void representation::get_clean_exemplar(combo_tree& result) { 
    using namespace reduct; 
    using std::cout;

    result.clear();
    result = exemplar();

    clean_reduce(result);

    (*_simplify)(result,result.begin());

    // clean_and_full_reduce(result,rng);
    //sequential(downwards(remove_null_vertices()),downwards(remove_dangling_junctors()))(result,result.begin());     
    
    return;
  }


  
} //~namespace moses
