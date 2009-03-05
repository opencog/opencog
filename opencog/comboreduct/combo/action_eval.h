#ifndef _COMBO_ACTION_EVAL_H
#define _COMBO_ACTION_EVAL_H

#include <LADSUtil/exceptions.h>
#include <LADSUtil/tree.h>
#include <LADSUtil/RandGen.h>

#include "ComboReduct/combo/vertex.h"
#include "ComboReduct/crutil/exception.h"
#include "ComboReduct/combo/using.h"
#include "ComboReduct/combo/type_tree.h"

#include <exception>

namespace combo {
  //inline boost::variant<vertex,combo_tree::iterator>& binding(int idx) {
  //  static LADSUtil::hash_map<int,boost::variant<vertex,combo_tree::iterator> > map; //to support lazy evaluation, can also bind to a subtree
  //  return map[idx];
  //}

  template<typename It>
  vertex action_eval_throws(LADSUtil::RandGen& rng, It it) throw(EvalException, LADSUtil::AssertionException, std::bad_exception) {
    typedef typename It::sibling_iterator sib_it;
    //argument
    //WARNING : we assume the argument is necessarily an action and therefor
    //never has a negative index
    if(is_argument(*it)) {
      int idx=get_argument(*it).idx;
      //assumption : when idx is negative the argument is necessary boolean
      LADSUtil::cassert(TRACE_INFO, idx > 0, "argument is necessarily an action and therefore never has a neg idx.");
      if (const vertex* v=boost::get<const vertex>(&binding(idx)))
	return *v;
      else
	return action_eval_throws(rng, boost::get<combo_tree::iterator>(binding(idx)));
    }
    //action sequence
    if(*it==id::sequential_and) {
      for(sib_it sib = it.begin(); sib != it.end(); ++sib) {
	if(action_eval_throws(rng, sib) != id::action_success)
	  return id::action_failure;
      }
      return id::action_success;
    }
    else if(*it==id::sequential_or) {
      for(sib_it sib = it.begin(); sib != it.end(); ++sib) {
	if(action_eval_throws(rng, sib) == id::action_success)
	  return id::action_success;
      }
      return id::action_failure;
    }
    else if(*it==id::sequential_exec) {
      for(sib_it sib = it.begin(); sib != it.end(); ++sib)
	action_eval_throws(rng, sib);
      return id::action_success;
    }
    //action_if
    else if(*it==id::boolean_action_if) {
      LADSUtil::cassert(TRACE_INFO, it.number_of_children()==3, 
             "combo_tree node should have exactly three children (id::boolean_if).");
      sib_it sib = it.begin();
      vertex vcond = action_eval_throws(rng, sib);
      ++sib;
      if(vcond==id::action_success) {
	//TODO : check that b1 and b2 have type boolean
	return eval_throws(rng, sib);
      }
      else {
	++sib;
	return eval_throws(rng, sib);
      }
    }
    else if(*it==id::contin_action_if) {
      LADSUtil::cassert(TRACE_INFO, it.number_of_children()==3,
             "combo_tree node should have exactly three children (id::contin_action_if).");
      sib_it sib = it.begin();
      vertex vcond = action_eval_throws(rng, sib);
      ++sib;
      if(vcond==id::action_success) {
	//TODO : check that b1 and b2 have type contin
	return eval_throws(rng, sib);
      }
      else {
	++sib;
	return eval_throws(rng, sib);
      }
    }
    else if(*it==id::action_action_if) {
      LADSUtil::cassert(TRACE_INFO, it.number_of_children()==3,
             "combo_tree node should have exactly three children (id::contin_action_if).");
      sib_it sib = it.begin();
      vertex vcond = action_eval_throws(rng, sib);
      ++sib;
      if(vcond==id::action_success) {
	//TODO : check that b1 and b2 have type action
	return action_eval_throws(rng, sib);
      }
      else {
	++sib;
	return action_eval_throws(rng, sib);
      }
    }
    else {
      return *it;
    }
  }

}

#endif
