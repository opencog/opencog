/*
 * opencog/comboreduct/combo/perception_eval.h
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
#ifndef _COMBO_PERCEPTION_EVAL_H
#define _COMBO_PERCEPTION_EVAL_H

#include <exception>

#include "util/exception.h"
#include "util/tree.h"

#include "comboreduct/combo/vertex.h"
#include "comboreduct/combo/using.h"
#include "comboreduct/combo/type_tree.h"


namespace opencog { namespace combo {

  template<typename It>
  vertex action_eval_throws(It it) throw(EvalException, std::bad_exception) {
    typedef typename It::sibling_iterator sib_it;
    //argument
    /*if(is_argument(*it)) {
      int idx=get_argument(*it).idx;
      //assumption : when idx is negative the argument is necessary boolean
      assert(idx>0);
      return binding(idx);
      }*/
    //perception
    if(*it==id::dummy_ultrametric) {
      return 0.0
    }
    else if(*it==id::dummy_transitive) {
      return id::logical_true;
    }
  }

} // ~namespace combo
} // ~namespace opencog 

#endif
