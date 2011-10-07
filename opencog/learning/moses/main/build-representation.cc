/*
 * opencog/learning/moses/main/build-representation.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#include <boost/lexical_cast.hpp>
#include <iostream>

#include <opencog/util/mt19937ar.h>
#include <opencog/comboreduct/reduct/reduct.h>

#include "../moses/moses.h"
#include "../optimization/optimization.h"
#include "../moses/scoring_functions.h"

using namespace opencog;
using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;

int main(int argc,char** argv) { 
    combo_tree tr;
    MT19937RandGen rng(0);
    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;
        
        logical_reduce(2, tr);
        
        representation rep(logical_reduction(2), logical_reduction(2),
                           tr, infer_type_tree(tr), rng);
    
        /*combo_tree tmp(rep.exemplar());
          
          for (int i=0;i<10;++i) { 
          cout << rep.exemplar() << endl;
          
          instance inst(rep.fields().packed_width());
          for (field_set::disc_iterator it=rep.fields().begin_raw(inst);
          it!=rep.fields().end_raw(inst);++it)
          it.randomize();	
          
          cout << rep.fields().stream(inst) << endl;
          rep.transform(inst);
          cout << rep.exemplar() << endl;
          
          rep.clear_exemplar();
          OC_ASSERT(tmp==rep.exemplar());
          }*/
    }
}
