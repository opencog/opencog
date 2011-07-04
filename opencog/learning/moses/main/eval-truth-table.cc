/*
 * opencog/learning/moses/eda/eval-truth_table.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Predrag Janicic
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
#include <opencog/learning/moses/moses/moses.h>
#include <opencog/learning/moses/moses/scoring_functions.h>

//ant_combo_vocabulary is used only for the boolean core vocabulary
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>

//using namespace std;
using namespace opencog;
using namespace moses;
using namespace ant_combo;

int main(int argc,char** argv) { 
  int arity;
  try {
    if (argc!=2)
      throw "foo";
  } catch (...) {
    cerr << "usage: " << argv[0] << " TestFileName" << endl;
    exit(1);
  }

  ifstream in(argv[1]);

  CaseBasedBoolean bc(in);
  arity=bc.arity();

  combo_tree tr;
  cout << "Enter formula:" << endl;
  cin >> tr;

  cout << "Evaluating formula:" << tr << endl;

  ConfusionMatrix cm=bc.ComputeConfusionMatrix(tr);

  cout << "TP: " << cm.TP << "  FN: " << cm.FN << endl;  
  cout << "FP: " << cm.FP << "  TN: " << cm.TN << endl;  

  cout << "Accuracy:  " << (float)(cm.TP+cm.TN)/(cm.FP+cm.TP+cm.TN+cm.FN) << endl;

}
