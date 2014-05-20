/*
 * opencog/comboreduct/main/tree_gen.cc
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
#include <iostream>
#include <boost/lexical_cast.hpp>

#include <opencog/util/selection.h>
#include <opencog/util/mt19937ar.h>

#include "tree_generation.h"

int main(int argc, char** argv)
{
    using namespace opencog;
    using namespace trees;
    using namespace boost;
    using namespace std;

    int max_depth;
    int noe; //number of expressions to generate
    bool tce; //type check enabled

    if (argc != 4) {
        cout << "Usage :" << endl <<
             "tree_gen [bool|real|mix] max_depth number_of_expressions" << endl;
        exit(1);
    }
    //Random number generator
    randGen().seed(0);
    NodeSelector<string> ss(randGen());

    max_depth = lexical_cast<int>(argv[2]);
    noe = lexical_cast<int>(argv[3]);
    tce = string(argv[1]) == string("mix");
    //When generate a boolean-type or mix-type tree, add logical operators
    if (string(argv[1]) == string("bool") || string(argv[1]) == string("mix")) {
        ss.add("and", 2, 100);
        ss.add("or", 2, 100);
        ss.add("not", 1, 100);
    }
    //When generate a real-type or mixed-type tree, add operators
    if (string(argv[1]) == string("real") || string(argv[1]) == string("mix")) {
        //second arg is arity, third is (relative) selection likelihood
        ss.add("+", 2, 100);
        ss.add("*", 2, 100);
        ss.add("/", 2, 100);

        ss.add("log", 1, 100);
        ss.add("exp", 1, 100);
        ss.add("sin", 1, 100);

        ss.add("1", 0, 100);
        ss.add("0", 0, 100);
        //and some random const between 0 and 1
        for (int i = 0;i < 200;++i)
            ss.add(lexical_cast<string>(randGen().randfloat()), 0, 1);
    }
    //special operators for mixed-type tree
    if (string(argv[1]) == string("mix")) {
        ss.add("contin_if", 3, 100);
        ss.add("0<", 1, 100);
        ss.add("impulse", 1, 100);
    }
    //add parameters
    ss.add("$1", 0, 100);
    ss.add("$2", 0, 100);
    ss.add("$3", 0, 100);
    ss.add("$4", 0, 100);
    ss.add("$5", 0, 100);

    vector<tree<string> > trees(noe);
    ramped_half_and_half(trees.begin(), trees.end(), ss, 2, max_depth, tce);
    //out put generated trees.
    for (vector<tree<string> >::const_iterator tr = trees.begin();
            tr != trees.end();++tr)
        cout << (*tr) << endl;
}
