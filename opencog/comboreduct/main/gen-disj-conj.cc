/*
 * opencog/comboreduct/main/gen-disj-conj.cc
 *
 * Copyright (C) 2012 OpenCog Foundation
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

#include <fstream>

#include <boost/program_options.hpp>
#include <boost/range/numeric.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/random.h>
#include <opencog/util/lazy_random_selector.h>
#include <opencog/util/dorepeat.h>

#include "../combo/iostream_combo.h"

using namespace std;
using namespace boost::program_options;
using namespace opencog;
using namespace combo;

/**
 * Program to generate a combo tree of a disjunction of
 * conjunctions. To test feature selection within moses when the
 * fitness function is to maximize precision.
 */
int main(int argc, char** argv)
{
    // See option help message for more info on these variables
    unsigned long rand_seed;
    unsigned disj_children, conj_children, conj_children_std;
    float repl_prob;
    string output_file;
    bool swap;
    
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")

        ("rand,r", value<unsigned long>(&rand_seed)->default_value(0),
         "Random seed.\n")

        ("disj-children,d", value<unsigned>(&disj_children)->default_value(10),
         "Number of children (conjunctions) of the main disjunction.\n")

        ("conj-children,c", value<unsigned>(&conj_children)->default_value(10),
         "Number of children (litterals) of the each conjunction.\n")

        ("conj-children-std,s",
         value<unsigned>(&conj_children_std)->default_value(0),
         "Standard deviation of the number of children (litterals) "
         "of the each conjunction (expect an integer).\n")

        ("repl-prob,p",
         value<float>(&repl_prob)->default_value(0.0),
         "When selecting a variable, probability of selecting with "
         "replacement (rather than without replacement). This is a way "
         "to control the degree of intersection of variables between "
         "conjunctions. 0 means no replacement (no intersection of variables), "
         "1 means maximum intersection (uniform distribution with replacement).\n")

        ("swap,w",
         "Swap conjunctions and disjunctions (that is generate a "
         "conjunction of disjunctions instead).\n")

        ("output-file,o", value<string>(&output_file),
         "File where to save the results. "
         "If none is provided it outputs on the stdout.\n")
        
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if(vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    // set flags
    swap = vm.count("swap");
    
    // init random generator
    randGen().seed(rand_seed);

    // determine the number of litterals of each conjunctions
    vector<unsigned> actual_conj_children;
    dorepeat(disj_children) {
        unsigned acc = gaussian_rand(conj_children, conj_children_std);
        actual_conj_children.push_back(acc);
    }

    // // debug print
    // cout << "actual_conj_children = ";
    // printlnContainer(actual_conj_children);
    // // ~debug print

    // determine the maximum number of variables
    unsigned max_var = boost::accumulate(actual_conj_children, 0);

    // // debug print
    // cout << "max_var = " << max_var << endl;
    // // ~debug print    

    // sampling without replacement
    lazy_random_selector smp_wor(max_var);

    // sampling with replacement
    auto smp_wr = [&]() -> unsigned { return randGen().randint(max_var); };

    auto pos_swap = [&swap](const vertex v) { return swap ? swap_and_or(v) : v; };
    
    // generate tree
    typedef combo_tree::iterator pre_it;
    // create disjunction (or conjunction if swapped)
    combo_tree tr(pos_swap(id::logical_or));
    pre_it head = tr.begin();
    foreach(unsigned cc, actual_conj_children) {
        // create conjunctions (or disjunctions if swapped)
        pre_it child = cc ?
            tr.append_child(head, pos_swap(id::logical_and)) : pre_it();
        // add litterals
        dorepeat(cc) {
            arity_t idx =
                (biased_randbool(repl_prob) ? smp_wr() : smp_wor.select()) + 1;
            if (biased_randbool(0.5)) // whether the litteral is negative
                idx *= -1;
            tr.append_child(child, argument(idx));
        }
    }

    // output tree
    if(output_file.empty())
        cout << tr << std::endl;
    else {
        ofstream of(output_file.c_str(), ios_base::app);
        of << tr << std::endl;
    }
}
