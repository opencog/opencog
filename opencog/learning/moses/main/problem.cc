/**
 * problem.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 * Copyright (C) 2012, 2013 Poulin Holdings LLC
 * Copyright (C) 2013, 2014 Linas Vepstas
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
 *         Linas Vepstas <linasvepstas@gmail.com>
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

#include <stdlib.h>

#include <iostream>
#include <map>
#include <set>

#include <opencog/util/Logger.h>
#include <opencog/comboreduct/type_checker/type_tree.h>

#include "problem.h"

namespace opencog { namespace moses {

// =================================================================
// Register options

void option_manager::register_options(option_base* ob)
{
    _option_set.insert(ob);
}

void option_manager::init_options()
{
    _desc.add_options()
        ("help,h", "Produce help message.\n");

    foreach(auto ob, _option_set) {
        ob->add_options(_desc);
    }
}

void option_manager::parse_options(int argc, char* argv[])
{
    namespace po = boost::program_options;
    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, _desc), vm);
    }
    catch (po::error& e) {
        OC_ASSERT(0, "Fatal error: invalid or duplicated argument:\n\t%s\n", e.what());
    }
    po::notify(vm);

    if (vm.count("help") or argc == 1) {
        std::cout << _desc << std::endl;
        exit(1);
    }

    foreach(auto ob, _option_set) {
        ob->parse_options(vm);
    }

    // Log command-line args. Do this after above, since one of the
    // options might change the log file location. Perhaps we should
    // move that (log-file location) code here?
    std::string cmdline = "Command line:";
    for (int i = 0; i < argc; ++i) {
         cmdline += " ";
         cmdline += argv[i];
    }
    logger().info(cmdline);
}

// =================================================================
// Register and find problems, by name, so that they can be run.

void problem_manager::register_problem(problem_base* prob)
{
    std::pair<std::string, problem_base*> pr(prob->name(), prob);
    _problem_set.insert(pr);
}

problem_base* problem_manager::find_problem(const std::string& name)
{
    auto it = _problem_set.find(name);
    if (it != _problem_set.end())
        return it->second;
    return NULL;
}

problem_manager::~problem_manager()
{
    foreach(auto pr, _problem_set) {
        delete pr.second;
    }
}

// =================================================================
// Common utility

static void log_output_error_exit(std::string err_msg)
{
    logger().error() << "Error: " << err_msg;
    std::cerr << "Error: " << err_msg << std::endl;
    exit(1);    
}

using namespace combo;

static bool contains_type(const type_tree_pre_it it, id::type_node ty)
{
    for (type_tree_sib_it tts = it.begin(); tts != it.end(); tts++)
        if (*tts == ty) return true;
    return false;
}

/**
 * Determine the alphabet size given the type_tree of the problem and
 * a list of operators to ignore.  This is a somewhat ad-hoc value,
 * meant to help compute the complexity ratio.
 */
unsigned alphabet_size(const type_tree& tt, const vertex_set ignore_ops)
{
    unsigned as = 0;
    // arity will be zero for anything that isn't a lambda_type.
    // However, all tables will be lambda_type ...
    combo::arity_t arity = type_tree_arity(tt);
    type_node output_type = get_type_node(get_signature_output(tt));

    switch (output_type) {
        case id::boolean_type:
            // 3 operators: and, or, not
            as = 3 + arity;
            break;

        case id::contin_type:
            // Set alphabet size, 8 is roughly the number of operators
            // in contin formula, it will have to be adapted
            // Well, there could be a mix of booleans too.
            as = 8 + arity - ignore_ops.size();
            break;

        case id::enum_type: {

            // Try to take into account a table with both booleans and
            // contins.
            type_tree_pre_it ttl = tt.begin();

            as = arity + enum_t::size();

            if (contains_type(ttl, id::contin_type))
                as += 8 - ignore_ops.size();

            if (contains_type(ttl, id::boolean_type))
                as += 3;

            break;
        }
        case id::ann_type:

            as = 2 + arity*arity; // to account for hidden neurons, very roughly
            break;

        case id::lambda_type:
        case id::application_type:
        case id::union_type:
        case id::arg_list_type:
        case id::action_result_type:
        case id::definite_object_type:
        case id::action_definite_object_type:
        case id::indefinite_object_type:
        case id::message_type:
        case id::action_symbol_type:
        case id::wild_card_type:
        case id::unknown_type:
        case id::ill_formed_type:
        default:
            std::stringstream ss;
            ss << "alphabet_size: type " << tt << " currently not supported.";
            log_output_error_exit(ss.str());
    }

    logger().info() << "Alphabet size = " << as
                    << " output = " << output_type;
    return as;
}

} // ~namespace moses
} // ~namespace opencog

