/** moses_exec.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 * Copyright (C) 2012, 2013 Poulin Holdings LLC
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

#include <signal.h>
#include <unistd.h>

#include <opencog/util/log_prog_name.h>

#include "moses_exec.h"
#include "demo-problems.h"
#include "table-problems.h"
#include "problem.h"

#include "../moses/moses_main.h"

namespace opencog { namespace moses {

using namespace std;
using namespace reduct;

static void log_output_error_exit(string err_msg)
{
    logger().error() << "Error: " << err_msg;
    cerr << "Error: " << err_msg << endl;
    exit(1);    
}

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
            stringstream ss;
            ss << "type " << tt << " currently not supported.";
            log_output_error_exit(ss.str());
    }

    logger().info() << "Alphabet size = " << as
                    << " output = " << output_type;
    return as;
}

int moses_exec(int argc, char** argv)
{
    problem_params pms;
    pms.parse_options(argc, argv);

    register_demo_problems();
    register_table_problems();

    problem_base* probm = find_problem(pms.problem);
    if (probm)
    {
        probm->run(pms);
        return 0;
    }

    stringstream ss;
    ss << "problem type \"" << pms.problem << "\" is currently unsupported.";
    log_output_error_exit(ss.str());

    return 1;
}

int moses_exec(const vector<string>& argvs)
{
    char** argv = new char*[argvs.size()];
    for(size_t i = 0; i < argvs.size(); ++i) {
        argv[i] = const_cast<char*>(argvs[i].c_str());
    }
    int res = moses_exec(argvs.size(), argv);
    delete[] argv;
    return res;
}

} // ~namespace moses
} // ~namespace opencog
