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

#include "moses_exec.h"
#include "demo-problems.h"
#include "table-problems.h"
#include "problem.h"

namespace opencog { namespace moses {

using namespace std;
using namespace reduct;

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

    logger().error() << "problem type \"" << pms.problem << "\" is currently unsupported.";
    std::cerr << "problem type \"" << pms.problem << "\" is currently unsupported." << std::endl;
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
