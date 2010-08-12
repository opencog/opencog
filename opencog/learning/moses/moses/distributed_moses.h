/** distributed_moses.h --- 
 *
 * Copyright (C) 2010 Novamente LLC
 *
 * Author: Nil Geisweiller
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


#ifndef _OPENCOG_DISTRIBUTED_MOSES_H
#define _OPENCOG_DISTRIBUTED_MOSES_H

#include <stdio.h>
#include <stropts.h>

#include <boost/program_options.hpp>

#include "../main/moses_options_names.h"
#include "metapopulation.h"
#include "moses.h"

namespace moses
{

using namespace boost::program_options;

/**
 * generate a command line that launches moses-exec with exemplar base
 * 'tr' keeping all initial options, but running over one generation
 * and returning the adequate information to merge the result with the
 * metapopulation
 */
string build_command_line(const variables_map& vm, 
                          const combo_tree& tr, unsigned int max_evals) {
    string res("./moses-exec");
    // replicate initial command's options, except:
    // exemplar, output options, jobs and max_evals
    for(variables_map::const_iterator it = vm.begin(); it != vm.end(); it++) {
        if(it->first != exemplars_str_opt_name
           && it->first != exemplars_str_opt_name
           && it->first != output_bscore_opt_name
           && it->first != output_complexity_opt_name
           && it->first != output_eval_number_opt_name
           && it->first != jobs_opt_name
           && it->first != max_evals_opt_name
           && !it->second.defaulted()) {
            res += string(" --") + it->first + " " + to_string(it->second);
        }
    }
    // add exemplar option
    stringstream ss;
    ss << tr;
    res += string(" -") + exemplars_str_opt_ab + " \"" + ss.str() + "\"";
    // add output options
    res += string(" -") + output_bscore_opt_ab + " 1";
    res += string(" -") + output_complexity_opt_ab + " 1";
    res += string(" -") + output_eval_number_opt_ab + " 1";    
    // add number of evals option
    res += string(" -") + max_evals_opt_ab + " " 
        + boost::lexical_cast<string>(max_evals);
    
    OC_ASSERT(res.size() < 255,
              "It is unlikely the OS support such a long name %s, the only thing"
              " to do is upgrade the code so that it can express this command"
              " in less than 255 chars", res.c_str());

    std::cout << res << std::endl;

    return res;
}

void parse_result() {
    // TODO
}

// run the given command, returns in stdout stream and fill pid with
// its PID.
// Works only under linux or other UNIX
FILE* run_command(string command, int& pid) {
    // launch the command
    FILE* fp = popen(command.c_str(), "r");
    // get its PID
    FILE* fp_pid = popen("pgrep -n moses-exec", "r");
    fscanf(fp_pid, "%u", &pid);
    return fp;
}

// check if a process is running, works only under linux or other UNIX
bool pid_running(int pid) {
    string path("/proc/");
    path += boost::lexical_cast<string>(pid);
    return access(path.c_str(), F_OK) != -1;
}

/**
 * the main function of Distributed MOSES
 *
 * @param mp          the metapopulation 
 */
template<typename Scoring, typename BScoring, typename Optimization>
void distributed_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
                       const variables_map& vm, unsigned int jobs,
                       const moses_parameters& pa = moses_parameters())
{
    // Logger
    logger().info("Distributed MOSES starts");
    // ~Logger

    // int gen_idx = 0;

    //    while ((mp.n_evals() < max_evals) && (max_gens != gen_idx++)) {
    const combo_tree exemplar(get_tree(*mp.select_exemplar()));

    string command_line = build_command_line(vm, exemplar,
                                             pa.max_evals - mp.n_evals());
    
    int pid;
    FILE* fp = run_command(command_line, pid);

    std::cout << pid << std::endl;

    while(pid_running(pid)) {
        sleep(1);
        std::cout << "STILL RUNNING" << std::endl;
    }

    char line[4096];
    while ( fgets( line, sizeof line, fp)) {
        printf("%s", line);
    }
    pclose(fp);
        
        //run a generation
        // if (mp.expand(max_evals - mp.n_evals(), max_score, ignore_ops,
        //               perceptions, actions)) {
        // } else // In iterative hillclimbing it is possible (but not
        //        // likely) that the metapop gets empty and expand
        //        // return false
        //     break;
        // if (mp.best_score() >= max_score || mp.empty())
        //     break;
    //    }    
    // Logger
    logger().info("Distributed MOSES ends");
    // ~Logger
}

} // ~namespace moses

#endif // _OPENCOG_DISTRIBUTED_MOSES_H
