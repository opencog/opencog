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

#include <ext/stdio_filebuf.h>

#include <boost/program_options.hpp>

#include <opencog/util/iostreamContainer.h>

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
    // exemplar, output options, jobs, max_evals, max_gens and log_file_dep_opt
    for(variables_map::const_iterator it = vm.begin(); it != vm.end(); it++) {
        if(it->first != exemplars_str_opt.first
           && it->first != exemplars_str_opt.first
           && it->first != output_bscore_opt.first
           && it->first != output_complexity_opt.first
           && it->first != output_eval_number_opt.first
           && it->first != jobs_opt.first
           && it->first != max_evals_opt.first
           && it->first != max_gens_opt.first
           && !it->second.defaulted()) {
            res += string(" --") + it->first + " " + to_string(it->second);
        }
    }
    // add exemplar option
    stringstream ss;
    ss << tr;
    res += string(" -") + exemplars_str_opt.second + " \"" + ss.str() + "\"";
    // add output options
    res += string(" -") + output_bscore_opt.second + " 1";
    res += string(" -") + output_complexity_opt.second + " 1";
    res += string(" -") + output_eval_number_opt.second + " 1";    
    // add number of evals option
    res += string(" -") + max_evals_opt.second + " " 
        + boost::lexical_cast<string>(max_evals);
    // add one generation option
    res += string(" -") + max_gens_opt.second + " 1";
    // add log option determined name option
    res += string(" -") + log_file_dep_opt_opt.second;
    
    OC_ASSERT(res.size() < 255,
              "It is unlikely the OS support such a long name %s, the only thing"
              " to do is upgrade the code so that it can express this command"
              " in less than 255 chars", res.c_str());

    std::cout << res << std::endl;

    return res;
}

/**
 * read the istream, add the candidates, fill max_evals
 */
void parse_result(istream& in, metapop_candidates& candidates, int& evals) {
    while(!in.eof()) {
        string s;
        in >> s;
        if(s.empty())
            continue;
        else if(s == string(number_of_evals_str).append(":")) {
            in >> evals;
        } else {
            // read score
            score_t score = boost::lexical_cast<score_t>(s);
            // read complexity
            complexity_t complexity;
            in >> complexity;
            // read candidate
            combo_tree tr;
            in >> tr;
            // read bscore
            behavioral_score bscore;
            opencog::istreamContainer(in, std::back_inserter(bscore), "[", "]");
            // insert read element in candidates
            candidates.insert(std::make_pair(tr,
                                             std::make_pair(bscore,
                                                            make_pair(score,
                                                                      complexity))));
        }
    }
};

// run the given command, returns in stdout stream and fill pid with
// its PID.
// Works only under linux or other UNIX
FILE* launch_command(string command, int& pid) {
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

    typedef typename metapopulation<Scoring, BScoring, Optimization>::const_iterator mp_cit;
    typedef std::map<FILE*, int> proc_map;
    proc_map pm;

    int gen_idx = 0;

    while ((mp.n_evals() < pa.max_evals) && (pa.max_gens != gen_idx)) {
        // if there exists free resource, launch a process
        if(pm.size() < jobs) {
            mp_cit exemplar = mp.select_exemplar();
            if(exemplar != mp.end()) {
                const combo_tree& tr = get_tree(*mp.select_exemplar());
                mp.visited().insert(tr);

                string command_line =
                    build_command_line(vm, tr, pa.max_evals - mp.n_evals());

                int pid;
                FILE* fp = launch_command(command_line, pid);

                // Logger
                logger().info("Generation: %d", gen_idx);
                logger().info("Launch command: '%s'", command_line.c_str());
                logger().info("corresponding to PID = %d", pid);
                // ~Logger

                pm.insert(make_pair(fp, pid));
                gen_idx++;
            }
        }

        // check for result and merge if so
        for(proc_map::iterator cit = pm.begin(); cit != pm.end();) {
            if(!pid_running(cit->second)) { // result is ready
                FILE* fp = cit->first;
                // build istream from fp
                __gnu_cxx::stdio_filebuf<char> pipe_buf(fp, ios_base::in);
                istream sp(&pipe_buf);
                
                // parse the result
                metapop_candidates candidates;
                int evals;
                parse_result(sp, candidates, evals);
                mp.n_evals() += evals;

                // update best and merge
                mp.update_best_candidates(candidates);
                merge_nondominating(candidates.begin(), candidates.end(), mp);

                // close file and remove proc info from pm
                pclose(fp);
                proc_map::iterator next_cit(cit);
                next_cit++;
                pm.erase(cit);
                cit = next_cit;
            }
            else cit++;
        }

        // wait for a second to not take all resources
        sleep(1);

        if (mp.best_score() >= pa.max_score || mp.empty())
            break;
    }
    // Logger
    logger().info("Distributed MOSES ends");
    // ~Logger
}

} // ~namespace moses

#endif // _OPENCOG_DISTRIBUTED_MOSES_H
