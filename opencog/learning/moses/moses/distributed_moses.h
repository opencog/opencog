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
#include <stdlib.h>
#include <signal.h>

#include <sys/types.h>
#include <unistd.h>

#include <ext/stdio_filebuf.h>

#include <boost/program_options.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/lexical_cast.hpp>

#include <opencog/util/iostreamContainer.h>
#include <opencog/util/log_prog_name.h>

#include "../main/moses_exec_def.h"
#include "metapopulation.h"
#include "moses.h"

namespace opencog { namespace moses {

using namespace boost::program_options;
using boost::tuple;
using boost::make_tuple;
using boost::lexical_cast;

// get the pid of the main instance of moses (the one launching the
// others). This is useful to have more uniqueness in the log names of
// the launched moseses.
pid_t get_parent_pid();

// map the PID to
// 1) the command line (string)
// 2) the process output temporary file name (string)
// 3) its corresponding FILE (FILE*)
// 4) and the number of jobs it is running (unsigned)
typedef tuple<string, string, FILE*, unsigned> proc_info;
typedef std::map<int, proc_info> proc_map;
// proc_map's access functions
int get_pid(const proc_map::value_type& pmv);
string get_cmd(const proc_map::value_type& pmv);
string get_tmp(const proc_map::value_type& pmv);
FILE* get_file(const proc_map::value_type& pmv);
unsigned get_num_jobs(const proc_map::value_type& pmv);

// map the name of host and its proc_map
typedef std::map<string, proc_map> host_proc_map;
// host_proc_map's access functions
const string& get_hostname(const host_proc_map::value_type& hp);
const proc_map& get_proc_map(const host_proc_map::value_type& hp);
const unsigned get_total_jobs(const host_proc_map::value_type& hp);

// number of running processes
unsigned running_proc_count(const host_proc_map& hpm);

/**
 * generate a command line that launches moses-exec with exemplar base
 * 'tr' keeping all initial options, but running over one generation
 * and returning the adequate information to merge the result with the
 * metapopulation
 */
string build_cmdline(const variables_map& vm, 
                     const combo_tree& tr,
                     const string& host_name,
                     unsigned n_jobs,
                     unsigned max_evals,
                     unsigned gen_idx);

// run the given command and return its corresponding proc_map
proc_map::value_type launch_cmd(string cmd, unsigned n_jobs);
// check if a file is being used by process of PID pid 
bool is_being_written(const string& file_name, int pid);

// check if a process is running
bool is_running(const proc_map::value_type& pmv);
/**
 * read the istream, add the candidates, fill max_evals
 * @todo replace metapop_candidates by bscored_combo_tree_set
 */
void parse_result(istream& in, metapop_candidates& candidates, int& evals);
// like above but uses a proc_map::value_type instead of istream
void parse_result(const proc_map::value_type& pmv, 
                  metapop_candidates& candidates, int& evals);

host_proc_map init(const jobs_t& jobs);
// remove it from proc_map and return the iterator pointing to the
// next process
proc_map::iterator remove_proc(proc_map& pm,  proc_map::iterator it);

// kill all processes in pm
void killall(proc_map& pm);
// kill all processes in hpm
void killall(host_proc_map& hpm);

host_proc_map::iterator find_free_resource(host_proc_map& hpm,
                                           const jobs_t& jobs);

// return true iff all resources are completely free, i.e. no more
// candidates are expected to be merged
bool all_resources_free(const host_proc_map& hpm);

/**
 * the main function of Distributed MOSES
 *
 * @param mp          the metapopulation 
 */
template<typename Scoring, typename BScoring, typename Optimization>
void distributed_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
                       const variables_map& vm, const jobs_t& jobs,
                       const moses_parameters& pa = moses_parameters())
{
    // Logger
    logger().info("Distributed MOSES starts");
    // ~Logger

    typedef typename metapopulation<Scoring, BScoring,
                                    Optimization>::const_iterator mp_cit;

    int gen_idx = 1;

    host_proc_map hpm = init(jobs);

    while ((mp.n_evals() < pa.max_evals) && (pa.max_gens != gen_idx) 
           && (mp.best_score() < pa.max_score)) {
        // if there exists free resource, launch a process
        host_proc_map::iterator hpm_it = find_free_resource(hpm, jobs);
        if(hpm_it != hpm.end()) {
            const string& hostname = get_hostname(*hpm_it);
            unsigned n_jobs = jobs.find(hostname)->second;
            mp_cit exemplar = mp.select_exemplar();
            if(exemplar != mp.end()) {
                const combo_tree& tr = get_tree(*mp.select_exemplar());
                mp.visited().insert(tr);

                string cmdline =
                    build_cmdline(vm, tr, hostname, n_jobs,
                                  pa.max_evals - mp.n_evals(), gen_idx);

                proc_map::value_type pmv(launch_cmd(cmdline, n_jobs));
                hpm_it->second.insert(pmv);

                // Logger
                logger().info("Generation: %d", gen_idx);
                logger().info("Launch command: %s", get_cmd(pmv).c_str());
                logger().info("corresponding to PID = %d", get_pid(pmv));
                // ~Logger
                gen_idx++;
            } else if(all_resources_free(hpm)) { // can't find any
                                             // available exemplar and
                                             // there is no hope that
                                             // one will come
                // Logger
                logger().info("There is no more exemplar in the metapopulation"
                              " that has not been visited and"
                              " no more results from other process are expected."
                              " This is a blockage situation, several options"
                              " can be used to prevent that, see moses-exec -h");
                // ~Logger
                break;
            }
        }   

        // check for results and merge if necessary
        foreach(host_proc_map::value_type& hpmv, hpm) {
            for(proc_map::iterator it = hpmv.second.begin();
                it != hpmv.second.end();) {
                if(!is_running(*it)) { // result is ready
                    // parse the result
                    metapop_candidates candidates;
                    int evals;

                    // Logger
                    logger().info("Parsing results from PID %d", get_pid(*it));
                    // ~Logger

                    parse_result(*it, candidates, evals);
                    mp.n_evals() += evals;

                    // update best and merge
                    // Logger
                    logger().info("Merge %u candidates with the metapopulation",
                                  candidates.size());
                    if(logger().getLevel() >= Logger::FINE) {
                        logger().fine("Candidates with their bscores to merge with"
                                      " the metapopulation:");
                        stringstream ss;
                        logger().fine(mp.ostream(ss, candidates.begin(),
                                                 candidates.end(),
                                                 -1, true, true).str());
                    }
                    // ~Logger

                    bscored_combo_tree_set mc(candidates.begin(),
                                              candidates.end());

                    mp.update_best_candidates(mc);

                    mp.merge_candidates(mc);

                    // Logger
                    logger().info("Metapopulation size is %u", mp.size());
                    if(logger().getLevel() >= Logger::FINE) {
                        stringstream ss;
                        ss << "Metapopulation after merging: " << std::endl;
                        logger().fine(mp.ostream(ss, -1, true, true).str());
                    }
                    logger().fine("Number of evaluations so far: %d",
                                  mp.n_evals());
                    // ~Logger

                    // Logger
                    mp.log_best_candidates();
                    // ~Logger

                    // remove proc info from pm
                    it = remove_proc(hpmv.second, it);
                }
                else it++;
            }
        }

        // Logger
        logger().fine("Number of running processes: %u", running_proc_count(hpm));
        // ~Logger

        // wait for a second to not take all resources
        sleep(1);
    }

    // kill all children processes
    killall(hpm);

    OC_ASSERT(running_proc_count(hpm) == 0);

    // Logger
    logger().info("Distributed MOSES ends");
    // ~Logger
}

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_DISTRIBUTED_MOSES_H
