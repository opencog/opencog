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

#include <boost/program_options.hpp>
#include <boost/tuple/tuple.hpp>

#include <opencog/util/iostreamContainer.h>
#include <opencog/util/log_prog_name.h>

#include "../metapopulation/metapopulation.h"
#include "moses_params.h"

namespace opencog { namespace moses {

// both sender and receiver must use exactly this string for n_evals.
static const std::string number_of_evals_str = "n_evals";

// get the pid of the main instance of moses (the one launching the
// others). This is useful to have more uniqueness in the log names of
// the launched moseses.
pid_t get_parent_pid();

// map the PID to
// 1) the command line (string)
// 2) the process output temporary file name (string)
// 3) its corresponding FILE (FILE*)
// 4) and the number of jobs it is running (unsigned)
typedef boost::tuple<std::string, std::string, FILE*, unsigned> proc_info;
typedef std::map<int, proc_info> proc_map;
// proc_map's access functions
int get_pid(const proc_map::value_type& pmv);
std::string get_cmd(const proc_map::value_type& pmv);
std::string get_tmp(const proc_map::value_type& pmv);
FILE* get_file(const proc_map::value_type& pmv);
unsigned get_num_jobs(const proc_map::value_type& pmv);

// map the name of host and its proc_map
typedef std::map<std::string, proc_map> host_proc_map;
// host_proc_map's access functions
const std::string& get_hostname(const host_proc_map::value_type& hp);
const proc_map& get_proc_map(const host_proc_map::value_type& hp);
const unsigned get_total_jobs(const host_proc_map::value_type& hp);

// number of running processes
unsigned running_proc_count(const host_proc_map& hpm);

/**
 * generate a command line that launches moses with exemplar base 'tr'
 * keeping all initial options, but running over one generation and
 * returning the adequate information to merge the result with the
 * metapopulation
 */
std::string build_cmdline(const boost::program_options::variables_map& vm, 
                     const combo_tree& tr,
                     const std::string& host_name,
                     unsigned n_jobs,
                     unsigned max_evals,
                     unsigned gen_idx);

// run the given command and return its corresponding proc_map
proc_map::value_type launch_cmd(std::string cmd, unsigned n_jobs);
// check if a file is being used by process of PID pid 
bool is_being_written(const std::string& file_name, int pid);

// check if a process is running
bool is_running(const proc_map::value_type& pmv);
/**
 * read the istream, add the candidates, fill max_evals
 */
void parse_result(std::istream& in, scored_combo_tree_set& candidates, int& evals);
// like above but uses a proc_map::value_type instead of istream
void parse_result(const proc_map::value_type& pmv, 
                  scored_combo_tree_set& candidates, int& evals);

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
 * Distributed MOSES main entry point. 
 *
 * This function implements a simple, network-distributed scatter-gather
 * algorithm to distribute moses over multiple networked machines.  This
 * algorithm is simple, and thus somewhat slow, in two different respects:
 *
 * 1) It uses only a single thread for both the scatter and gathe steps.
 *    The gather step involves a rather slow, expensive step, the merge
 *    of a deme into the metapopulation.  Because of this, some processors
 *    may starve for input, as they will not be fed until the merge is
 *    complete.  In particular, this means that this algorithm will not
 *    scale up to any large number of processors (probably not more than
 *    a few dozen!?)  A solution to this problem would be to run the scatter
 *    and gather steps in separate threads.
 *
 * 2) It performs network distribution by launching a new instance of moses
 *    for each deme, using the ssh protocol.  This adds a significant
 *    overhead: a) ssh negotiation b) ssh encryption c) moses startup
 *    d) ascii encoding encryption, decryption and finally, decoding of
 *    the combo_tree and behavioural score results.  The biggest overhead
 *    is probably due to step c), followed by step d), and then a). These
 *    overheads might be in the multi-second range.  This can be firghtful,
 *    especially if the deme evaluation takes only a few seconds.
 *    The solution to this is to use a tigher communications medium and
 *    design, such as MPI.
 *
 * Note: this algo requires that the commands 'fuser', 'pgrep' and of
 * course 'moses' be found in the default ssh shell search path.
 *
 * @param mp          the metapopulation 
 */
void distributed_moses(metapopulation& mp,
                       const moses_parameters& pa,
                       moses_statistics& stats);

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_DISTRIBUTED_MOSES_H
