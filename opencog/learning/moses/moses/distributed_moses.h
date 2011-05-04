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

#include <ext/stdio_filebuf.h>

#include <boost/program_options.hpp>
#include <boost/tuple/tuple.hpp>

#include <opencog/util/iostreamContainer.h>
#include <opencog/util/log_prog_name.h>

#include "../main/moses_exec_def.h"
#include "metapopulation.h"
#include "moses.h"

namespace moses
{

using namespace boost::program_options;
using boost::tuple;
using boost::make_tuple;

// map the PID to the command line (cmd), the temporary file name
// (tmp), and the FILE of the output of the process (file)
typedef tuple<string, string, FILE*> cmd_tmp_file;
typedef std::map<int, cmd_tmp_file> proc_map;
int get_pid(const proc_map::value_type& pmv) {
    return pmv.first;
}
string get_cmd(const proc_map::value_type& pmv) {
    return pmv.second.get<0>();
}
string get_tmp(const proc_map::value_type& pmv) {
    return pmv.second.get<1>();
}
FILE* get_file(const proc_map::value_type& pmv) {
    return pmv.second.get<2>();
}
// map the name of host and its proc_map
typedef std::map<string, proc_map> host_proc_map;

// number of running processes
unsigned int running_proc_count(const host_proc_map& hpm) {
    unsigned int acc = 0;
    foreach(const host_proc_map::value_type& hpmv, hpm)
        acc += hpmv.second.size();
    return acc;
}

/**
 * generate a command line that launches moses-exec with exemplar base
 * 'tr' keeping all initial options, but running over one generation
 * and returning the adequate information to merge the result with the
 * metapopulation
 */
string build_cmdline(const variables_map& vm, 
                     const combo_tree& tr,
                     const string& host_name,
                     unsigned int max_evals) {
    string res;
    if(host_name == localhost)
        res = "moses-exec";
    else
        res = string("ssh ") + host_name + " 'moses-exec";
    // replicate initial command's options, except:
    // exemplar, output options, jobs, max_evals, max_gens and log_file_dep_opt
    for(variables_map::const_iterator it = vm.begin(); it != vm.end(); ++it) {
        if(it->first != exemplars_str_opt.first
           && it->first != exemplars_str_opt.first
           && it->first != output_bscore_opt.first
           && it->first != output_complexity_opt.first
           && it->first != output_eval_number_opt.first
           && it->first != jobs_opt.first
           && it->first != max_evals_opt.first
           && it->first != max_gens_opt.first
           && it->first != result_count_opt.first
           && !it->second.defaulted()) {
            string opt_name(" --");
            opt_name += it->first + " \"";
            res += opt_name + to_string(it->second, string("\"") + opt_name) + "\"";
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
    // add option to return all results
    res += string(" -") + result_count_opt.second + " -1";

    if(host_name != localhost) {
        res += "'";
    }

    return res;
}

// run the given command and return its corresponding proc_map
proc_map::value_type launch_cmd(string cmd) {
    // append " > tempfile&" to cmd
    char tempfile[] = "/tmp/moses-execXXXXXX";
    int fd = mkstemp(tempfile);
    if(fd == -1) {
        std::cerr << "could not create temporary file" << std::endl;
        exit(1);
    }
    string tmp(tempfile);
    cmd += string(" > ") + tmp + "&";

    // launch the command
    int res = system(cmd.c_str());
    if(res != 0) {
        std::cerr << "the execution of " << cmd << "has failed" << std::endl;
        exit(1);        
    }
    FILE* fp = fopen(tempfile, "r");

    // get its PID
    string exec_name = cmd.substr(0, cmd.find(" "));
    FILE* fp_pid = popen(string("pgrep -n ").append(exec_name).c_str(), "r");
    int pid;
    int count_matches = fscanf(fp_pid, "%u", &pid);
    OC_ASSERT(count_matches == 1);

    // make the proc_map
    return make_pair(pid, make_tuple(cmd, tmp, fp));
}

// check if a file is being used by process of PID pid 
    bool is_being_written(const string& file_name, int pid) {
    FILE* fp = popen(string("fuser ").append(file_name).append(" 2> /dev/null").c_str(), "r");
    while(!feof(fp)) {
        int p;
        int count_matches = fscanf(fp, "%u", &p);
        OC_ASSERT(count_matches == 1);
        if(pid == p)
            return true;
    }
    return false;
}

// check if a process is running
bool is_running(const proc_map::value_type& pmv) {
    // check if the file is still empty
    FILE* fp = get_file(pmv);
    if(fseek(fp, 0, SEEK_END)) {
        std::cerr << "Error while seeking to end of file" << std::endl;
        exit(1);
    }
    int length = ftell(fp);
    if (length < 0) {
        std::cerr << "Error while reading file position" << std::endl;
        exit(1);
    } else if(length == 0) {
        return true;
    }
    return is_being_written(get_tmp(pmv), get_pid(pmv));
}

/**
 * read the istream, add the candidates, fill max_evals
 */
void parse_result(istream& in, metapop_candidates& candidates, int& evals) {
    in.seekg(0);
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
            bscored_combo_tree candidate =
                make_pair(tr, make_pair(bscore, make_pair(score, complexity)));
            candidates.insert(candidate);
            // Logger
            if(logger().getLevel() >= opencog::Logger::FINE) {
                logger().fine("Parsed candidate:");
                stringstream ss;
                logger().fine(ostream_bscored_combo_tree(ss, candidate,
                                                         true, true).str());
            }
            // ~Logger
        }
    }
};
// like above but uses a proc_map::value_type instead of istream
void parse_result(const proc_map::value_type& pmv, 
                  metapop_candidates& candidates, int& evals) {
    FILE* fp = get_file(pmv);
    // build istream from fp
    __gnu_cxx::stdio_filebuf<char> pipe_buf(fp, ios_base::in);
    istream sp(&pipe_buf);
    // parse result
    parse_result(sp, candidates, evals);
    // close fp
    pclose(fp);
}


host_proc_map init(const jobs_t& jobs) {
    host_proc_map hpm;
    foreach(const jobs_t::value_type job, jobs) {
        hpm.insert(make_pair(job.first, proc_map()));
    }
    return hpm;
}

// remove it from proc_map and return the iterator pointing to the
// next process
proc_map::iterator remove_proc(proc_map& pm,  proc_map::iterator it) {
    proc_map::iterator next_it(it);
    next_it++;
    pm.erase(it);
    return next_it;
}

// kill all processes in pm
void killall(proc_map& pm) {
    for(proc_map::iterator it = pm.begin(); it != pm.end();) {
        if(is_running(*it))
            kill(get_pid(*it), 15); // send a TERM signal
        it = remove_proc(pm, it);
    }
}

// kill all processes in hpm
void killall(host_proc_map& hpm) {
    foreach(host_proc_map::value_type& hpmv, hpm)
        killall(hpmv.second);
}

host_proc_map::iterator find_free_resource(host_proc_map& hpm, const jobs_t& jobs) {
    host_proc_map::iterator hpm_it = hpm.begin();
    for(jobs_t::const_iterator jit = jobs.begin(); jit != jobs.end();
        ++jit, ++hpm_it) {
        OC_ASSERT(jit->first == hpm_it->first);
        if(hpm_it->second.size() < jit->second)
            break;
    }
    return hpm_it;
}

// return true iff all resources are completely free, i.e. no more
// candidates are expected to be merged
bool all_resources_free(const host_proc_map& hpm) {
    foreach(const host_proc_map::value_type& hpmv, hpm) {
        if(!hpmv.second.empty())
            return false;
    }
    return true;
}

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

    typedef typename metapopulation<Scoring, BScoring, Optimization>::const_iterator mp_cit;

    int gen_idx = 0;

    host_proc_map hpm = init(jobs);

    while ((mp.n_evals() < pa.max_evals) && (pa.max_gens != gen_idx) 
           && (mp.best_score() < pa.max_score)) {
        // if there exists free resource, launch a process
        host_proc_map::iterator hpm_it = find_free_resource(hpm, jobs);
        if(hpm_it != hpm.end()) {
            mp_cit exemplar = mp.select_exemplar();
            if(exemplar != mp.end()) {
                const combo_tree& tr = get_tree(*mp.select_exemplar());
                mp.visited().insert(tr);

                string cmdline =
                    build_cmdline(vm, tr, hpm_it->first,
                                  pa.max_evals - mp.n_evals());

                proc_map::value_type pmv(launch_cmd(cmdline));
                hpm_it->second.insert(pmv);

                // Logger
                logger().info("Generation: %d", gen_idx + 1);
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
                    if(logger().getLevel() >= opencog::Logger::FINE) {
                        logger().fine("Candidates with their bscores to merge with"
                                      " the metapopulation:");
                        stringstream ss;
                        logger().fine(mp.ostream(ss, candidates.begin(),
                                                 candidates.end(),
                                                 -1, true, true).str());
                    }
                    // ~Logger

                    mp.update_best_candidates(candidates);

                    // Logger
                    mp.log_best_candidates();
                    // ~Logger

                    mp.merge_candidates(candidates);

                    // Logger
                    logger().info("Metapopulation size is %u", mp.size());
                    if(logger().getLevel() >= opencog::Logger::FINE) {
                        stringstream ss;
                        ss << "Metapopulation after merging: " << std::endl;
                        logger().fine(mp.ostream(ss, -1, true, true).str());
                    }
                    logger().fine("Number of evaluations so far: %d",
                                  mp.n_evals());
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

#endif // _OPENCOG_DISTRIBUTED_MOSES_H
