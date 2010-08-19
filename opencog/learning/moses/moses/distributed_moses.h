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

#include <ext/stdio_filebuf.h>

#include <boost/program_options.hpp>

#include <opencog/util/iostreamContainer.h>

#include "../main/moses_exec_def.h"
#include "metapopulation.h"
#include "moses.h"

// uncomment to use popen() instead of system()
// #define USE_POPEN

namespace moses
{

using namespace boost::program_options;

// map the PID to the FILE of the output of the process and its command line
typedef std::pair<string, FILE*> string_file;
typedef std::map<int, string_file> proc_map;
int get_pid(const proc_map::value_type& pmv) {
    return pmv.first;
}
string get_cmd(const proc_map::value_type& pmv) {
    return pmv.second.first;
}
FILE* get_file(const proc_map::value_type& pmv) {
    return pmv.second.second;
}
// map the name of host and its proc_map
typedef std::map<string, proc_map> host_proc_map;

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
    for(variables_map::const_iterator it = vm.begin(); it != vm.end(); it++) {
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
    // add option to return all results
    res += string(" -") + result_count_opt.second + " -1";

    if(host_name != localhost) {
        res += "'";
    }

    // it seems ok so far, apparently the limit is 4096 not 255
    // OC_ASSERT(res.size() < 255,
    //           "It is unlikely the OS support such a long name %s, the only thing"
    //           " to do is upgrade the code so that it can express this command"
    //           " in less than 255 chars", res.c_str());

    return res;
}

// run the given command and return its corresponding proc_map
proc_map::value_type launch_cmd(string cmd) {
#ifndef USE_POPEN
    // append " > tempfile&" to cmd
    char tempfile[] = "/tmp/moses-execXXXXXX";
    int fd = mkstemp(tempfile);
    if(fd == -1) {
        std::cerr << "could not create temporary file" << std::endl;
        exit(1);
    }
    cmd += string(" > ") + string(tempfile) + string("&");
#endif

    // launch the command
#ifdef USE_POPEN
    FILE* fp = popen(cmd.c_str(), "r");
#else
    int res = system(cmd.c_str());
    if(res != 0) {
        std::cerr << "the execution of " << cmd << "has failed" << std::endl;
        exit(1);        
    }
    FILE* fp = fopen(tempfile, "r");
#endif

    // get its PID
    string exec_name = cmd.substr(0, cmd.find(" "));
    FILE* fp_pid = popen(string("pgrep -n ").append(exec_name).c_str(), "r");
    int pid;
    int count_matches = fscanf(fp_pid, "%u", &pid);
    OC_ASSERT(count_matches == 1);
    return make_pair(pid, make_pair(cmd, fp));
}

// check if a process is running, works only under linux or other UNIX
bool is_running(const proc_map::value_type& pmv) {
#ifdef USE_POPEN
    string path("/proc/");
    path += boost::lexical_cast<string>(get_pid(pmv));
    bool pid_file = access(path.c_str(), F_OK) != -1;
    if(pid_file) { // double check that it is the right PID by
                   // comparing the initial command line with the
                   // current one
        // path += "/cmdline";
        // ifstream clf;
        // sleep(10);
        // clf.open(path.c_str());
        // clf.seekg(0, ios::end);
        // int length = clf.tellg();
        // std::cout << length << std::endl;
        // clf.seekg(0, ios::beg);
        // char* cmdline = new char[length];
        // clf.read(cmdline, length);
        // for(int i = 0; i < length; i++) {
        //     char c = cmdline[i];
        //     cmdline[i] = (c == 0? 32 : c);
        // }
        // clf.close();
        // std::cout << cmdline << std::endl;
        // std::cout << "original|" << get_cmd(pmv) << "|" << std::endl;
        // // std::cout << "fromproc|" << cls << "|" << std::endl;        
        // // return get_cmd(pmv) == cls;
        return true;
    }
    return false;
#else
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
    }
    return length == 0;
#endif
}

/**
 * read the istream, add the candidates, fill max_evals
 */
void parse_result(istream& in, metapop_candidates& candidates, int& evals) {
#ifndef USE_POPEN
    in.seekg(0);
#endif
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


host_proc_map init(const jobs_t& jobs)
{
    host_proc_map hpm;
    foreach(const jobs_t::value_type job, jobs) {
        hpm.insert(make_pair(job.first, proc_map()));
    }
    return hpm;
}

host_proc_map::iterator find_free_resource(host_proc_map& hpm, const jobs_t& jobs)
{
    host_proc_map::iterator hpm_it = hpm.begin();
    for(jobs_t::const_iterator jit = jobs.begin(); jit != jobs.end();
        jit++, hpm_it++) {
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
                logger().info("Generation: %d", gen_idx);
                logger().info("Launch command: %s", cmdline.c_str());
                logger().info("corresponding to PID = %d", get_pid(pmv));
                // ~Logger

                gen_idx++;
            } else if(all_resources_free(hpm)) { // can't find any
                                             // available exemplar and
                                             // there is no hope that
                                             // one will come
                // Logger
                logger().info("There is no more exempalr in the metapopulation"
                              " that has not been visited and"
                              " no more results from other process are expected."
                              " This is a blockage situation, several options"
                              " can be used to prevent that, see moses-exec -h");
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
                    parse_result(*it, candidates, evals);
                    mp.n_evals() += evals;

                    // update best and merge
                    // Logger
                    logger().info("Merge %u candidates from PID %d"
                                  " with the metapopulation",
                                  candidates.size(), get_pid(*it));
                    // ~Logger

                    mp.update_best_candidates(candidates);

                    // Logger
                    mp.log_best_candidates();
                    // ~Logger

                    merge_nondominating(candidates.begin(), candidates.end(), mp);

                    // Logger
                    logger().info("Number of evaluations so far: %d",
                                  mp.n_evals());
                    // ~Logger

                    // remove proc info from pm
                    proc_map::iterator next_it(it);
                    next_it++;
                    hpmv.second.erase(it);
                    it = next_it;
                }
                else it++;
            }
        }

        // wait for a second to not take all resources
        sleep(1);
    }
    // Logger
    logger().info("Distributed MOSES ends");
    // ~Logger
}

} // ~namespace moses

#endif // _OPENCOG_DISTRIBUTED_MOSES_H
