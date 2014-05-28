/** distributed_moses.cc --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
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

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

#include <boost/program_options.hpp>
#include <ext/stdio_filebuf.h>

#include "distributed_moses.h"
#include "../main/moses_exec_def.h"

namespace opencog { namespace moses {

pid_t get_parent_pid() {
    return getpid();
}

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
unsigned get_num_jobs(const proc_map::value_type& pmv) {
    return pmv.second.get<3>();
}

const string& get_hostname(const host_proc_map::value_type& hp)
{
    return hp.first;
}

const proc_map& get_proc_map(const host_proc_map::value_type& hp)
{
    return hp.second;
}

const unsigned get_total_jobs(const host_proc_map::value_type& hp)
{
    // return the number of jobs for all processes of hp's host
    unsigned acc = 0;
    for (const proc_map::value_type& p : get_proc_map(hp))
        acc += get_num_jobs(p);
    return acc;
}

unsigned running_proc_count(const host_proc_map& hpm)
{
    unsigned acc = 0;
    for (const host_proc_map::value_type& hpmv : hpm)
        acc += hpmv.second.size();
    return acc;
}

string build_cmdline(const boost::program_options::variables_map& vm, 
                     const combo_tree& tr,
                     const string& host_name,
                     unsigned n_jobs,
                     unsigned max_evals,
                     unsigned gen_idx)
{
    namespace po = boost::program_options;
    string res;
    if (host_name == localhost)
        res = "moses";
    else
        res = string("ssh ") + host_name + " 'moses";
    // replicate initial command's options, except all of those
    // interfering with the command to be built, that is:
    // exemplar, output options, jobs, max_evals, max_gens and
    // log file name options.
    for (po::variables_map::const_iterator it = vm.begin(); it != vm.end(); ++it) {
        if (it->first != exemplars_str_opt.first
            and it->first != exemplars_str_opt.first
            and it->first != output_score_opt.first
            and it->first != output_bscore_opt.first
            and it->first != output_penalty_opt.first
            and it->first != output_eval_number_opt.first
            and it->first != output_with_labels_opt.first
            and it->first != output_file_opt.first
            and it->first != jobs_opt.first
            and it->first != max_evals_opt.first
            and it->first != max_gens_opt.first
            and it->first != result_count_opt.first
            and it->first != log_file_opt.first
            and it->first != log_file_dep_opt_opt.first
            and !it->second.defaulted())
         {
            string opt_name(" --");
            opt_name += it->first + " \"";
            res += opt_name + to_string(it->second, string("\"") + opt_name) + "\"";
        }
    }
    // add exemplar option
    stringstream ss;
    ss << tr;

    // When using ssh, must escape the $varname
    string trs = ss.str();
    size_t pos = trs.find('$');
    while (pos != string::npos) {
        trs.insert(pos, 1, '\\');
        pos = trs.find('$', pos+2);
    }
    res += string(" -") + exemplars_str_opt.second + " \"" + trs + "\"";
    // add output options
    res += string(" -") + output_bscore_opt.second + " 1";
    res += string(" -") + output_penalty_opt.second + " 1";
    res += string(" -") + output_eval_number_opt.second + " 1";
    // add number of jobs option
    res += string(" -") + jobs_opt.second + boost::lexical_cast<string>(n_jobs);
    // add number of evals option
    res += string(" -") + max_evals_opt.second + " " 
        + boost::lexical_cast<string>(max_evals);
    // add one generation option
    res += string(" -") + max_gens_opt.second + " 1";
    // add log option determined name option
    res += string(" -") + log_file_opt.second + "distributed_moses"
        + "_from_parent_" + boost::lexical_cast<string>(get_parent_pid())
        + "_gen_" + boost::lexical_cast<string>(gen_idx) + ".log";
    // add option to return all results
    res += string(" -") + result_count_opt.second + " -1";

    if (host_name != localhost) {
        res += "'";
    }

    return res;
}

proc_map::value_type launch_cmd(string cmd, unsigned n_jobs)
{
    // append " > tempfile&" to cmd
    char tempfile[] = "/tmp/mosesXXXXXX";
    int fd = mkstemp(tempfile);
    if (fd == -1) {
        std::cerr << "could not create temporary file" << std::endl;
        exit(1);
    }
    string tmp(tempfile);
    cmd += string(" > ") + tmp + "&";

    // launch the command
    int res = system(cmd.c_str());
    if (res != 0) {
        std::cerr << "the execution of " << cmd << "has failed" << std::endl;
        exit(1);        
    }
    FILE* fp = fdopen(fd, "r");

    // get its PID
    string exec_name = cmd.substr(0, cmd.find(" "));
    FILE* fp_pid = popen(string("pgrep -n ").append(exec_name).c_str(), "r");
    int pid;
    int count_matches = fscanf(fp_pid, "%u", &pid);
    OC_ASSERT(count_matches == 1);
    pclose(fp_pid);

    // make the proc_map
    return make_pair(pid, boost::make_tuple(cmd, tmp, fp, n_jobs));
}

bool is_being_written(const string& file_name, int pid)
{
    // Arghhh FIXME. fuser might not be installed, or it may not be in
    // the default search path.  RedHat/CentOS puts it into /sbin/fuser
    // which is not in the default searchpath.
    FILE* fp = popen(string("fuser ").append(file_name).append(" 2> /dev/null").c_str(), "r");
    while (!feof(fp)) {
        int p;
        int count_matches = fscanf(fp, "%u", &p);
        OC_ASSERT(count_matches == 1, "The fuser command failed; is it installed?");
        if (pid == p) {
            pclose(fp);
            return true;
        }
    }
    pclose(fp);
    return false;
}

bool is_running(const proc_map::value_type& pmv)
{
    // check if the file is still empty
    FILE* fp = get_file(pmv);
    if (fseek(fp, 0, SEEK_END)) {
        std::cerr << "Error while seeking to end of file" << std::endl;
        exit(1);
    }
    int length = ftell(fp);
    if (length < 0) {
        std::cerr << "Error while reading file position" << std::endl;
        exit(1);
    } else if (length == 0) {
        return true;
    }
    return is_being_written(get_tmp(pmv), get_pid(pmv));
}

void parse_result(istream& in, scored_combo_tree_set& candidates, int& evals)
{
    in.seekg(0);
    while (!in.eof()) {
        string s;
        in >> s;
        if (s.empty())
            continue;
        else if (s == string(number_of_evals_str).append(":")) {
            in >> evals;
        } else {
            // read score
            score_t score = boost::lexical_cast<score_t>(s);
            // read complexity
            complexity_t complexity;
            in >> complexity;
            // read complexity penalty
            score_t complexity_penalty;
            in >> complexity_penalty;
            // read complexity penalty
            score_t diversity_penalty;
            in >> diversity_penalty;
            // read candidate
            combo_tree tr;
            in >> tr;
            // read bscore, which is preceeded by the bscore penalty
            score_t bpenalty;
            in >> bpenalty;

            behavioral_score bscore;
            if (0 != bpenalty) {
                OC_ASSERT(bpenalty == complexity_penalty,
                          "Behavioural score is mangled!");
                istreamContainer(in, std::back_inserter(bscore), "[", "]");
            } else {
                // Discard remaining characters till end-of-line.
                in.ignore(4123123, '\n');
            }
            // insert read element in candidates
            composite_score cs(score, complexity, complexity_penalty);
            scored_combo_tree candidate =
                  scored_combo_tree(tr, demeID_t(), cs, bscore);
            candidates.insert(candidate);

            if (logger().isFineEnabled()) {
                logger().fine("Parsed candidate:");
                stringstream ss;
                ostream_scored_combo_tree(ss, candidate,
                              true, true);
                logger().fine(ss.str());
            }
        }
    }
};

void parse_result(const proc_map::value_type& pmv, 
                  scored_combo_tree_set& candidates, int& evals)
{
    FILE* fp = get_file(pmv);
    // build istream from fp
    __gnu_cxx::stdio_filebuf<char> pipe_buf(fp, ios_base::in);
    istream sp(&pipe_buf);
    // parse result
    parse_result(sp, candidates, evals);
    // close fp
    fclose(fp);

    // Don't clutter the file system with temp files.
    unlink(get_tmp(pmv).c_str());
}


host_proc_map init(const jobs_t& jobs)
{
    host_proc_map hpm;
    for (const jobs_t::value_type job : jobs) {
        hpm.insert(make_pair(job.first, proc_map()));
    }
    return hpm;
}

proc_map::iterator remove_proc(proc_map& pm,  proc_map::iterator it)
{
    proc_map::iterator next_it(it);
    next_it++;
    pm.erase(it);
    return next_it;
}

void killall(proc_map& pm)
{
    for(proc_map::iterator it = pm.begin(); it != pm.end();) {
        if(is_running(*it))
            kill(get_pid(*it), 15); // send a TERM signal
        it = remove_proc(pm, it);
    }
}

void killall(host_proc_map& hpm)
{
    for (host_proc_map::value_type& hpmv : hpm)
        killall(hpmv.second);
}

host_proc_map::iterator find_free_resource(host_proc_map& hpm,
                                           const jobs_t& jobs)
{
    host_proc_map::iterator hpm_it = hpm.begin();
    for(jobs_t::const_iterator jit = jobs.begin(); jit != jobs.end();
        ++jit, ++hpm_it) {
        OC_ASSERT(jit->first == hpm_it->first,
                  "Hosts names are different, there must be a bug");
        if(get_total_jobs(*hpm_it) < jit->second)
            break;
    }
    return hpm_it;
}

bool all_resources_free(const host_proc_map& hpm)
{
    for (const host_proc_map::value_type& hpmv : hpm) {
        if(!hpmv.second.empty())
            return false;
    }
    return true;
}

void distributed_moses(metapopulation& mp,
                       const moses_parameters& pa,
                       moses_statistics& stats)
{
    logger().info("Distributed MOSES starts");

    const boost::program_options::variables_map& vm = pa.vm;
    const jobs_t& jobs = pa.jobs;

    typedef scored_combo_tree_ptr_set_cit mp_cit;

    host_proc_map hpm = init(jobs);

    while ((stats.n_evals < pa.max_evals)
           && (pa.max_gens != stats.n_expansions) 
           && (mp.best_score() < pa.max_score))
    {
        // Launch processes as long as there are free resources.
        // We'll be greedy here, and launch as many as we can.
        host_proc_map::iterator hpm_it = find_free_resource(hpm, jobs);
        while (hpm_it != hpm.end()) {
            mp_cit exemplar = mp.select_exemplar();

            if (exemplar == mp.end()) {
                if (all_resources_free(hpm)) {
                    logger().warn("There are no more exemplars in the metapopulation "
                                  "that have not been visited and yet a solution was "
                                  "not found.  Perhaps reduction is too strict?");
                    goto theend;
                }

                // If there are no more exemplars in our pool, we will
                // have to wait for some more to come rolling in.
                break;
            }

            const string& hostname = get_hostname(*hpm_it);
            unsigned n_jobs = jobs.find(hostname)->second;
            const combo_tree& tr = exemplar->get_tree();

            string cmdline =
                build_cmdline(vm, tr, hostname, n_jobs,
                              pa.max_evals - stats.n_evals, stats.n_expansions);

            proc_map::value_type pmv(launch_cmd(cmdline, n_jobs));
            hpm_it->second.insert(pmv);

            stats.n_expansions++;
            logger().info() << "Generation: "<< stats.n_expansions;
            logger().info("Launch command: %s", get_cmd(pmv).c_str());
            logger().info("corresponding to PID = %d", get_pid(pmv));

            hpm_it = find_free_resource(hpm, jobs);
        }

        // Check for results and merge any that are found
        bool found_one = false;
        for (host_proc_map::value_type& hpmv : hpm) {
            for (proc_map::iterator it = hpmv.second.begin();
                it != hpmv.second.end(); )
            {
                if (is_running(*it)) {
                    it++;
                    continue;
                }

                found_one = true;

                // parse the result
                scored_combo_tree_set candidates;
                int evals;

                logger().info("Parsing results from PID %d file %s",
                    get_pid(*it), get_tmp(*it).c_str());

                parse_result(*it, candidates, evals);
                stats.n_evals += evals;

                // update best and merge
                mp.update_best_candidates(candidates);
                mp.merge_candidates(candidates);
                mp.log_best_candidates();

                // Remove proc info from pm
                it = remove_proc(hpmv.second, it);

                // Break out of loop, feed hungry mouths.
                break;
            }

            // Break out of loop, feed hungry mouths before
            // merging more results.
            if (found_one) break;
        }

        logger().fine("Number of running processes: %u", running_proc_count(hpm));

        if (!found_one) {
            // If we are here, everyone has been given work to do,
            // and no one has any results to report.
            // Wait for a second, so as not to take all resources.
            // A somewhat more elegant solution would be to have the
            // sender and receiver run in different threads, with the
            // sender waiting on a mutex lock for machines to free up.
            // But this solution more or less works, mostly because
            // most problems require more than a few seconds to perform
            // one evaluation.
            sleep(1);
        }
    }

theend:
    // kill all children processes
    killall(hpm);

    OC_ASSERT(running_proc_count(hpm) == 0);

    logger().info("Distributed MOSES ends");
}

} // ~namespace moses
} // ~namespace opencog
