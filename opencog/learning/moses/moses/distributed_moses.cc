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

#include "distributed_moses.h"

namespace opencog { namespace moses {

using namespace boost::program_options;
using boost::tuple;
using boost::make_tuple;
using boost::lexical_cast;

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

const string& get_hostname(const host_proc_map::value_type& hp) {
    return hp.first;
}
const proc_map& get_proc_map(const host_proc_map::value_type& hp) {
    return hp.second;
}
const unsigned get_total_jobs(const host_proc_map::value_type& hp) {
    // return the number of jobs for all processes of hp's host
    unsigned acc = 0;
    foreach(const proc_map::value_type& p, get_proc_map(hp))
        acc += get_num_jobs(p);
    return acc;
}

unsigned running_proc_count(const host_proc_map& hpm) {
    unsigned acc = 0;
    foreach(const host_proc_map::value_type& hpmv, hpm)
        acc += hpmv.second.size();
    return acc;
}

string build_cmdline(const variables_map& vm, 
                     const combo_tree& tr,
                     const string& host_name,
                     unsigned n_jobs,
                     unsigned max_evals,
                     unsigned gen_idx) {
    string res;
    if(host_name == localhost)
        res = "moses";
    else
        res = string("ssh ") + host_name + " 'moses";
    // replicate initial command's options, except all of those
    // interfering with the command to be built, that is:
    // exemplar, output options, jobs, max_evals, max_gens and
    // log file name options.
    for(variables_map::const_iterator it = vm.begin(); it != vm.end(); ++it) {
        if(it->first != exemplars_str_opt.first
           && it->first != exemplars_str_opt.first
           && it->first != output_score_opt.first
           && it->first != output_bscore_opt.first
           && it->first != output_complexity_opt.first
           && it->first != output_eval_number_opt.first
           && it->first != output_with_labels_opt.first
           && it->first != output_file_opt.first
           && it->first != jobs_opt.first
           && it->first != max_evals_opt.first
           && it->first != max_gens_opt.first
           && it->first != result_count_opt.first
           && it->first != log_file_opt.first
           && it->first != log_file_dep_opt_opt.first
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
    // add number of jobs option
    res += string(" -") + jobs_opt.second + lexical_cast<string>(n_jobs);
    // add number of evals option
    res += string(" -") + max_evals_opt.second + " " 
        + lexical_cast<string>(max_evals);
    // add one generation option
    res += string(" -") + max_gens_opt.second + " 1";
    // add log option determined name option
    res += string(" -") + log_file_opt.second + "distributed_moses"
        + "_from_parent_" + lexical_cast<string>(get_parent_pid())
        + "_gen_" + lexical_cast<string>(gen_idx) + ".log";
    // add option to return all results
    res += string(" -") + result_count_opt.second + " -1";

    if(host_name != localhost) {
        res += "'";
    }

    return res;
}

proc_map::value_type launch_cmd(string cmd, unsigned n_jobs) {
    // append " > tempfile&" to cmd
    char tempfile[] = "/tmp/mosesXXXXXX";
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
    return make_pair(pid, boost::make_tuple(cmd, tmp, fp, n_jobs));
}

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
            score_t score = lexical_cast<score_t>(s);
            // read complexity
            complexity_t complexity;
            in >> complexity;
            // read candidate
            combo_tree tr;
            in >> tr;
            // read bscore
            behavioral_score bscore;
            istreamContainer(in, std::back_inserter(bscore), "[", "]");
            // insert read element in candidates
            bscored_combo_tree candidate =
                make_pair(tr, make_pair(bscore, make_pair(score, complexity)));
            candidates.insert(candidate);
            // Logger
            if(logger().getLevel() >= Logger::FINE) {
                logger().fine("Parsed candidate:");
                stringstream ss;
                logger().fine(ostream_bscored_combo_tree(ss, candidate,
                                                         true, true).str());
            }
            // ~Logger
        }
    }
};
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

proc_map::iterator remove_proc(proc_map& pm,  proc_map::iterator it) {
    proc_map::iterator next_it(it);
    next_it++;
    pm.erase(it);
    return next_it;
}

void killall(proc_map& pm) {
    for(proc_map::iterator it = pm.begin(); it != pm.end();) {
        if(is_running(*it))
            kill(get_pid(*it), 15); // send a TERM signal
        it = remove_proc(pm, it);
    }
}

void killall(host_proc_map& hpm) {
    foreach(host_proc_map::value_type& hpmv, hpm)
        killall(hpmv.second);
}

host_proc_map::iterator find_free_resource(host_proc_map& hpm,
                                           const jobs_t& jobs) {
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

bool all_resources_free(const host_proc_map& hpm) {
    foreach(const host_proc_map::value_type& hpmv, hpm) {
        if(!hpmv.second.empty())
            return false;
    }
    return true;
}

} // ~namespace moses
} // ~namespace opencog
