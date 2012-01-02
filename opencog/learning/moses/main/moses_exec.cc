/** moses_exec.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
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

#include <boost/format.hpp>
#include <opencog/util/numeric.h>
#include <opencog/util/log_prog_name.h>

namespace opencog { namespace moses {

using boost::format;
using boost::str;

static const unsigned int max_filename_size = 255;

/**
 * Display error message about unspecified combo tree and exit
 */
void unspecified_combo_exit() {
    std::cerr << "error: you must specify which combo tree to learn (option -y)"
              << std::endl;
    exit(1);
}

/**
 * Display error message about unsupported type and exit
 */
void unsupported_type_exit(const type_tree& tt) {
    std::cerr << "error: type " << tt << " currently not supported" << std::endl;
    exit(1);
}
void unsupported_type_exit(type_node type) {
    unsupported_type_exit(type_tree(type));
}

/**
 * Display error message about ill formed combo tree and exit
 */
void illformed_exit(const combo_tree& tr) {
    std::cerr << "error: apparently the combo tree "
              << tr << "is not well formed" << std::endl;
    exit(1);
}

/**
 * Display error message about unsupported problem and exit
 */
void unsupported_problem_exit(const string& problem) {
    std::cerr << "error: problem " << problem
              << " unsupported for the moment" << std::endl;
    exit(1);
}

/**
 * Display error message about not recognized combo operator and exist
 */
void not_recognized_combo_operator(const string& ops_str) {
    std::cerr << "error: " << ops_str
              << " is not recognized as combo operator" << std::endl;
    exit(1);
}

/**
 * determine the initial exemplar of a given type
 */
combo_tree type_to_exemplar(type_node type) {
    switch(type) {
    case id::boolean_type: return combo_tree(id::logical_and);
    case id::contin_type: return combo_tree(id::plus);
    case id::ill_formed_type:
        std::cerr << "The data type is incorrect, perhaps it has not been"
                  << " possible to infer it from the input table." << std::endl;
        exit(1);
    default:
        unsupported_type_exit(type);
    }
    return combo_tree();
}

combo_tree ann_exemplar(combo::arity_t arity)
{
    combo_tree ann_tr(ann_type(0, id::ann));
    // ann root
    combo_tree::iterator root_node = ann_tr.begin();
    // output node
    combo_tree::iterator output_node =
        ann_tr.append_child(root_node, ann_type(1, id::ann_node));
    // input nodes
    for (combo::arity_t i = 0; i <= arity; ++i)
        ann_tr.append_child(output_node, ann_type(i + 2, id::ann_input));
    // input nodes' weights
    ann_tr.append_children(output_node, 0.0, arity + 1);
 
    return ann_tr;
}

/**
 * determine the alphabet size given the type_tree of the problem and
 * the of operator that are ignored
 */
int alphabet_size(const type_tree& tt, const vertex_set ignore_ops)
{
    combo::arity_t arity = type_tree_arity(tt);
    type_node output_type = *type_tree_output_type_tree(tt).begin();
    if (output_type == id::boolean_type) {
        return 3 + arity;
    } else if (output_type == id::contin_type) {
        // set alphabet size, 8 is roughly the number of operators
        // in contin formula, it will have to be adapted
        return 8 + arity - ignore_ops.size();
    } else if (output_type == id::ann_type) {
        return 2 + arity*arity; // to account for hidden neurones, very roughly
    } else {
        unsupported_type_exit(tt);
        return 0;
    }
}

// combo_tree
combo_tree str_to_combo_tree(const string& combo_str) {
    stringstream ss;
    combo_tree tr;
    ss << combo_str;
    ss >> tr;
    return tr;
}

// return true iff the problem is based on data file
bool data_based_problem(const string& problem) {
    return problem == it || problem == ann_it || problem == kl;
}

// return true iff the problem is based on a combo tree
bool combo_based_problem(const string& problem) {
    return problem == cp || problem == ann_cp;
}

// Infer the arity of the problem
combo::arity_t infer_arity(const string& problem,
                    unsigned int problem_size,
                    const string& input_table_file,
                    const string& combo_str)
{
    if (data_based_problem(problem))
        return dataFileArity(input_table_file);
    else if(combo_based_problem(problem))
    {
        if(combo_str.empty())
            unspecified_combo_exit();
        // get the combo_tree and infer its type
        combo_tree tr = str_to_combo_tree(combo_str);
        type_tree tt = infer_type_tree(tr);
        if(is_well_formed(tt)) {
            return type_tree_arity(tt);
        } else {
            illformed_exit(tr);
            return -1;
        }
    }
    else if (problem == pa || problem == dj)
    {
        return problem_size;
    }
    else if (problem == mux)
    {
        return problem_size + (1<<problem_size);
    }
    else if (problem == sr)
    {
        return 1;
    }
    else
    {
        unsupported_problem_exit(problem);
        return -1;
    }
}

int moses_exec(int argc, char** argv)
{
    // for(int i = 0; i < argc; ++i)
    //     std::cout << "arg = " << argv[i] << std::endl;

    // program options, see options_description below for their meaning
    unsigned long rand_seed;
    string input_data_file;
    string target_feature;
    string problem;
    string combo_str;
    unsigned int problem_size;
    int nsamples;
    float min_rand_input;
    float max_rand_input;
    unsigned long max_evals;
    long result_count;
    bool output_score;
    bool output_complexity;
    bool output_bscore;
    bool output_eval_number;
    bool output_with_labels;
    string output_file;
    int max_gens;
    string log_level;
    string log_file;
    bool log_file_dep_opt;
    float stdev;
    float prob;
    vector<string> include_only_ops_str;
    vector<string> ignore_ops_str;
    string opt_algo; //optimization algorithm
    vector<string> exemplars_str;
    int reduct_candidate_effort;
    int reduct_knob_building_effort;
    bool enable_cache;
    vector<string> jobs_str;
    bool weighted_accuracy;
    // metapop_param
    int max_candidates;
    bool reduce_all;
    bool revisit = false;
    bool include_dominated;
    // optim_param
    double pop_size_ratio;
    score_t max_score;
    double max_dist_ratio;
    // hc_param
    bool hc_terminate_if_improvement;
    // continuous optimization
    vector<contin_t> discretize_thresholds;

    // Declare the supported options.
    // XXX TODO: make this print correctly, instead of using brackets.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")
        (opt_desc_str(rand_seed_opt).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")
        (opt_desc_str(max_evals_opt).c_str(),
         value<unsigned long>(&max_evals)->default_value(10000),
         "Maximum number of fitness function evaluations.\n")
        (opt_desc_str(result_count_opt).c_str(),
         value<long>(&result_count)->default_value(10),
         "The number of non-dominated results to return, "
         "ordered according to score. If negative, then return all results.\n")
        (opt_desc_str(output_score_opt).c_str(),
         value<bool>(&output_score)->default_value(true),
         "If 1, outputs the score before each candidate (at the left of the complexity).\n")
        (opt_desc_str(output_complexity_opt).c_str(),
         value<bool>(&output_complexity)->default_value(false),
         "If 1, outputs the complexity before each candidate (at the right of the score).\n")
        (opt_desc_str(output_bscore_opt).c_str(),
         value<bool>(&output_bscore)->default_value(false),
         "If 1, outputs the bscore below each candidate.\n")
        (opt_desc_str(output_eval_number_opt).c_str(),
         value<bool>(&output_eval_number)->default_value(false),
         "If 1, outputs the actual number of evaluations.\n")
        (opt_desc_str(output_with_labels_opt).c_str(),
         value<bool>(&output_with_labels)->default_value(false),
         "If 1, outputs the candidates with their labels instead of place holders. for instance *(\"#price\" \"#temprature\") instead of *(#1 #2). This only works for data fitting problems where the data file contains labels in its header\n")
        (opt_desc_str(output_file_opt).c_str(),
         value<string>(&output_file)->default_value(""),
         "File where to save the results. If empty then it outputs on the stdout.\n")
        (opt_desc_str(max_gens_opt).c_str(),
         value<int>(&max_gens)->default_value(-1),
         "Maximum number of demes to generate and optimize, negative means no generation limit.\n")
        (opt_desc_str(input_data_file_opt).c_str(),
         value<string>(&input_data_file),
         "Input table file in CSV format. Colums correspond to features and the number of samples is taken to be the number of rows.\n")
        (opt_desc_str(target_feature_opt).c_str(),
         value<string>(&target_feature),
         "Label of the target feature to fit. If none is given the first one is used.\n")
        (opt_desc_str(problem_opt).c_str(),
         value<string>(&problem)->default_value(it),
         str(format("Problem to solve, supported problems are:\n"
                    "%s, regression based on input table\n"
                    "%s, search interesting patterns, where interestingness"
                    " is defined in terms of maximizing the Kullback-Leibler"
                    " divergence between the distribution of the outputs and"
                    " that same distribution in the context of the pattern"
                    " being true.\n"
                    "%s, regression based on input table using ann\n"
                    "%s, regression based on combo program\n"
                    "%s, even parity\n"
                    "%s, disjunction\n"
                    "%s, multiplex\n"
                    "%s, regression of f(x)_o = sum_{i={1,o}} x^i\n")
             % it % kl % ann_it % cp % pa % dj % mux % sr).c_str())
        (opt_desc_str(combo_str_opt).c_str(),
         value<string>(&combo_str),
         str(format("Combo program to learn, used when the problem"
                    " %s is selected (option -%s).\n")
             % cp % problem_opt.second).c_str())
        (opt_desc_str(problem_size_opt).c_str(),
         value<unsigned int>(&problem_size)->default_value(5),
         str(format("For even parity (%s), disjunction (%s) and multiplex (%s)"
                    " the problem size corresponds to the arity."
                    " For regression of f(x)_o = sum_{i={1,o}} x^i (%s)"
                    " the problem size corresponds to the order o.\n")
             % pa % dj % mux % sr).c_str())
        (opt_desc_str(nsamples_opt).c_str(),
         value<int>(&nsamples)->default_value(-1),
         "Number of samples to describe the problem. "
         "If nsample is negative, null or larger than the maximum "
         "number of samples allowed it is ignored. If the default "
         "problem size is larger than the value provided with that "
         "option then the dataset is subsampled randomly to reach the "
         "target size.\n")
        (opt_desc_str(min_rand_input_opt).c_str(),
         value<float>(&min_rand_input)->default_value(0),
         "Min of an input value chosen randomly, only used when the problem takes continuous inputs.\n")
        (opt_desc_str(max_rand_input_opt).c_str(),
         value<float>(&max_rand_input)->default_value(1),
         "Max of an input value chosen randomly, only used when the problem takes continuous inputs.\n")

        (opt_desc_str(log_level_opt).c_str(),
         value<string>(&log_level)->default_value("INFO"),
         "Log level, possible levels are NONE, ERROR, WARN, INFO, "
         "DEBUG, FINE. Case does not matter.\n")
        (opt_desc_str(log_file_dep_opt_opt).c_str(),
         str(format("The name of the log is determined by the options, for"
                    " instance if moses-exec is called with -%s 123 -%s %s"
                    " the log name is moses_random-seed_123_problem_pa.log."
                    " The name will be truncated in order not to"
                    " be longer than %s characters.\n")
             % rand_seed_opt.second % problem_opt.second % pa
             % max_filename_size).c_str())
        (opt_desc_str(log_file_opt).c_str(),
         value<string>(&log_file)->default_value(default_log_file),
         str(format("File name where to write the log."
                    " This option is overwritten by %s.\n")
             % log_file_dep_opt_opt.first).c_str())
        (opt_desc_str(stdev_opt).c_str(),
         value<float>(&stdev)->default_value(0),
         "In the case of contin regression. standard deviation of an assumed Gaussian around each candidate's output, useful if the data are noisy or to control an Occam's razor bias, 0 or negative means no Occam's razor, otherwise the higher the standard deviation the stronger the Occam's razor.\n")
        (opt_desc_str(prob_opt).c_str(),
         value<float>(&prob)->default_value(0),
         "In the case of boolean regression, probability that an output datum is wrong (returns false while it should return true or the other way around), useful if the data are noisy or to control an Occam's razor bias, only values 0 < p < 0.5 are meaningful, out of this range it means no Occam's razor, otherwise the greater p the greater the Occam's razor.\n")
        (opt_desc_str(include_only_ops_str_opt).c_str(),
         value<vector<string> >(&include_only_ops_str),
         "Include only the operator in the solution, can be used several times, for the moment only plus, times, div, sin, exp, log and variables (#n) are supported. Note that variables and operators are decoralated (including only some operators still include all variables and including only some variables still include all operators). You may need to put variables under double quotes. This option does not work with ANN.\n")
        (opt_desc_str(ignore_ops_str_opt).c_str(),
         value<vector<string> >(&ignore_ops_str),
         str(format("Ignore the following operator in the program solution,"
                    " can be used several times, for the moment only div,"
                    " sin, exp, log  and variables (#n) can be ignored."
                    " You may need to put variables under double quotes."
                    " This option has the priority over --%s."
                    " That is if an operator is both be included and ignored,"
                    " it is ignored. This option does not work with ANN.\n")
             % include_only_ops_str_opt.first).c_str())
        (opt_desc_str(opt_algo_opt).c_str(),
         value<string>(&opt_algo)->default_value(hc),
         str(format("Optimization algorithm, supported algorithms are"
                    " univariate (%s), simulation annealing (%s),"
                    " hillclimbing (%s).\n")
             % un % sa % hc).c_str())
        (opt_desc_str(exemplars_str_opt).c_str(),
         value<vector<string> >(&exemplars_str),
         "Start the search with a given exemplar, can be used several times.\n")
        (opt_desc_str(max_candidates_opt).c_str(),
         value<int>(&max_candidates)->default_value(-1),
         "Maximum number of considered candidates to be added to the metapopulation after optimizing deme.\n")
        (opt_desc_str(reduce_all_opt).c_str(),
         value<bool>(&reduce_all)->default_value(true),
         "Reduce all candidates before being evaluated.  Otherwise "
         "they are only reduced before being added to the "
         "metapopulation. This option can be valuable if memoization "
         "is enabled to avoid re-evaluate of duplicates.\n")
        (opt_desc_str(reduct_candidate_effort_opt).c_str(),
         value<int>(&reduct_candidate_effort)->default_value(2),
         "Effort allocated for reduction of candidates, in the range 0-3. "
         "0 means minimum effort, 3 means maximum effort.\n")
        (opt_desc_str(reduct_knob_building_effort_opt).c_str(),
         value<int>(&reduct_knob_building_effort)->default_value(2),
         "Effort allocated for reduction during knob building, 0-3, 0 means minimum effort, 3 means maximum effort. The bigger the effort the lower the dimension of the deme.\n")
        (opt_desc_str(enable_cache_opt).c_str(),
         value<bool>(&enable_cache)->default_value(true),
         "Memoize, that is, cache evaluation results, so that identical "
         "candidates are not re-evaluated. The cache size is dynamically "
         "adjusted to fit in the RAM.\n")
        (opt_desc_str(jobs_opt).c_str(),
         value<vector<string> >(&jobs_str),
         str(format("Number of jobs allocated for deme optimization."
                    " Jobs can be executed on a remote machine as well,"
                    " in such case the notation -%1% N:REMOTE_HOST is used,"
                    " where N is the number of jobs on the machine REMOTE_HOST."
                    " For instance one can enter the options"
                    " -%1%4 -%1%16%2%my_server.org"
                    " (or -%1%16%2%user@my_server.org if one wishes to"
                    " run the remote jobs under a different user name),"
                    " meaning that 4 jobs are allocated on the local machine"
                    " and 16 jobs are allocated on my_server.org."
                    " The assumption is that moses must be on the remote"
                    " machine and is located in a directory included in the"
                    " PATH environment variable. Beware that a lot of log"
                    " files are gonna be generated when using this option on"
                    " the remote machines.\n")
             % jobs_opt.second % job_seperator).c_str())
        (opt_desc_str(weighted_accuracy_opt).c_str(),
         value<bool>(&weighted_accuracy)->default_value(false),
         "This option is useful in case of unbalanced data as it "
         "weights the score so that each class weights equally "
         "regardless of their proportion in terms of sample size.\n")
        (opt_desc_str(pop_size_ratio_opt).c_str(),
         value<double>(&pop_size_ratio)->default_value(20),
         "The higher the more effort is spent on a deme.\n")
        (opt_desc_str(max_score_opt).c_str(),
         value<score_t>(&max_score)->default_value(best_score),
         "The max score to reach, once reached MOSES halts. MOSES is sometimes able to calculate the max score that can be reached for a particular problem, in such case the max_score is automatically reset of the minimum between MOSES's calculation and the user's option.\n")
        (opt_desc_str(max_dist_ratio_opt).c_str(),
         value<double>(&max_dist_ratio)->default_value(1),
         "The max distance from the exemplar to explore a deme is determined by that value * log2(information_theoretic_bits(deme)).\n")
        (opt_desc_str(include_dominated_opt).c_str(),
         value<bool>(&include_dominated)->default_value(false),
         "Include dominated candidates (according behavioral score) when merging candidates in the metapopulation. Faster merging but results in a very large metapopulation.\n")
        (opt_desc_str(hc_terminate_if_improvement_opt).c_str(),
         value<bool>(&hc_terminate_if_improvement)->default_value(true),
         "Hillclimbing parameter. If 1 then deme search terminates when an improvement is found, if 0 it keeps searching until another termination condition is met.\n")
        (opt_desc_str(discretize_threshold_opt).c_str(),
         value<vector<contin_t> >(&discretize_thresholds),
         "If the domain is continuous, discretize the target feature. A unique used of that option produces 2 classes, x < thresold and x >= threshold. The option can be used several times (n-1) to produce n classes and the thresholds are automatically sorted.\n")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    // set flags
    log_file_dep_opt = vm.count(log_file_dep_opt_opt.first) > 0;
 
    if (vm.count("help") || argc == 1) {
        cout << desc << "\n";
        return 1;
    }

    // set log
    if(log_file_dep_opt) {
        std::set<std::string> ignore_opt{log_file_dep_opt_opt.first};
        log_file = determine_log_name(default_log_file_prefix,
                                      vm, ignore_opt,
                                      std::string(".").append(default_log_file_suffix));
    }

    // remove log_file
    remove(log_file.c_str());
    logger().setFilename(log_file);
    logger().setLevel(logger().getLevelFromString(log_level));
    logger().setBackTraceLevel(Logger::ERROR);

    // init random generator
    MT19937RandGen rng(rand_seed);

    // infer arity
    combo::arity_t arity = infer_arity(problem, problem_size, input_data_file, combo_str);

    // Convert include_only_ops_str to the set of actual operators to
    // ignore.
    vertex_set ignore_ops;
    if (vm.count(include_only_ops_str_opt.first.c_str())) {
        bool ignore_arguments = false;
        bool ignore_operators = false;
        foreach (const string& s, include_only_ops_str) {
            vertex v;
            if (builtin_str_to_vertex(s, v)) {
                if(!ignore_operators) {
		  ignore_ops = {id::plus, id::times, id::div,
				id::exp, id::log, id::sin};
                    ignore_operators = true;
                }
                ignore_ops.erase(v);
            } else if (argument_str_to_vertex(s, v)) {
                if (!ignore_arguments) {
                    for (combo::arity_t arg = 1; arg <= arity; ++arg)
                        ignore_ops.insert(argument(arg));
                    ignore_arguments = true;
                }
                ignore_ops.erase(v);
            } else not_recognized_combo_operator(s);
        }
    }

    // Convert ignore_ops_str to the set of actual operators to ignore.
    foreach (const string& s, ignore_ops_str) {
        vertex v;
        if(builtin_str_to_vertex(s, v) || argument_str_to_vertex(s, v))
            ignore_ops.insert(v);
        else not_recognized_combo_operator(s);
    }

    // Set the initial exemplars.
    vector<combo_tree> exemplars;
    foreach(const string& exemplar_str, exemplars_str) {
        exemplars.push_back(str_to_combo_tree(exemplar_str));
    }

    // fill jobs
    jobs_t jobs{{localhost, 1}}; // by default the localhost has 1 job
    bool only_local = true;
    foreach(const string& js, jobs_str) {
        size_t pos = js.find(job_seperator);
        if(pos != string::npos) {
            unsigned int nj = boost::lexical_cast<unsigned int>(js.substr(0, pos));
            string host_name = js.substr(pos + 1);
            jobs[host_name] = nj;
            only_local = false;
        } else {
            jobs[localhost] = boost::lexical_cast<unsigned int>(js);
        }
    }

    // set metapopulation parameters
    metapop_parameters meta_params(max_candidates, reduce_all,
                                   revisit, include_dominated, jobs[localhost]);

    // set optim_parameters
    optim_parameters opt_params(pop_size_ratio, max_score, max_dist_ratio);

    // set moses_parameters
    moses_parameters moses_params(max_evals, max_gens, max_score, ignore_ops);

    // find the position of the target feature of the data file if any
    int target_pos = 0;
    if (!target_feature.empty() && !input_data_file.empty())
        target_pos = findTargetFeaturePosition(input_data_file, target_feature);

    // read labels on data file
    vector<string> labels;
    if (output_with_labels && !input_data_file.empty())
        labels = readInputLabels(input_data_file, target_pos);

    // set metapop_moses_results_parameters
    metapop_moses_results_parameters mmr_pa(vm, result_count,
                                            output_score, output_complexity,
                                            output_bscore, output_eval_number,
                                            output_with_labels, opt_algo,
                                            enable_cache, labels,
                                            output_file, jobs, only_local,
                                            hc_terminate_if_improvement);

    // Continuous reduction rules used during search and representation
    // building.
    const rule& contin_reduct = contin_reduction(ignore_ops, rng);

    // Logical reduction rules used during search.
    const rule& bool_reduct = logical_reduction(reduct_candidate_effort);

    // Logical reduction rules used during representation building.
    const rule& bool_reduct_rep = logical_reduction(reduct_knob_building_effort);

    // Problem based on input table.
    if (data_based_problem(problem)) {
        // infer the signature based on the input table
        type_tree table_tt = infer_data_type_tree(input_data_file);

        // read input_data_file file
        logger().debug("Read data file %s", input_data_file.c_str());
        Table table = istreamTable(input_data_file, target_pos);

        // possible subsample the table
        if (nsamples > 0)
            subsampleTable(table.itable, table.otable, nsamples, rng);

        if (problem == it) { // regression based on input table
 
            // infer the type of the input table
            type_tree table_output_tt = type_tree_output_type_tree(table_tt);
            type_node table_output_tn = *table_output_tt.begin();

            // determine the default exemplar to start with
            if(exemplars.empty())
                exemplars.push_back(type_to_exemplar(table_output_tn));

            type_node output_type = 
                *(get_output_type_tree(*exemplars.begin()->begin()).begin());
            if(output_type == id::unknown_type) {
                output_type = table_output_tn;
            }

            OC_ASSERT(output_type == table_output_tn);

            if(nsamples>0)
                subsampleTable(table.itable, table.otable, nsamples, rng);
        
            type_tree tt = gen_signature(output_type, arity);
            int as = alphabet_size(tt, ignore_ops);
        
            if(output_type == id::boolean_type) {
                CTable ctable = table.compress();
                occam_ctruth_table_bscore bscore(ctable, prob, as, rng);
                metapop_moses_results(rng, exemplars, tt,
                                      bool_reduct, bool_reduct_rep, bscore,
                                      opt_params, meta_params, moses_params,
                                      mmr_pa);
            }
            else if(output_type == id::contin_type) {
                if(discretize_thresholds.empty()) {
                    occam_contin_bscore bscore(table.otable, table.itable,
                                               stdev, as, rng);
                    metapop_moses_results(rng, exemplars, tt,
                                          contin_reduct, contin_reduct, bscore,
                                          opt_params, meta_params, moses_params,
                                          mmr_pa);
                } else {
                    occam_discretize_contin_bscore bscore(table.otable,
                                                          table.itable,
                                                          discretize_thresholds,
                                                          weighted_accuracy,
                                                          prob, as,
                                                          rng);
                    metapop_moses_results(rng, exemplars, tt,
                                          contin_reduct, contin_reduct, bscore,
                                          opt_params, meta_params, moses_params,
                                          mmr_pa);
                }
            } else {
                unsupported_type_exit(output_type);
            }
        } else if (problem == kl) { // find interesting patterns
            // it assumes that the inputs are boolean and the output is contin
            type_tree ettt = gen_signature(id::boolean_type,
                                           id::contin_type, arity);
            OC_ASSERT(ettt == table_tt,
                      "The input table doesn't have the right data types."
                      " The output should be contin and the inputs should"
                      " be boolean");
            // signature of the functions to learn
            type_tree tt = gen_signature(id::boolean_type, arity);

            // determine the default exemplar to start with
            if(exemplars.empty())
                exemplars.push_back(type_to_exemplar(id::boolean_type));

            int as = alphabet_size(tt, ignore_ops);

            occam_max_KLD_bscore bscore(table, stdev, as, rng);
            metapop_moses_results(rng, exemplars, tt,
                                  bool_reduct, bool_reduct_rep, bscore,
                                  opt_params, meta_params, moses_params, mmr_pa);
        }
        else if (problem == ann_it)
        { // regression based on input table using ann
            Table table = istreamTable(input_data_file, target_pos);
            // if no exemplar has been provided in option insert the default one
            if (exemplars.empty()) {
                exemplars.push_back(ann_exemplar(arity));
            }

            // subsample the table
            if(nsamples>0)
                subsampleTable(table.itable, table.otable, nsamples, rng);

            type_tree tt = gen_signature(id::ann_type, 0);
        
            int as = alphabet_size(tt, ignore_ops);

            occam_contin_bscore bscore(table.otable, table.itable, stdev, as, rng);
            metapop_moses_results(rng, exemplars, tt,
                                  ann_reduction(), ann_reduction(), bscore,
                                  opt_params, meta_params, moses_params, mmr_pa);
        }
    }

    // Demo/Example: Problem based on input combo program.
    // Learn a program that should be identical to the specified input
    // program.
    else if (combo_based_problem(problem))
    {
        combo_tree tr = str_to_combo_tree(combo_str);

        if (problem == cp) { // regression based on combo program
            // get the combo_tree and infer its type
            type_tree tt = infer_type_tree(tr);

            type_node output_type = *type_tree_output_type_tree(tt).begin();
            // if no exemplar has been provided in option, use the default one
            if (exemplars.empty()) {
                exemplars.push_back(type_to_exemplar(output_type));
            }
            if (output_type == id::boolean_type) {
                // @todo: Occam's razor and nsamples is not taken into account
                logical_bscore bscore(tr, arity);
                metapop_moses_results(rng, exemplars, tt,
                                      bool_reduct, bool_reduct_rep, bscore,
                                      opt_params, meta_params, moses_params,
                                      mmr_pa);
            }
            else if (output_type == id::contin_type) {
                // @todo: introduce some noise optionally
                if (nsamples <= 0)
                    nsamples = default_nsamples;

                ITable it(tt, rng, nsamples, max_rand_input, min_rand_input);
                OTable ot(tr, it, rng);

                int as = alphabet_size(tt, ignore_ops);

                occam_contin_bscore bscore(ot, it, stdev, as, rng);
                metapop_moses_results(rng, exemplars, tt,
                                      contin_reduct, contin_reduct, bscore,
                                      opt_params, meta_params, moses_params,
                                      mmr_pa);
            } else {
                unsupported_type_exit(tt);
            }
        }
        else if (problem == ann_cp)
        {
            // regression based on combo program using ann
            // get the combo_tree and infer its type
            type_tree tt = gen_signature(id::ann_type, 0);
            int as = alphabet_size(tt, ignore_ops);

            // if no exemplar has been provided in option use the default one
            if (exemplars.empty()) {
                exemplars.push_back(ann_exemplar(arity));
            }
 
            // @todo: introduce some noise optionally
            if (nsamples <= 0)
                nsamples = default_nsamples;

            ITable it(tt, rng, nsamples, max_rand_input, min_rand_input);
            OTable ot(tr, it, rng);
 
            occam_contin_bscore bscore(ot, it, stdev, as, rng);
            metapop_moses_results(rng, exemplars, tt,
                                  contin_reduct, contin_reduct, bscore,
                                  opt_params, meta_params, moses_params, mmr_pa);
        }
    }

    // Demo/Example: learn a combo program that determines if the
    // program inputs are even parity or not.  That is, the combo
    // program will be a boolean circuit that computes parity.
    else if (problem == pa)
    {
        even_parity func;

        // if no exemplar has been provided in option use the default
        // contin_type exemplar (and)
        if (exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = gen_signature(id::boolean_type, arity);
        logical_bscore bscore(func, arity);
        metapop_moses_results(rng, exemplars, tt,
                              bool_reduct, bool_reduct_rep, bscore,
                              opt_params, meta_params, moses_params, mmr_pa);
    }

    // Demo/example problem: learn the logical disjunction. That is,
    // moses should learn the following program: or(#1 #2 ... #k) where
    // k is the number of inputs specified by the -k option.
    else if (problem == dj)
    {
        // @todo: for the moment occam's razor and partial truth table are ignored
        disjunction func;

        // if no exemplar has been provided in option use the default
        // contin_type exemplar (and)
        if (exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = gen_signature(id::boolean_type, arity);
        logical_bscore bscore(func, arity);
        metapop_moses_results(rng, exemplars, tt,
                              bool_reduct, bool_reduct_rep, bscore,
                              opt_params, meta_params, moses_params, mmr_pa);
    }

    // Demo/example problem: multiplex. Learn the combo program that
    // corresponds to the boolean (electrical) circuit that is a
    // (de-)multiplexer.  That is, a k-bit binary address will specify
    // one and exactly one wire out of 2^k wires.  Here, k==problem_size.
    else if (problem == mux)
    {
        // @todo: for the moment occam's razor and partial truth table are ignored
        // arity = problem_size + 1<<problem_size
        multiplex func(problem_size);

        // if no exemplar has been provided in option use the default
        // contin_type exemplar (and)
        if (exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = gen_signature(id::boolean_type, arity);
        logical_bscore bscore(func, arity);
        metapop_moses_results(rng, exemplars, tt,
                              bool_reduct, bool_reduct_rep, bscore,
                              opt_params, meta_params, moses_params, mmr_pa);
    }

    // Demo/Example problem: polynomial regression.  Given the polynomial
    // p(x)=x+x^2+x^3+...x^k, this searches for the  shortest  program
    // consisting  of  nested arithmetic operators to compute p(x),
    // given x as a free variable.  So, for example the order-2 polynomial
    // can be written as x+x^2, and the shortest combo program is
    // *(+(1 #1) #1) (that is, the  solution is p(x)=x(x+1) in the usual
    // arithmetical notation).
    else if (problem == sr)
    { // simple regression of f(x)_o = sum_{i={1,o}} x^i
        // if no exemplar has been provided in option use the default
        // contin_type exemplar (+)
        if (exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::contin_type));
        }
 
        type_tree tt = gen_signature(id::contin_type, arity);

        ITable it(tt, rng, (nsamples>0 ? nsamples : default_nsamples));

        int as = alphabet_size(tt, ignore_ops);

        occam_contin_bscore bscore(simple_symbolic_regression(problem_size),
                                   it, stdev, as, rng);
        metapop_moses_results(rng, exemplars, tt,
                              contin_reduct, contin_reduct, bscore,
                              opt_params, meta_params, moses_params, mmr_pa);
    }
    else
       unsupported_problem_exit(problem);
    return 0;
}

int moses_exec(const vector<string>& argvs)
{
    char** argv = new char*[argvs.size()];
    for(size_t i = 0; i < argvs.size(); ++i) {
        argv[i] = const_cast<char*>(argvs[i].c_str());
    }
    int res = moses_exec(argvs.size(), argv);
    delete argv;
    return res;
}

} // ~namespace moses
} // ~namespace opencog
