/** moses-exec.cc --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
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

#include <opencog/util/numeric.h>

const unsigned int max_filename_size = 255;

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


ifstream* open_data_file(const string& file) {
    ifstream* in = new ifstream(file.c_str());
    if(!in->is_open()) {
        if(file.empty()) {
            std::cerr << "the input file is empty" << std::endl;
            std::cerr << "To indicate the file to open use the option"
                      << " -i or --input-file" << std::endl;
        } else {
            std::cerr << "Could not open " << file << std::endl;
        }
        exit(1);
    }
    return in;
}

/**
 * if the data file has a first row with labels
 */
vector<string> read_data_file_labels(const string& file) {
    auto_ptr<ifstream> in(open_data_file(file));
    std::string line;
    getline(*in, line);    
    return tokenizeRow<std::string>(line).first;
}

/**
 * check the token, if it is "0" or "1" then it is boolean, otherwise
 * it is contin. It is not 100% reliable of course and should be
 * improved.
 */
type_node infer_type_from_token(const string& token) {
    if(token == "0" || token == "1")
        return id::boolean_type;
    else {
        try {
            lexical_cast<contin_t>(token);
            return id::contin_type;
        }
        catch(...) {
            return id::ill_formed_type;
        }
    }    
}

/**
 * check the last element of the first or second row of a data file
 * according to infer_type_from_token.
 */
type_node infer_type_from_data_file(const string& file) {
    type_node res;
    auto_ptr<ifstream> in(open_data_file(file));
    string line;
    // check the last token of the first row
    getline(*in, line);
    res = infer_type_from_token(tokenizeRow<string>(line).second);
    if(res == id::ill_formed_type) { // check the last token if the second row
        getline(*in, line);
        res = infer_type_from_token(tokenizeRow<string>(line).second);
    }
    return res;
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

combo_tree ann_exemplar(arity_t arity) {
    combo_tree ann_tr(ann_type(0, id::ann));
    // ann root
    combo_tree::iterator root_node = ann_tr.begin();
    // output node
    combo_tree::iterator output_node =
        ann_tr.append_child(root_node, ann_type(1, id::ann_node));
    // input nodes
    for(arity_t i = 0; i <= arity; i++) 
        ann_tr.append_child(output_node, ann_type(i + 2, id::ann_input));
    // input nodes' weights
    ann_tr.append_children(output_node, 0.0, arity + 1);
    
    return ann_tr;
}

/**
 * determine the alphabet size given the type_tree of the problem and
 * the of operator that are ignored
 */
int alphabet_size(const type_tree& tt, const vertex_set ignore_ops) {
    arity_t arity = type_tree_arity(tt);
    type_node output_type = *type_tree_output_type_tree(tt).begin();
    if(output_type == id::boolean_type) {
        return 3 + arity;
    } else if(output_type == id::contin_type) {
        // set alphabet size, 8 is roughly the number of operators
        // in contin formula, it will have to be adapted
        return 8 + arity - ignore_ops.size();
    } else if(output_type == id::ann_type) {
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

// infer the arity of the problem
arity_t infer_arity(const string& problem,                    
                    unsigned int problem_size,
                    const string& input_table_file,
                    const string& combo_str) {
    if(problem == it || problem == ann_it) {
        auto_ptr<ifstream> in(open_data_file(input_table_file));
        return istreamArity(*in);
    } else if(problem == cp || problem == ann_cp) {
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
    } else if(problem == pa || problem == dj || problem == mux) {
        return problem_size;
    } else if(problem == sr) {
        return 1;
    } else {
        unsupported_problem_exit(problem);
        return -1;
    }
}

// returns n such that a = n+2^n
arity_t multiplex_arity(arity_t a) {
    for(unsigned int n = 1; n <= integer_log2(a); n++)
        if(n+pow2(n) == (unsigned int)a)
            return n;
    // not found, exit
    std::cerr << "error: for multiplex the arity " << a 
              << " must be equal to n+2^n, but there is no such n" << std::endl;
    exit(1);
    return -1;
}

int moses_exec(int argc, char** argv) { 
    // for(int i = 0; i < argc; i++)
    //     std::cout << "arg = " << argv[i] << std::endl;

    // program options, see options_description below for their meaning
    unsigned long rand_seed;
    string input_data_file;
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
    bool output_score_complexity_old_moses;
    bool output_bscore;
    bool output_eval_number;
    bool output_with_labels;
    string output_file;
    int max_gens;
    string log_level;
    string log_file;
    bool log_file_dep_opt;
    float variance;
    float prob;
    vector<string> include_only_ops_str;
    vector<string> ignore_ops_str;
    string opt_algo; //optimization algorithm
    vector<string> exemplars_str;
    int reduct_candidate_effort;
    int reduct_knob_building_effort;
    unsigned long cache_size;
    vector<string> jobs_str;
    double fs_intensity;
    unsigned int fs_target_size;
    double rf_intensity;
    unsigned int fs_interaction_terms;
    // metapop_param
    int max_candidates;
    bool reduce_all;
    bool count_base; // true if the scorer is count based, otherwise
                     // complexity based
    bool revisit;
    bool ignore_bscore;
    // optim_param
    double pop_size_ratio;
    double max_score;

    // Declare the supported options.
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
         "The number of non-dominated best results to return ordered according to their score, if negative then returns all of them.\n")
        (opt_desc_str(output_score_opt).c_str(),
         value<bool>(&output_score)->default_value(true),
         "If 1, outputs the score before each candidate (at the left of the complexity).\n")
        (opt_desc_str(output_complexity_opt).c_str(),
         value<bool>(&output_complexity)->default_value(false),
         "If 1, outputs the complexity before each candidate (at the right of the score).\n")
        (opt_desc_str(output_score_complexity_old_moses_opt).c_str(),
         value<bool>(&output_score_complexity_old_moses)->default_value(false),
         string("If 1, outputs the complexity and the score according to a previous version of MOSES (for compatibility issue). This option overwrites ").
         append(output_score_opt.first).append(" and ").
         append(output_complexity_opt.first).append(".\n").c_str())
        (opt_desc_str(output_bscore_opt).c_str(),
         value<bool>(&output_bscore)->default_value(false),
         "If 1, outputs the bscore below each candidate.\n")
        (opt_desc_str(output_eval_number_opt).c_str(),
         value<bool>(&output_eval_number)->default_value(false),
         "If 1, outputs the actual number of evaluations.\n")
        (opt_desc_str(output_with_labels_opt).c_str(),
         value<bool>(&output_with_labels)->default_value(false),
         "If 1, outputs the candidates with their labels instead of place holders. for instance *(\"price\" \"temprature\") instead of *(#1 #2). This only works for data fitting problems where the data file contains labels in its header\n")
        (opt_desc_str(output_file_opt).c_str(),
         value<string>(&output_file)->default_value(""),
         "File where to save the results. If empty then it outputs on the stdout.")
        (opt_desc_str(max_gens_opt).c_str(),
         value<int>(&max_gens)->default_value(-1),
         "Maximum number of demes to generate and optimize, negative means no generation limit.\n")
        (opt_desc_str(input_data_file_opt).c_str(),
         value<string>(&input_data_file),
         "Input table file, the maximum number of samples is the number of rows in the file.\n")
        (opt_desc_str(problem_opt).c_str(),
         value<string>(&problem)->default_value("it"),
         string("Problem to solve, supported problems are"
                " regression based on input table (").append(it).
         append("), regression based on input table using ann (").append(ann_it).
         append("), regression based on combo program (").append(cp).
         append("), even parity (").append(pa).
         append("), disjunction (").append(dj).
         append("), multiplex (").append(mux).
         append("), regression of f(x)_o = sum_{i={1,o}} x^i (").append("sr").
         append(").\n").c_str())
        (opt_desc_str(combo_str_opt).c_str(),
         value<string>(&combo_str),
         string("Combo program to learn, use when the problem ").append(cp).append(" is selected (option -").append(problem_opt.second).append(").\n").c_str())
        (opt_desc_str(problem_size_opt).c_str(),
         value<unsigned int>(&problem_size)->default_value(5),
         string("For even parity (").append(pa).
         append("), disjunction (").append(dj).
         append(") and multiplex (").append(mux).
         append(") the problem size corresponds to the arity.").
         append(" Note that for multiplex (").append(mux).
         append(") the problem size must be equal to n+2^n.").
         append(" For regression of f(x)_o = sum_{i={1,o}} x^i (").append(sr).
         append(") the problem size corresponds to the order o.\n").c_str())
        (opt_desc_str(nsamples_opt).c_str(),
         value<int>(&nsamples)->default_value(-1),
         "Number of samples to describ the problem. If nsample is negative, null or larger than the maximum number of samples allowed it is ignored.\n")
        (opt_desc_str(min_rand_input_opt).c_str(),
         value<float>(&min_rand_input)->default_value(0),
         "Min of an input value chosen randomly, only used when the problem takes continuous inputs.\n")
        (opt_desc_str(max_rand_input_opt).c_str(),
         value<float>(&max_rand_input)->default_value(1),
         "Max of an input value chosen randomly, only used when the problem takes continuous inputs.\n")
        (opt_desc_str(log_level_opt).c_str(),
         value<string>(&log_level)->default_value("DEBUG"),
         "Log level, possible levels are NONE, ERROR, WARN, INFO, DEBUG, FINE. Case does not matter.\n")
        (opt_desc_str(log_file_dep_opt_opt).c_str(),
         string("The name of the log is determined by the options, for instance if moses-exec is called with -r 123 -H pa the log name is moses_random-seed_123_problem_pa.log. Note that the name will be truncated in order not to be longer than ").append(lexical_cast<string>(max_filename_size)).append(" characters.\n").c_str())
        (opt_desc_str(log_file_opt).c_str(),
         value<string>(&log_file)->default_value(default_log_file),
         string("File name where to write the log. This option is overwritten by ").append(log_file_dep_opt_opt.first).append(".\n").c_str())
        (opt_desc_str(variance_opt).c_str(),
         value<float>(&variance)->default_value(0),
         "In the case of contin regression. variance of an assumed Gaussian around each candidate's output, useful if the data are noisy or to control an Occam's razor bias, 0 or negative means no Occam's razor, otherwise the higher v the stronger the Occam's razor.\n")
        (opt_desc_str(prob_opt).c_str(),
         value<float>(&prob)->default_value(0),
         "In the case of boolean regression, probability that an output datum is wrong (returns false while it should return true or the other way around), useful if the data are noisy or to control an Occam's razor bias, only values 0 < p < 0.5 are meaningful, out of this range it means no Occam's razor, otherwise the greater p the greater the Occam's razor.\n")
        (opt_desc_str(include_only_ops_str_opt).c_str(),
         value<vector<string> >(&include_only_ops_str),
         "Include only the operator in the solution, can be used several times, for the moment only plus, times, div, sin, exp, log and variables (#n) are supported. Note that variables and operators are decoralated (including only some operators still include all variables and including only some variables still include all operators). You may need to put variables under double quotes. This option does not work with ANN.\n")
        (opt_desc_str(ignore_ops_str_opt).c_str(),
         value<vector<string> >(&ignore_ops_str),
         string("Ignore the following operator in the program solution, can be used several times, for the moment only div, sin, exp, log and variables (#n) can be ignored. You may need to put variables under double quotes. This option has the priority over ").append(include_only_ops_str_opt.first).append(". That is if an operator is both be included and ignored, it is ignored. This option does not work with ANN.\n").c_str())
        (opt_desc_str(opt_algo_opt).c_str(),
         value<string>(&opt_algo)->default_value(hc),
         string("Optimization algorithm, supported algorithms are"
                " univariate (").append(un).
         append("), simulation annealing (").append(sa).
         append("), hillclimbing (").append(hc).append(").\n").c_str())
        (opt_desc_str(exemplars_str_opt).c_str(),
         value<vector<string> >(&exemplars_str),
         "Start the search with a given exemplar, can be used several times.\n")
        (opt_desc_str(max_candidates_opt).c_str(),
         value<int>(&max_candidates)->default_value(-1),
         "Maximum number of considered candidates to be added to the metapopulation after optimizing deme.\n")
        (opt_desc_str(reduce_all_opt).c_str(),
         value<bool>(&reduce_all)->default_value(true),
         "Reduce all candidates before being evaluated, otherwise there are only reduced before being added to the metapopulation. This option can be valuable if the cache is enabled to not re-evaluate duplicates.\n")
        (opt_desc_str(reduct_candidate_effort_opt).c_str(),         
         value<int>(&reduct_candidate_effort)->default_value(2),
         "Effort allocated for reduction of candidates, 0-3, 0 means minimum effort, 3 means maximum effort.\n")
        (opt_desc_str(reduct_knob_building_effort_opt).c_str(),
         value<int>(&reduct_knob_building_effort)->default_value(2),
         "Effort allocated for reduction during knob building, 0-3, 0 means minimum effort, 3 means maximum effort. The bigger the effort the lower the dimension of the deme.\n")
        (opt_desc_str(count_base_opt).c_str(),
         value<bool>(&count_base)->default_value(false),
         "If 1 then a count based scorer is used, otherwise, if 0, a complexity based scorer is used.\n")
        (opt_desc_str(cache_size_opt).c_str(),
         value<unsigned long>(&cache_size)->default_value(1000000),
         "Cache size, so that identical candidates are not re-evaluated, 0 means no cache.\n")
        (opt_desc_str(revisit_opt).c_str(),
         "Revisit visited examplars when all have been visited.\n")
        (opt_desc_str(jobs_opt).c_str(),
         value<vector<string> >(&jobs_str),
         string("Number of jobs allocated for deme optimization. Jobs can be executed on a remote machine as well, in such case the notation -j N:REMOTE_HOST is used. For instance one can enter the options -j 4 -j 16").append(job_seperator).append("my_server.org (or -j 16").append(job_seperator).append("user@my_server.org if wishes to run the remote jobs under a different user name), meaning that 4 jobs are allocated on the local machine and 16 jobs are allocated on my_server.org. The assumption is that moses-exec must be on the remote machine and is located in a directory included in the PATH environment variable. Beware that a lot of log files are gonna be generated when using this option.\n").c_str())
        (opt_desc_str(feature_selection_intensity_opt).c_str(),
         value<double>(&fs_intensity)->default_value(0),
         "Value between 0 and 1. 0 means all features are selected, 1 corresponds to the stronger selection pressure, probably no features are selected at 1.\n")
        (opt_desc_str(feature_selection_target_size_opt).c_str(),
         value<unsigned int>(&fs_target_size)->default_value(0),
         "The number of features to attempt to select. This option overwrites feature-selection-intensity. 0 means disabled.\n")
        (opt_desc_str(redundant_feature_intensity_opt).c_str(),
         value<double>(&rf_intensity)->default_value(0.1),
         "Value between 0 and 1. 0 means no redundant features are discarded, 1 means redudant features are maximally discarded. This option is only active when feature selection is active.\n")
        (opt_desc_str(feature_selection_interaction_terms_opt).c_str(),
         value<unsigned int>(&fs_interaction_terms)->default_value(1),
         "Maximum number of interaction terms considered during feature selection. Higher values make the feature selection more accurate but is computationally expensive.\n")
        (opt_desc_str(pop_size_ratio_opt).c_str(),
         value<double>(&pop_size_ratio)->default_value(200),
         "The higher the more effort is spent on a deme.\n")
        (opt_desc_str(max_score_opt).c_str(),
         value<double>(&max_score)->default_value(0),
         "The max score to reach, once reached MOSES halts.\n")
        (opt_desc_str(ignore_bscore_opt).c_str(),
         value<bool>(&ignore_bscore)->default_value(false),
         "Ignore the behavioral score when merging candidates in the population. This option is useful either when the problem has no obvious behavioral score, or it happens that dominated candidates worth keeping.\n")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    // set flags
    revisit = vm.count(revisit_opt.first) > 0;
    log_file_dep_opt = vm.count(log_file_dep_opt_opt.first) > 0;
    
    if (vm.count("help") || argc == 1) {
        cout << desc << "\n";
        return 1;
    }

    // set log
    if(log_file_dep_opt) {
        // determine the name of the log depending on the program options
        log_file = default_log_file_prefix;
        for(variables_map::const_iterator it = vm.begin(); it != vm.end(); it++)
            // we ignore the option log_file_dep_opt and any default one
            if(it->first != log_file_dep_opt_opt.first && !it->second.defaulted()) {
                string str = string("_") + it->first + "_" + to_string(it->second);
                // this is because OSs usually do not handle file name
                // above 255 chars
                unsigned int expected_max_size =
                    log_file.size()+str.size()+default_log_file_suffix.size()+1;
                if(expected_max_size < max_filename_size) {
                    log_file += str;
                }
            }
        log_file += string(".") + default_log_file_suffix;
        // replace / by d because unix file name cannot have / in it
        replace(log_file.begin(), log_file.end(), '/', 'd');
        OC_ASSERT(log_file.size() <= max_filename_size);
    }
    // remove log_file
    remove(log_file.c_str());
    logger().setFilename(log_file);
    logger().setLevel(logger().getLevelFromString(log_level));
    logger().setBackTraceLevel(Logger::ERROR);
    // init random generator
    MT19937RandGen rng(rand_seed);

    // infer arity
    arity_t arity = infer_arity(problem, problem_size, input_data_file, combo_str);

    // convert include_only_ops_str to the set of actual operators to
    // ignore
    vertex_set ignore_ops;
    if(vm.count(include_only_ops_str_opt.first.c_str())) {
        bool ignore_arguments = false;
        bool ignore_operators = false;
        foreach(const string& s, include_only_ops_str) {
            vertex v;
            if(builtin_str_to_vertex(s, v)) {
                if(!ignore_operators) {
                    ignore_ops =
                        list_of(id::plus)(id::times)(id::div)(id::exp)(id::log)(id::sin);
                    ignore_operators = true;
                }
                ignore_ops.erase(v);
            } else if(argument_str_to_vertex(s, v)) {
                if(!ignore_arguments) {
                    for(arity_t arg = 1; arg <= arity; arg++)
                        ignore_ops.insert(argument(arg));
                    ignore_arguments = true;
                }
                ignore_ops.erase(v);
            } else not_recognized_combo_operator(s);
        }
    }

    // convert ignore_ops_str to the set of actual operators to ignore
    foreach(const string& s, ignore_ops_str) {
        vertex v;
        if(builtin_str_to_vertex(s, v) || argument_str_to_vertex(s, v))
            ignore_ops.insert(v);
        else not_recognized_combo_operator(s);
    }

    // set the initial exemplars
    vector<combo_tree> exemplars;
    foreach(const string& exemplar_str, exemplars_str) {
        exemplars.push_back(str_to_combo_tree(exemplar_str));
    }

    // fill jobs
    jobs_t jobs;
    foreach(const string& js, jobs_str) {
        size_t pos = js.find(job_seperator);
        if(pos != string::npos) {
            unsigned int nj = boost::lexical_cast<unsigned int>(js.substr(0, pos));
            string host_name = js.substr(pos + 1);
            jobs.insert(make_pair(host_name, nj));
        } else {
            jobs.insert(make_pair(localhost, boost::lexical_cast<unsigned int>(js)));
        }
    }

    // set metapopulation parameters
    metapop_parameters meta_params(max_candidates, reduce_all,
                                   count_base, revisit, ignore_bscore);

    // set optim_parameters
    optim_parameters opt_params(pop_size_ratio, max_score);

    // set moses_parameters
    moses_parameters moses_params(max_evals, max_gens, max_score, ignore_ops);

    // read labels on data file
    vector<string> labels;
    if(output_with_labels && !input_data_file.empty())
        labels = read_data_file_labels(input_data_file);

    // set metapop_moses_results_parameters
    metapop_moses_results_parameters mmr_pa(result_count,
                                            output_score, output_complexity,
                                            output_score_complexity_old_moses,
                                            output_bscore, output_eval_number,
                                            output_with_labels, labels,
                                            output_file, jobs);

    if(problem == it) { // regression based on input table
        
        // try to infer the type of the input table
        type_node inferred_type = infer_type_from_data_file(input_data_file);
        if(exemplars.empty()) {            
            exemplars.push_back(type_to_exemplar(inferred_type));
        }

        type_node output_type = 
            *(get_output_type_tree(*exemplars.begin()->begin()).begin());
        if(output_type == id::unknown_type) {
            output_type = inferred_type;
        }

        OC_ASSERT(output_type == inferred_type);

        auto_ptr<ifstream> in(open_data_file(input_data_file));

        if(output_type == id::boolean_type) {
            // read input_data_file file
            truth_table_inputs it;
            partial_truth_table ot;
            istreamTable<truth_table_inputs,
                         partial_truth_table, bool>(*in, it, ot);
            it.set_ignore_args(ignore_ops); // to speed up binding
            if(nsamples>0)
                subsampleTable(it, ot, nsamples, rng);

            // feature selection
            if(fs_intensity > 0 || fs_target_size > 0) {
                // Logger
                logger().info("Computing feature selection");
                // ~Logger

                typedef ConditionalEntropy<truth_table_inputs,
                                           partial_truth_table,
                                           std::set<arity_t> > FeatureScorer;
                FeatureScorer fs(it, ot);
                std::set<arity_t> features = it.get_considered_args_from_zero();
                std::set<arity_t> selected_features = 
                    fs_target_size > 0?
                    cached_adaptive_incremental_selection(features, fs,
                                                          fs_target_size,
                                                          fs_interaction_terms,
                                                          rf_intensity)
                    : cached_incremental_selection(features, fs,
                                                   fs_intensity,
                                                   fs_interaction_terms,
                                                   rf_intensity);

                if(selected_features.empty()) {
                    std::cerr << "No features have been selected. Please retry with a lower feature selection intensity" << std::endl;
                    exit(1);
                } else {
                    it.set_consider_args_from_zero(selected_features);
                    vertex_set ignore_args = it.get_ignore_args();
                    moses_params.ignore_ops.insert(ignore_args.begin(),
                                                   ignore_args.end());
                    
                    // Logger
                    vertex_set selected_args = it.get_considered_args();
                    logger().info("%u inputs have been selected", selected_args.size());
                    stringstream ss;
                    ostreamContainer(ss, it.get_considered_args());
                    logger().info(ss.str());
                    // ~Logger
                }
            }

            type_tree tt = declare_function(output_type, arity);

            int as = alphabet_size(tt, ignore_ops);

            occam_truth_table_bscore bscore(ot, it, prob, as, rng);
            metapop_moses_results(rng, exemplars, tt,
                                  logical_reduction(reduct_candidate_effort),
                                  logical_reduction(reduct_knob_building_effort),
                                  bscore, cache_size, opt_algo,
                                  opt_params, meta_params, moses_params,
                                  vm, mmr_pa);
        }
        else if(output_type == id::contin_type) {
            // read input_data_file file
            contin_input_table it;
            contin_table ot;
            istreamTable<contin_input_table,
                         contin_table, contin_t>(*in, it, ot);
            it.set_ignore_args(ignore_ops); // to speed up binding
            if(nsamples>0)
                subsampleTable(it, ot, nsamples, rng);

            type_tree tt = declare_function(output_type, arity);
            int as = alphabet_size(tt, ignore_ops);

            // if no exemplar has been provided in option use the default
            // contin_type exemplar (+)
            if(exemplars.empty()) {            
                exemplars.push_back(type_to_exemplar(id::contin_type));
            }

            occam_contin_bscore bscore(ot, it, variance, as, rng);
            metapop_moses_results(rng, exemplars, tt,
                                  contin_reduction(ignore_ops, rng, 1000000),
                                  contin_reduction(ignore_ops, rng, 1000000),
                                  bscore, cache_size, opt_algo,
                                  opt_params, meta_params, moses_params,
                                  vm, mmr_pa);
        } else {
            unsupported_type_exit(output_type);
        }
    } else if(problem == cp) { // regression based on combo program
        // get the combo_tree and infer its type
        combo_tree tr = str_to_combo_tree(combo_str);
        type_tree tt = infer_type_tree(tr);

        type_node output_type = *type_tree_output_type_tree(tt).begin();
        // if no exemplar has been provided in option use the default one
        if(exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(output_type));
        }
        if(output_type == id::boolean_type) {
            // @todo: Occam's razor and nsamples is not taken into account
            logical_bscore bscore(tr, arity);
            metapop_moses_results(rng, exemplars, tt,
                                  logical_reduction(reduct_candidate_effort),
                                  logical_reduction(reduct_knob_building_effort),
                                  bscore, cache_size, opt_algo,
                                  opt_params, meta_params, moses_params,
                                  vm, mmr_pa);                
        }
        else if (output_type == id::contin_type) {
            // @todo: introduce some noise optionally
            if(nsamples<=0)
                nsamples = default_nsamples;
            
            contin_input_table it(nsamples, arity, rng,
                                           max_rand_input, min_rand_input);
            it.set_ignore_args(ignore_ops); // to speed up binding
            contin_table table_outputs(tr, it, rng);
            
            int as = alphabet_size(tt, ignore_ops);
            
            occam_contin_bscore bscore(table_outputs, it,
                                       variance, as, rng);
            metapop_moses_results(rng, exemplars, tt,
                                  contin_reduction(ignore_ops, rng, 1000000),
                                  contin_reduction(ignore_ops, rng, 1000000),
                                  bscore, cache_size, opt_algo,
                                  opt_params, meta_params, moses_params,
                                  vm, mmr_pa);
        } else {
            unsupported_type_exit(tt);
        }
    } else if(problem == pa) { // even parity
        // @todo: for the moment occam's razor and partial truth table are ignored
        even_parity func;

        // if no exemplar has been provided in option use the default
        // contin_type exemplar (and)
        if(exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = declare_function(id::boolean_type, arity);
        logical_bscore bscore(func, arity);
        metapop_moses_results(rng, exemplars, tt,
                              logical_reduction(reduct_candidate_effort),
                              logical_reduction(reduct_knob_building_effort),
                              bscore, cache_size, opt_algo,
                              opt_params, meta_params, moses_params,
                              vm, mmr_pa);
    } else if(problem == dj) { // disjunction
        // @todo: for the moment occam's razor and partial truth table are ignored
        disjunction func;

        // if no exemplar has been provided in option use the default
        // contin_type exemplar (and)
        if(exemplars.empty()) {            
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = declare_function(id::boolean_type, arity);
        logical_bscore bscore(func, arity);
        metapop_moses_results(rng, exemplars, tt,
                              logical_reduction(reduct_candidate_effort),
                              logical_reduction(reduct_knob_building_effort),
                              bscore, cache_size, opt_algo,
                              opt_params, meta_params, moses_params,
                              vm, mmr_pa);
    } else if(problem == mux) { // multiplex
        // @todo: for the moment occam's razor and partial truth table are ignored
        multiplex func(multiplex_arity(arity));

        // if no exemplar has been provided in option use the default
        // contin_type exemplar (and)
        if(exemplars.empty()) {            
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = declare_function(id::boolean_type, arity);
        logical_bscore bscore(func, arity);
        metapop_moses_results(rng, exemplars, tt,
                              logical_reduction(reduct_candidate_effort),
                              logical_reduction(reduct_knob_building_effort),
                              bscore, cache_size, opt_algo,
                              opt_params, meta_params, moses_params,
                              vm, mmr_pa);
    } else if(problem == sr) { // simple regression of f(x)_o = sum_{i={1,o}} x^i
        // if no exemplar has been provided in option use the default
        // contin_type exemplar (+)
        if(exemplars.empty()) {            
            exemplars.push_back(type_to_exemplar(id::contin_type));
        }
        
        type_tree tt = declare_function(id::contin_type, arity);

        contin_input_table rands((nsamples>0? nsamples : default_nsamples),
                                  arity, rng);

        int as = alphabet_size(tt, ignore_ops);

        occam_contin_bscore bscore(simple_symbolic_regression(problem_size),
                                   rands, variance, as, rng);
        metapop_moses_results(rng, exemplars, tt,
                              contin_reduction(ignore_ops, rng, 1000000),
                              contin_reduction(ignore_ops, rng, 1000000),
                              bscore, cache_size, opt_algo,
                              opt_params, meta_params, moses_params,
                              vm, mmr_pa);
    //////////////////
    // ANN problems //
    //////////////////
    } else if(problem == ann_it) { // regression based on input table using ann
        auto_ptr<ifstream> in(open_data_file(input_data_file));
        contin_input_table it;
        contin_table ot;
        // read input_data_file file
        istreamTable<contin_input_table,
                     contin_table, contin_t>(*in, it, ot);
        // if no exemplar has been provided in option insert the default one
        if(exemplars.empty()) {
            exemplars.push_back(ann_exemplar(arity));
        }

        // subsample the table
        if(nsamples>0)
            subsampleTable(it, ot, nsamples, rng);

        type_tree tt = declare_function(id::ann_type, 0);
        
        int as = alphabet_size(tt, ignore_ops);

        occam_contin_bscore bscore(ot, it, variance, as, rng);
        metapop_moses_results(rng, exemplars, tt,
                              ann_reduction(),
                              ann_reduction(),
                              bscore, cache_size, opt_algo,
                              opt_params, meta_params, moses_params,
                              vm, mmr_pa);
    } else if(problem == ann_cp) { // regression based on combo program using ann
        // get the combo_tree and infer its type
        combo_tree tr = str_to_combo_tree(combo_str);

        // if no exemplar has been provided in option use the default one
        if(exemplars.empty()) {
            exemplars.push_back(ann_exemplar(arity));
        }
        
        // @todo: introduce some noise optionally
        if(nsamples<=0)
            nsamples = default_nsamples;
        
        contin_input_table it(nsamples, arity, rng,
                              max_rand_input, min_rand_input);
        contin_table table_outputs(tr, it, rng);
        
        type_tree tt = declare_function(id::ann_type, 0);
        
        int as = alphabet_size(tt, ignore_ops);
        
        occam_contin_bscore bscore(table_outputs, it,
                                   variance, as, rng);
        metapop_moses_results(rng, exemplars, tt,
                              contin_reduction(ignore_ops, rng, 1000000),
                              contin_reduction(ignore_ops, rng, 1000000),
                              bscore, cache_size, opt_algo,
                              opt_params, meta_params, moses_params,
                              vm, mmr_pa);
    }
    else unsupported_problem_exit(problem);
    return 0;
}

int moses_exec(const vector<string>& argvs) {
    char** argv = new char*[argvs.size()];
    for(size_t i = 0; i < argvs.size(); i++) {
        argv[i] = const_cast<char*>(argvs[i].c_str());
    }
    int res = moses_exec(argvs.size(), argv);
    delete argv;
    return res;
}
