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
#include "moses-exec.h"

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

ifstream* open_table_file(string file) {
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
 * check the first element of the data file, if it is "0" or "1" then
 * it is boolean, otherwise it is contin. It is not 100% reliable of
 * course and should be improved.
 */
type_node infer_type_from_data_file(string file) {
    auto_ptr<ifstream> in(open_table_file(file));
    string str;
    *in >> str;
    if(str == "0" || str == "1")
        return id::boolean_type;
    else {
        try {
            lexical_cast<contin_t>(str);
            return id::contin_type;
        }
        catch(...) {
            return id::ill_formed_type;
        }
    }
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


int main(int argc,char** argv) { 

    // program options, see options_description below for their meaning
    unsigned long rand_seed;
    static const string rand_seed_opt_name = "random-seed";
    static const string rand_seed_opt_ab = "r";

    string input_table_file;
    static const string input_table_file_opt_name = "input-file";
    static const string input_table_file_opt_ab = "i";

    string problem;
    static const string problem_opt_name = "problem";
    static const string problem_opt_ab = "H";

    string combo_str;
    static const string combo_str_opt_name = "combo-program";
    static const string combo_str_opt_ab = "y";

    unsigned int problem_size;
    static const string problem_size_opt_name = "problem-size";
    static const string problem_size_opt_ab = "k";

    int nsamples;
    static const string nsamples_opt_name = "nsamples";
    static const string nsamples_opt_ab = "b";

    float min_rand_input;
    static const string min_rand_input_opt_name = "min-rand-input";
    static const string min_rand_input_opt_ab = "q";

    float max_rand_input;
    static const string max_rand_input_opt_name = "max-rand-input";
    static const string max_rand_input_opt_ab = "w";

    unsigned long max_evals;
    static const string max_evals_opt_name = "max-evals";
    static const string max_evals_opt_ab = "m";

    long result_count;
    static const string result_count_opt_name = "result-count";
    static const string result_count_opt_ab = "c";

    bool output_bscore;
    static const string output_bscore_opt_name = "output-bscore";
    static const string output_bscore_opt_ab = "t";

    bool output_complexity;
    static const string output_complexity_opt_name = "output-complexity";
    static const string output_complexity_opt_ab = "x";

    int max_gens;
    static const string max_gens_opt_name = "max-gens";
    static const string max_gens_opt_ab = "g";

    string log_level;
    static const string log_level_opt_name = "log-level";
    static const string log_level_opt_ab = "l";

    string log_file;
    static const string log_file_opt_name = "log-file";
    static const string log_file_opt_ab = "f";
    static const string default_log_file_prefix = "moses";
    static const string default_log_file_suffix = "log";
    static const string default_log_file = default_log_file_prefix + "." + default_log_file_suffix;

    bool log_file_dep_opt;
    static const string log_file_dep_opt_opt_name = "log-file-dep-opt";
    static const string log_file_dep_opt_opt_ab = "L";

    float variance;
    static const string variance_opt_name = "variance";
    static const string variance_opt_ab = "v";

    float prob;
    static const string prob_opt_name = "probability";
    static const string prob_opt_ab = "p";

    vector<string> ignore_ops_str;
    static const string ignore_ops_str_opt_name = "ignore-operator";
    static const string ignore_ops_str_opt_ab = "n";

    string opt_algo; //optimization algorithm
    static const string opt_algo_opt_name = "opt-algo";
    static const string opt_algo_opt_ab = "a";

    vector<string> exemplars_str;
    static const string exemplars_str_opt_name = "exemplar";
    static const string exemplars_str_opt_ab = "e";

    int reduct_candidate_effort;
    static const string reduct_candidate_effort_opt_name = "reduct-candidate-effort";
    static const string reduct_candidate_effort_opt_ab = "E";

    int reduct_knob_building_effort;
    static const string reduct_knob_building_effort_opt_name = "reduct-knob-building-effort";
    static const string reduct_knob_building_effort_opt_ab = "B";

    bool reduce_all;
    static const string reduce_all_opt_name = "reduce-all";
    static const string reduce_all_opt_ab = "d";

    bool count_base; // true if the scorer is count based, otherwise
                     // complexity based
    static const string count_base_opt_name = "count-based-scorer";
    static const string count_base_opt_ab = "u";

    unsigned long cache_size;
    static const string cache_size_opt_name = "cache-size";
    static const string cache_size_opt_ab = "s";

    bool revisit;
    static const string revisit_opt_name = "revisit";
    static const string revisit_opt_ab = "R";

    // eda_param
    double pop_size_ratio;
    static const string pop_size_ratio_opt_name = "pop-size-ratio";
    static const string pop_size_ratio_opt_ab = "P";

    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message.")
        (string(rand_seed_opt_name).append(",").append(rand_seed_opt_ab).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "random seed.")
        (string(max_evals_opt_name).append(",").append(max_evals_opt_ab).c_str(),
         value<unsigned long>(&max_evals)->default_value(10000),
         "maximum number of fitness function evaluations.")
        (string(result_count_opt_name).append(",").append(result_count_opt_ab).c_str(),
         value<long>(&result_count)->default_value(10),
         "the number of non-dominated best results to return ordered according to their score, if negative then returns all of them.")
        (string(output_complexity_opt_name).append(",").append(output_complexity_opt_ab).c_str(),
         value<bool>(&output_complexity)->default_value(false),
         "if 1, outputs the complexity before each candidate (at the right of the score).")
        (string(output_bscore_opt_name).append(",").append(output_bscore_opt_ab).c_str(),
         value<bool>(&output_bscore)->default_value(false),
         "if 1, outputs the bscore below each candidate.")
        (string(max_gens_opt_name).append(",").append(max_gens_opt_ab).c_str(),
         value<int>(&max_gens)->default_value(-1),
         "maximum number of demes to generate and optimize, negative means no generation limit.")
        (string(input_table_file_opt_name).append(",").append(input_table_file_opt_ab).c_str(),
         value<string>(&input_table_file),
         "input table file, the maximum number of samples is the number of rows in the file.")
        (string(problem_opt_name).append(",").append(problem_opt_ab).c_str(),
         value<string>(&problem)->default_value("it"),
         string("problem to solve, supported problems are"
                " regression based on input table (").append(it).
         append("), regression based on input table using ann (").append(ann_it).
         append("), regression based on combo program (").append(cp).
         append("), even parity (").append(pa).
         append("), disjunction (").append(dj).
         append("), regression of f(x)_o = sum_{i={1,o}} x^i (").append("sr").
         append(").").c_str())
        (string(combo_str_opt_name).append(",").append(combo_str_opt_ab).c_str(),
         value<string>(&combo_str),
         string("combo program to learn, use when the problem ").append(cp).append(" is selected (option -h).").c_str())
        (string(problem_size_opt_name).append(",").append(problem_size_opt_ab).c_str(),
         value<unsigned int>(&problem_size)->default_value(5),
         string("for even parity (").append(pa).
         append(") and disjunction (").append(dj).
         append(") the problem size corresponds to the arity,"
                " for regression of f(x)_o = sum_{i={1,o}} x^i (").append(sr).
         append(") the problem size corresponds to the order o.").c_str())
        (string(nsamples_opt_name).append(",").append(nsamples_opt_ab).c_str(),
         value<int>(&nsamples)->default_value(-1),
         "number of samples to describ the problem. If nsample is negative, null or larger than the maximum number of samples allowed it is ignored.")
        (string(min_rand_input_opt_name).append(",").append(min_rand_input_opt_ab).c_str(),
         value<float>(&min_rand_input)->default_value(0),
         "min of an input value chosen randomly, only used when the problem takes continuous inputs")
        (string(max_rand_input_opt_name).append(",").append(max_rand_input_opt_ab).c_str(),
         value<float>(&max_rand_input)->default_value(1),
         "max of an input value chosen randomly, only used when the problem takes continuous inputs")
        (string(log_level_opt_name).append(",").append(log_level_opt_ab).c_str(),
         value<string>(&log_level)->default_value("DEBUG"),
         "log level, possible levels are NONE, ERROR, WARN, INFO, DEBUG, FINE. Case does not matter.")
        (string(log_file_dep_opt_opt_name).append(",").append(log_file_dep_opt_opt_ab).c_str(),
         "the name of the log is determined by the options, for instance if moses-exec is called with -r 123 -H pa the log name is moses_random-seed_123_problem_pa.log")
        (string(log_file_opt_name).append(",").append(log_file_opt_ab).c_str(),
         value<string>(&log_file)->default_value(default_log_file),
         string("file name where to write the log. This option overwrite ").append(log_file_dep_opt_opt_name).append(".").c_str())
        (string(variance_opt_name).append(",").append(variance_opt_ab).c_str(),
         value<float>(&variance)->default_value(0),
         "in the case of contin regression. variance of an assumed Gaussian around each candidate's output, useful if the data are noisy or to control an Occam's razor bias, 0 or negative means no Occam's razor, otherwise the higher v the stronger the Occam's razor.")
        (string(prob_opt_name).append(",").append(prob_opt_ab).c_str(),
         value<float>(&prob)->default_value(0),
         "in the case of boolean regression, probability that an output datum is wrong (returns false while it should return true or the other way around), useful if the data are noisy or to control an Occam's razor bias, only values 0 < p < 0.5 are meaningful, out of this range it means no Occam's razor, otherwise the greater p the greater the Occam's razor.")
        (string(ignore_ops_str_opt_name).append(",").append(ignore_ops_str_opt_ab).c_str(),
         value<vector<string> >(&ignore_ops_str),
         "ignore the following operator in the program solution, can be used several times, for moment only div, sin, exp and log can be ignored.")
        (string(opt_algo_opt_name).append(",").append(opt_algo_opt_ab).c_str(),
         value<string>(&opt_algo)->default_value(un),
         string("optimization algorithm, supported algorithms are"
                " univariate (").append(un).
         append("), simulation annealing (").append(sa).
         append("), hillclimbing (").append(hc).append(").").c_str())
        (string(exemplars_str_opt_name).append(",").append(exemplars_str_opt_ab).c_str(),
         value<vector<string> >(&exemplars_str),
         "start the search with a given exemplar, can be used several times.")
        (string(reduce_all_opt_name).append(",").append(reduce_all_opt_ab).c_str(),
         value<bool>(&reduce_all)->default_value(true),
         "reduce all candidates before being evaluated, otherwise there are only reduced before being added to the metapopulation. This option can be valuable if the cache is enabled to not evaluate multiple time equivalent candidates.")
        (string(reduct_candidate_effort_opt_name).append(",").append(reduct_candidate_effort_opt_ab).c_str(),         
         value<int>(&reduct_candidate_effort)->default_value(2),
         "effort allocated for reduction of candidates, 0-3, 0 means minimum effort, 3 means maximum effort.")
        (string(reduct_knob_building_effort_opt_name).append(",").append(reduct_knob_building_effort_opt_ab).c_str(),
         value<int>(&reduct_knob_building_effort)->default_value(2),
         "effort allocated for reduction during knob building, 0-3, 0 means minimum effort, 3 means maximum effort. The bigger the effort the lower the dimension of the deme.")
        (string(count_base_opt_name).append(",").append(count_base_opt_ab).c_str(),
         value<bool>(&count_base)->default_value(false),
         "if 1 then a count based scorer is used, otherwise, if 0, a complexity based scorer is used.")
        (string(cache_size_opt_name).append(",").append(cache_size_opt_ab).c_str(),
         value<unsigned long>(&cache_size)->default_value(1000000),
         "cache size, so that identical candidates are not re-evaluated, 0 means no cache.")
        (string(revisit_opt_name).append(",").append(revisit_opt_ab).c_str(),
         "revisit visited examplars when all have been visited")
        (string(pop_size_ratio_opt_name).append(",").append(pop_size_ratio_opt_ab).c_str(),
         value<double>(&pop_size_ratio)->default_value(200),
         "the higher the more effort is spent on a deme")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    // set flags
    revisit = vm.count(revisit_opt_name) > 0;
    log_file_dep_opt = vm.count(log_file_dep_opt_opt_name) > 0 && log_file == default_log_file;
    
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
            if(it->first != log_file_dep_opt_opt_name && !it->second.defaulted()
               // this is because OSs usually do not handle file name above 255 chars
               && log_file.size() < 255)
                log_file += string("_") + it->first + "_" + to_string(it->second);
        log_file += string(".") + default_log_file_suffix;
    }
    // remove log_file
    remove(log_file.c_str());
    logger().setFilename(log_file);
    logger().setLevel(logger().getLevelFromString(log_level));
    logger().setBackTraceLevel(opencog::Logger::ERROR);
    // init random generator
    opencog::MT19937RandGen rng(rand_seed);

    // convert ignore_ops_str to the set of actual operators to ignore
    vertex_set ignore_ops;
    foreach(const string& s, ignore_ops_str) {
        vertex v;
        if(builtin_str_to_vertex(s, v))
            ignore_ops.insert(v);
        else {
            std::cerr << "error: " << s 
                      << " is not recognized as combo operator" << std::endl;
            return 1;
        }
    }

    // set the initial exemplars
    vector<combo_tree> exemplars;
    foreach(const string& exemplar_str, exemplars_str) {
        stringstream ss;
        combo_tree exemplar;
        ss << exemplar_str;
        ss >> exemplar;
        exemplars.push_back(exemplar);
    }

    // set metapopulation parameters
    metapop_parameters meta_param(revisit);

    // set eda_parameters
    eda_parameters eda_param(pop_size_ratio);

    if(problem == it) { // regression based on input table
        
        // if no exemplar has been provided in option try to infer it
        if(exemplars.empty()) {            
            type_node inferred_type = infer_type_from_data_file(input_table_file);
            exemplars.push_back(type_to_exemplar(inferred_type));
        }

        type_node output_type = 
            *(get_output_type_tree(*exemplars.begin()->begin()).begin());

        auto_ptr<ifstream> in(open_table_file(input_table_file));

        if(output_type == id::boolean_type) {
            // read input_table_file file
            truth_table_inputs inputtable;
            partial_truth_table booltable;
            istreamTable<truth_table_inputs,
                         partial_truth_table, bool>(*in, inputtable, booltable);
            if(nsamples>0)
                subsampleTable(inputtable, booltable, nsamples, rng);
            unsigned int arity = inputtable[0].size();
        
            type_tree tt = declare_function(output_type, arity);

            int as = alphabet_size(tt, ignore_ops);

            occam_truth_table_bscore bscore(booltable, inputtable,
                                            prob, as, rng);
            metapop_moses_results(rng, exemplars, tt,
                                  logical_reduction(reduct_candidate_effort),
                                  logical_reduction(reduct_knob_building_effort),
                                  reduce_all, bscore, cache_size,
                                  count_base, opt_algo,
                                  eda_param, meta_param,
                                  max_evals, max_gens, ignore_ops,
                                  result_count, output_complexity, output_bscore);
        }
        else if(output_type == id::contin_type) {
            // read input_table_file file
            contin_table_inputs inputtable;
            contin_table contintable;
            istreamTable<contin_table_inputs,
                         contin_table, contin_t>(*in, inputtable, contintable);
            if(nsamples>0)
                subsampleTable(inputtable, contintable, nsamples, rng);

            unsigned int arity = inputtable[0].size();
            type_tree tt = declare_function(output_type, arity);
            int as = alphabet_size(tt, ignore_ops);

            // if no exemplar has been provided in option use the default
            // contin_type exemplar (+)
            if(exemplars.empty()) {            
                exemplars.push_back(type_to_exemplar(id::contin_type));
            }

            occam_contin_bscore bscore(contintable, inputtable,
                                       variance, as, rng);
            metapop_moses_results(rng, exemplars, tt,
                                  contin_reduction(ignore_ops, rng),
                                  contin_reduction(ignore_ops, rng),
                                  reduce_all, bscore, cache_size,
                                  count_base, opt_algo,
                                  eda_param, meta_param,
                                  max_evals, max_gens, ignore_ops,
                                  result_count, output_complexity,
                                  output_bscore);
        } else {
            unsupported_type_exit(output_type);
        }
    } else if(problem == cp) { // regression based on combo program
        if(combo_str.empty())
            unspecified_combo_exit();
        // get the combo_tree and infer its type
        stringstream ss;
        combo_tree tr;
        ss << combo_str;
        ss >> tr;
        type_tree tt = infer_type_tree(tr);

        if(is_well_formed(tt)) {
            type_node output_type = *type_tree_output_type_tree(tt).begin();
            arity_t arity = type_tree_arity(tt);
            // if no exemplar has been provided in option use the default one
            if(exemplars.empty()) {
                exemplars.push_back(type_to_exemplar(output_type));
            }
            if(output_type == id::boolean_type) {
                // @todo: Occam's razor and nsamples is not taken into account
                logical_bscore bscore(tr, arity, rng);
                metapop_moses_results(rng, exemplars, tt,
                                      logical_reduction(reduct_candidate_effort),
                                      logical_reduction(reduct_knob_building_effort),
                                      reduce_all, bscore, cache_size,
                                      count_base, opt_algo,
                                      eda_param, meta_param,
                                      max_evals, max_gens, ignore_ops,
                                      result_count, output_complexity,
                                      output_bscore);                
            }
            else if (output_type == id::contin_type) {
                // @todo: introduce some noise optionally
                if(nsamples<=0)
                    nsamples = default_nsamples;
                
                contin_table_inputs inputtable(nsamples, arity, rng,
                                               max_rand_input, min_rand_input);
                contin_table table_outputs(tr, inputtable, rng);

                int as = alphabet_size(tt, ignore_ops);

                occam_contin_bscore bscore(table_outputs, inputtable,
                                           variance, as, rng);
                metapop_moses_results(rng, exemplars, tt,
                                      contin_reduction(ignore_ops, rng),
                                      contin_reduction(ignore_ops, rng),
                                      reduce_all, bscore, cache_size,
                                      count_base, opt_algo,
                                      eda_param, meta_param,
                                      max_evals, max_gens, ignore_ops,
                                      result_count, output_complexity,
                                      output_bscore);
            } else {
                unsupported_type_exit(tt);
            }
        }
        else {
            illformed_exit(tr);
        }
    } else if(problem == pa) { // even parity
        // @todo: for the moment occam's razor and partial truth table are ignored
        unsigned int arity = problem_size;
        even_parity func;

        // if no exemplar has been provided in option use the default
        // contin_type exemplar (and)
        if(exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = declare_function(id::boolean_type, arity);
        logical_bscore bscore(func, arity, rng);
        metapop_moses_results(rng, exemplars, tt,
                              logical_reduction(reduct_candidate_effort),
                              logical_reduction(reduct_knob_building_effort),
                              reduce_all, bscore, cache_size,
                              count_base, opt_algo,
                              eda_param, meta_param,
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity,
                              output_bscore);        
    } else if(problem == dj) { // disjunction
        // @todo: for the moment occam's razor and partial truth table are ignored
        unsigned int arity = problem_size;
        disjunction func;

        // if no exemplar has been provided in option use the default
        // contin_type exemplar (and)
        if(exemplars.empty()) {            
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = declare_function(id::boolean_type, arity);
        logical_bscore bscore(func, arity, rng);
        metapop_moses_results(rng, exemplars, tt,
                              logical_reduction(reduct_candidate_effort),
                              logical_reduction(reduct_knob_building_effort),
                              reduce_all, bscore, cache_size,
                              count_base, opt_algo,
                              eda_param, meta_param,
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity,
                              output_bscore);        
    } else if(problem == sr) { // simple regression of f(x)_o = sum_{i={1,o}} x^i
        unsigned int arity = 1;

        // if no exemplar has been provided in option use the default
        // contin_type exemplar (+)
        if(exemplars.empty()) {            
            exemplars.push_back(type_to_exemplar(id::contin_type));
        }
        
        type_tree tt = declare_function(id::contin_type, arity);

        contin_table_inputs rands((nsamples>0? nsamples : default_nsamples),
                                  arity, rng);

        int as = alphabet_size(tt, ignore_ops);

        occam_contin_bscore bscore(simple_symbolic_regression(problem_size),
                                   rands, variance, as, rng);
        metapop_moses_results(rng, exemplars, tt,
                              contin_reduction(ignore_ops, rng),
                              contin_reduction(ignore_ops, rng),
                              reduce_all, bscore, cache_size,
                              count_base, opt_algo,
                              eda_param, meta_param,
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity,
                              output_bscore);
    //////////////////
    // ANN problems //
    //////////////////
    } else if(problem == ann_it) { // regression based on input table using ann
        auto_ptr<ifstream> in(open_table_file(input_table_file));
        contin_table_inputs inputtable;
        contin_table contintable;
        // read input_table_file file
        istreamTable<contin_table_inputs,
                     contin_table, contin_t>(*in, inputtable, contintable);

        unsigned int arity = inputtable[0].size();

        // if no exemplar has been provided in option insert the default one
        if(exemplars.empty()) {
            exemplars.push_back(ann_exemplar(arity));
        }

        // subsample the table
        if(nsamples>0)
            subsampleTable(inputtable, contintable, nsamples, rng);

        type_tree tt = declare_function(id::ann_type, 0);
        
        int as = alphabet_size(tt, ignore_ops);

        occam_contin_bscore bscore(contintable, inputtable,
                                   variance, as, rng);
        metapop_moses_results(rng, exemplars, tt,
                              ann_reduction(),
                              ann_reduction(),
                              reduce_all, bscore, cache_size,
                              count_base, opt_algo,
                              eda_param, meta_param,
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity,
                              output_bscore);
    } else if(problem == ann_cp) { // regression based on combo program using ann
        if(combo_str.empty())
            unspecified_combo_exit();
        // get the combo_tree and infer its type
        stringstream ss;
        combo_tree tr;
        ss << combo_str;
        ss >> tr;
        type_tree problem_tt = infer_type_tree(tr);

        if(is_well_formed(problem_tt)) {
            arity_t arity = type_tree_arity(problem_tt);
            // if no exemplar has been provided in option use the default one
            if(exemplars.empty()) {
                exemplars.push_back(ann_exemplar(arity));
            }

            // @todo: introduce some noise optionally
            if(nsamples<=0)
                nsamples = default_nsamples;

            contin_table_inputs inputtable(nsamples, arity, rng,
                                               max_rand_input, min_rand_input);
            contin_table table_outputs(tr, inputtable, rng);
            
            type_tree tt = declare_function(id::ann_type, 0);

            int as = alphabet_size(tt, ignore_ops);
            
            occam_contin_bscore bscore(table_outputs, inputtable,
                                       variance, as, rng);
            metapop_moses_results(rng, exemplars, tt,
                                  contin_reduction(ignore_ops, rng),
                                  contin_reduction(ignore_ops, rng),
                                  reduce_all, bscore, cache_size,
                                  count_base, opt_algo,
                                  eda_param, meta_param,
                                  max_evals, max_gens, ignore_ops,
                                  result_count, output_complexity,
                                  output_bscore);
        } else illformed_exit(tr);
    }
    else unsupported_problem_exit(problem);
}
