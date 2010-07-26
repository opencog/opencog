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
        std::cerr << "Type " << type << " is not supported yet" << std::endl;
        exit(1);
    }
    return combo_tree();
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
        std::cerr << "type " << tt << "not supported yet" << std::endl;
        exit(1);
    }
}

int main(int argc,char** argv) { 

    // program options, see options_description below for their meaning
    unsigned long rand_seed;
    string input_table_file;
    string problem;
    string combo_str;
    unsigned int problem_size;
    int nsamples;
    float min_rand_input;
    float max_rand_input;
    unsigned long max_evals;
    long result_count;
    bool output_bscore;
    bool output_complexity;
    int max_gens;
    string log_level;
    string log_file;
    float variance;
    float prob;
    vector<string> ignore_ops_str;
    string opt_algo; //optimization algorithm
    vector<string> exemplars_str;
    bool reduce_all;
    bool count_base; // true if the scorer is count based, otherwise
                     // complexity based
    unsigned long cache_size;
    
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message.")
        ("random-seed,r", value<unsigned long>(&rand_seed)->default_value(1),
         "random seed.")
        ("max-evals,m", value<unsigned long>(&max_evals)->default_value(10000),
         "maximum number of fitness function evaluations.")
        ("result-count,c", value<long>(&result_count)->default_value(10),
         "the number of non-dominated best results to return ordered according to their score, if negative then returns all of them.")
        ("output-complexity,x", value<bool>(&output_complexity)->default_value(false),
         "if 1, outputs the complexity before each candidate (at the right of the score).")
        ("output-bscore,t", value<bool>(&output_bscore)->default_value(false),
         "if 1, outputs the bscore below each candidate.")
        ("max-gens,g", value<int>(&max_gens)->default_value(-1),
         "maximum number of demes to generate and optimize, negative means no generation limit.")
        ("input-file,i", value<string>(&input_table_file),
         "input table file, the maximum number of samples is the number of rows in the file.")
        ("problem,h", value<string>(&problem)->default_value("it"),
         string("problem to solve, supported problems are"
                " regression based on input table (").append(it).
         append(", regression based on input table using ann (").append(ann_it).
         append("), regression based on combo program (").append(cp).
         append("), even parity (").append(pa).
         append("), disjunction (").append(dj).
         append("), regression of f(x)_o = sum_{i={1,o}} x^i (").append("sr").
         append(").").c_str())
        ("combo-program,y", value<string>(&combo_str),
         string("combo program to learn, use when the problem ").append(cp).append(" is selected (option -h).").c_str())
        ("problem-size,k", value<unsigned int>(&problem_size)->default_value(5),
         string("for even parity (").append(pa).
         append(") and disjunction (").append(dj).
         append(") the problem size corresponds to the arity,"
                " for regression of f(x)_o = sum_{i={1,o}} x^i (").append(sr).
         append(") the problem size corresponds to the order o.").c_str())
        ("nsamples,b", value<int>(&nsamples)->default_value(-1),
         "number of samples to describ the problem. If nsample is negative, null or larger than the maximum number of samples allowed it is ignored.")
        ("min-rand-input,q", value<float>(&min_rand_input)->default_value(-1),
         "min of an input value chosen randomly, only used when the problem takes continuous inputs")
        ("max-rand-input,w", value<float>(&max_rand_input)->default_value(1),
         "max of an input value chosen randomly, only used when the problem takes continuous inputs")
        ("log-level,l", value<string>(&log_level)->default_value("DEBUG"),
         "log level, possible levels are NONE, ERROR, WARN, INFO, DEBUG, FINE. Case does not matter.")
        ("log-file,f", value<string>(&log_file)->default_value("moses.log"),
         "file name where to write the log.")
        ("variance,v", value<float>(&variance)->default_value(0),
         "in the case of contin regression. variance of an assumed Gaussian around each candidate's output, useful if the data are noisy or to control an Occam's razor bias, 0 or negative means no Occam's razor, otherwise the higher v the stronger the Occam's razor.")
        ("probability,p", value<float>(&prob)->default_value(0),
         "in the case of boolean regression, probability that an output datum is wrong (returns false while it should return true or the other way around), useful if the data are noisy or to control an Occam's razor bias, only values 0 < p < 0.5 are meaningful, out of this range it means no Occam's razor, otherwise the greater p the greater the Occam's razor.")
        ("ignore-operator,n", value<vector<string> >(&ignore_ops_str),
         "ignore the following operator in the program solution, can be used several times, for moment only div, sin, exp and log can be ignored.")
        ("opt-alg,a", value<string>(&opt_algo)->default_value(un),
         string("optimization algorithm, supported algorithms are univariate (").append(un).append("), simulation annealing (").append(sa).append("), hillclimbing (").append(hc).append(").").c_str())
        ("exemplar,e", value<vector<string> >(&exemplars_str),
         "start the search with a given exemplar, can be used several times.")
        ("reduce-all,d", value<bool>(&reduce_all)->default_value(true),
         "reduce all candidates before being evaluated, can be valuable if the cache is enabled.")
        ("count-based-scorer,u", value<bool>(&count_base)->default_value(false),
         "if 1 then a count based scorer is used, otherwise, if 0, a complexity based scorer is used.")
        ("cache-size,s", value<unsigned long>(&cache_size)->default_value(1000000),
         "cache size, so that identical candidates are not re-evaluated, 0 means no cache.")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);    
    
    if (vm.count("help") || argc == 1) {
        cout << desc << "\n";
        return 1;
    }

    // set log
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
        
            type_tree tt = declare_function(type_tree(output_type), arity);

            int as = alphabet_size(tt, ignore_ops);

            occam_truth_table_bscore bscore(booltable, inputtable,
                                            prob, as, rng);
            metapop_moses_results(rng, exemplars, tt, logical_reduction(),
                                  reduce_all, bscore, cache_size,
                                  count_base, opt_algo,
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
            type_tree tt = declare_function(type_tree(output_type), arity);
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
                                  reduce_all, bscore, cache_size,
                                  count_base, opt_algo,
                                  max_evals, max_gens, ignore_ops,
                                  result_count, output_complexity,
                                  output_bscore);
        } else {
            std::cerr << "Type " << output_type 
                      << " unhandled for the moment" << std::endl;
            return 1;
        }
    } else if(problem == ann_it) { // regression based on input table using ann
        auto_ptr<ifstream> in(open_table_file(input_table_file));
        contin_table_inputs inputtable;
        contin_table contintable;
        // read input_table_file file
        istreamTable<contin_table_inputs,
                     contin_table, contin_t>(*in, inputtable, contintable);

        unsigned int arity = inputtable[0].size();

        // if no exemplar has been provided in option try to infer it
        if(exemplars.empty()) {
            combo_tree ann_tr(ann_type(0, id::ann));
            // ann root
            combo_tree::iterator root_node = ann_tr.begin();
            // output node
            combo_tree::iterator output_node =
                ann_tr.append_child(root_node, ann_type(1, id::ann_node));
            // input nodes
            for(unsigned int i = 0; i <= arity; i++) 
                ann_tr.append_child(output_node, ann_type(i + 2, id::ann_input));
            // input nodes' weights
            ann_tr.append_children(output_node, 0.0, arity + 1);
            
            std::cout << ann_tr << std::endl;

            exemplars.push_back(ann_tr);
        }

        // subsample the table
        if(nsamples>0)
            subsampleTable(inputtable, contintable, nsamples, rng);

        type_tree tt = declare_function(type_tree(id::ann_type), 0);
        
        int as = alphabet_size(tt, ignore_ops);

        occam_contin_bscore bscore(contintable, inputtable,
                                   variance, as, rng);
        metapop_moses_results(rng, exemplars, tt,
                              ann_reduction(),
                              reduce_all, bscore, cache_size,
                              count_base, opt_algo,
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity,
                              output_bscore);
    } else if(problem == cp) { // regression based on combo program
        if(combo_str.empty()) {
            std::cerr << "You must specify which combo tree to learn (option -y)"
                      << std::endl;
            return 1;
        }
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
                metapop_moses_results(rng, exemplars, tt, logical_reduction(),
                                      reduce_all, bscore, cache_size,
                                      count_base, opt_algo,
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
                                      reduce_all, bscore, cache_size,
                                      count_base, opt_algo,
                                      max_evals, max_gens, ignore_ops,
                                      result_count, output_complexity,
                                      output_bscore);
            } else {
                std::cerr << "type " << tt
                          << " currently not suppoerted" << std::endl;
                return 1;
            }
        }
        else {
            std::cerr << "apparently the combo tree " 
                      << tr << "is not well formed" << std::endl;
            return 1;
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

        type_tree tt = declare_function(type_tree(id::boolean_type), arity);
        logical_bscore bscore(func, arity, rng);
        metapop_moses_results(rng, exemplars, tt,
                              logical_reduction(),
                              reduce_all, bscore, cache_size,
                              count_base, opt_algo,
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

        type_tree tt = declare_function(type_tree(id::boolean_type), arity);
        logical_bscore bscore(func, arity, rng);
        metapop_moses_results(rng, exemplars, tt,
                              logical_reduction(),
                              reduce_all, bscore, cache_size,
                              count_base, opt_algo,
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
        
        type_tree tt = declare_function(type_tree(id::contin_type), arity);

        contin_table_inputs rands((nsamples>0? nsamples : default_nsamples),
                                  arity, rng);

        int as = alphabet_size(tt, ignore_ops);

        occam_contin_bscore bscore(simple_symbolic_regression(problem_size),
                                   rands, variance, as, rng);
        metapop_moses_results(rng, exemplars, tt,
                              contin_reduction(ignore_ops, rng),
                              reduce_all, bscore, cache_size,
                              count_base, opt_algo,
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity,
                              output_bscore);
    } else {
        std::cerr << "Problem " << problem 
                  << " unsupported for the moment" << std::endl;
        return 1;
    }
}
