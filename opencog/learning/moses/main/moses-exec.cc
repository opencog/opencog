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
 * check the first element of the data file, if it is "0" or "1" then
 * it is boolean, otherwise it is contin. It is not 100% reliable of
 * course and should be improved.
 */
type_node infer_type_from_data_file(string file) {
    ifstream in(file.c_str());
    string str;
    in >> str;
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

int main(int argc,char** argv) { 

    // program options, see options_description below for their meaning
    unsigned long rand_seed;
    string input_table_file;
    unsigned int nsamples;
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
        ("help", "produce help message")
        ("random-seed,r", value<unsigned long>(&rand_seed)->default_value(1),
         "random seed")
        ("max-evals,m", value<unsigned long>(&max_evals)->default_value(10000),
         "maximum number of fitness function evaluations")
        ("result-count,c", value<long>(&result_count)->default_value(10),
         "the number of non-dominated best results to return ordered according to their score, if negative then returns all of them")
        ("output-complexity,x", value<bool>(&output_complexity)->default_value(false),
         "if 1, outputs the complexity before each candidate (at the right of the score)")
        ("output-bscore,t", value<bool>(&output_bscore)->default_value(false),
         "if 1, outputs the bscore below each candidate")
        ("max-gens,g", value<int>(&max_gens)->default_value(-1),
         "maximum number of demes to generate and optimize, negative means no generation limit")
        ("input-file,i", value<string>(&input_table_file),
         "input table file")
        ("nsamples,b", value<unsigned int>(&nsamples)->default_value(100),
         "number of samples from the input table, function or problem. In case an input table is provided (option -i) which contains less than or equal to nsamples then all samples of the input table are used, otherwise they are randomly selected to get down to nsamples")
        ("log-level,l", value<string>(&log_level)->default_value("DEBUG"),
         "log level, possible levels are NONE, ERROR, WARN, INFO, DEBUG, FINE. Case does not matter")
        ("log-file,f", value<string>(&log_file)->default_value("moses.log"),
         "file name where to write the log")
        ("variance,v", value<float>(&variance)->default_value(0),
         "in the case of contin regression. variance of an assumed Gaussian around each candidate's output, useful if the data are noisy or to control an Occam's razor bias, 0 or negative means no Occam's razor, otherwise the higher v the stronger the Occam's razor")
        ("prob,p", value<float>(&prob)->default_value(0),
         "in the case of boolean regression, probability that an output datum is wrong (returns false while it should return true or the other way around), useful if the data are noisy or to control an Occam's razor bias, only values 0 < p < 0.5 are meaningful, out of this range it means no Occam's razor, otherwise the greater p the greater the Occam's razor")
        ("ignore-operator,n", value<vector<string> >(&ignore_ops_str),
         "ignore the following operator in the program solution, can be used several times, for moment only div, sin, exp and log can be ignored")
        ("opt-alg,a", value<string>(&opt_algo)->default_value(un),
         "optimization algorithm, current supported algorithms are univariate (un), simulation annealing (sa), hillclimbing (hc)")
        ("exemplar,e", value<vector<string> >(&exemplars_str),
         "start the search with a given exemplar, can be used several times")
        ("reduce-all,d", value<bool>(&reduce_all)->default_value(true),
         "reduce all candidates before being evaluated, can be valuable if the cache is enabled")
        ("count-based-scorer,u", value<bool>(&count_base)->default_value(false),
         "if 1 then a count based scorer is used, otherwise, if 0, a complexity based scorer is used")
        ("cache-size,s", value<unsigned long>(&cache_size)->default_value(1000000),
         "cache size, so that identical candidates are not re-evaluated, 0 means no cache")
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

    if(exemplars_str.empty()) {
        type_node inferred_type = infer_type_from_data_file(input_table_file);
        switch(inferred_type) {
        case id::boolean_type: exemplars_str.push_back("and");
            break;
        case id::contin_type: exemplars_str.push_back("+");
            break;
        case id::ill_formed_type:
            std::cerr << "The data type of the table "
                      << input_table_file << " is not recognized"
                      << std::endl;
            exit(1);
        default:
            std::cerr << "Type " << inferred_type << " is not supported yet"
                      << std::endl;
            exit(1);
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

    type_node output_type = 
        *(get_output_type_tree(*exemplars.begin()->begin()).begin());

    // open input_table_file file
    ifstream in(input_table_file.c_str());
    if(!in.is_open()) {
        if(input_table_file.empty()) {
            std::cerr << "the input file is empty" << std::endl;
            std::cerr << "To indicate the file to open use the option -i or --input-file" << std::endl;
        } else {
            std::cerr << "Could not open " 
                      << input_table_file << std::endl;
        }
        exit(1);
    }

    if(output_type == id::boolean_type) {
        // read input_table_file file
        truth_table_inputs inputtable;
        partial_truth_table booltable;
        istreamTable<truth_table_inputs, partial_truth_table, bool>(in,
                                                                    inputtable,
                                                                    booltable);
        subsampleTable(inputtable, booltable, nsamples, rng);
        unsigned int arity = inputtable[0].size();
        
        type_tree tt(id::lambda_type);
        tt.append_children(tt.begin(), output_type, arity + 1);

        // set alphabet size
        int alphabet_size = 3 + arity - ignore_ops.size();

        typedef occam_truth_table_bscore BScore;
        // typedef opencog::lru_cache<BScore> BScoreCache;
        typedef opencog::simple_cache<BScore> BScoreCache;
        typedef bscore_based_score<BScoreCache> Score;
        // typedef opencog::lru_cache<Score> ScoreCache;
        typedef opencog::simple_cache<Score> ScoreCache;        
        BScore bscore(booltable, inputtable, prob, alphabet_size, rng);
        if(cache_size>0) {
            BScoreCache bscore_cache(cache_size, bscore);
            Score score(bscore_cache);
            ScoreCache score_cache(cache_size, score);
            metapop_moses_results(rng, exemplars, tt, logical_reduction(),
                                  reduce_all, score_cache, bscore_cache,
                                  count_base, opt_algo,
                                  max_evals, max_gens, ignore_ops,
                                  result_count, output_complexity, output_bscore);
        } else {
            bscore_based_score<BScore> score(bscore);
            metapop_moses_results(rng, exemplars, tt, logical_reduction(),
                                  reduce_all, score, bscore, count_base, opt_algo,
                                  max_evals, max_gens, ignore_ops,
                                  result_count, output_complexity, output_bscore);
        }
    }
    else if(output_type == id::contin_type) {
        // read input_table_file file
        contin_table_inputs inputtable;
        contin_table contintable;
        istreamTable<contin_table_inputs, contin_table, contin_t>(in,
                                                                  inputtable,
                                                                  contintable);
        subsampleTable(inputtable, contintable, nsamples, rng);
        unsigned int arity = inputtable[0].size();

        type_tree tt(id::lambda_type);
        tt.append_children(tt.begin(), output_type, arity + 1);

        // set alphabet size
        int alphabet_size = 8 + arity - ignore_ops.size(); // 8 is roughly the
                                                           // number of operators
                                                           // in contin formula,
                                                           // it will have to be
                                                           // adapted

        typedef occam_contin_bscore BScore;
        // typedef opencog::lru_cache<BScore> BScoreCache;
        typedef opencog::simple_cache<BScore> BScoreCache;
        typedef bscore_based_score<BScoreCache> Score;
        // typedef opencog::lru_cache<Score> ScoreCache;
        typedef opencog::simple_cache<Score> ScoreCache;
        BScore bscore(contintable, inputtable, variance, alphabet_size, rng);
        if(cache_size>0) {
            BScoreCache bscore_cache(cache_size, bscore);
            Score score(bscore_cache);
            ScoreCache score_cache(cache_size, score);
            metapop_moses_results(rng, exemplars, tt,
                                  contin_reduction(ignore_ops, rng),
                                  reduce_all, score_cache, bscore_cache,
                                  count_base, opt_algo,
                                  max_evals, max_gens, ignore_ops,
                                  result_count, output_complexity, output_bscore);
        } else {
            bscore_based_score<BScore> score(bscore);
            metapop_moses_results(rng, exemplars, tt,
                                  contin_reduction(ignore_ops, rng),
                                  reduce_all, score, bscore, count_base, opt_algo,
                                  max_evals, max_gens, ignore_ops,
                                  result_count, output_complexity, output_bscore);
        }
    } else {
        std::cerr << "Type " << output_type 
                  << " unhandled for the moment" << std::endl;
        return 1;        
    }
}
