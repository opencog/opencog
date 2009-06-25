/*
 * opencog/learning/moses/moses/optimization.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _MOSES_OPTIMIZE_H
#define _MOSES_OPTIMIZE_H

#include "moses.h"
#include <opencog/learning/moses/eda/termination.h>
#include <opencog/learning/moses/eda/replacement.h>
#include <opencog/learning/moses/eda/logging.h>
#include <opencog/learning/moses/eda/local_structure.h>
#include <opencog/learning/moses/eda/optimize.h>
#include <opencog/learning/moses/eda/initialization.h>
#include <opencog/util/selection.h>
#include <opencog/util/dorepeat.h>
#include <opencog/util/exceptions.h>

#define MAX_DISTANCE_FROM_EXEMPLAR 5
#define FRACTION_OF_REMAINING     10
#define MINIMUM_DEME_SIZE         50
#define MAX_EVALS_PER_SLICE       10


namespace moses
{

struct eda_parameters {
    eda_parameters() :
        term_total(1),         //optimization is teriminated after term_total*n
        term_improv(1),        //generations, or term_improv*sqrt(N/w) consecutive
        //generations with no improvement (w=windowsize)
        
        pop_size_ratio(20),     //populations are sized at N = popsize_ratio*n^1.05
        //where n is problem size in info-t bits
        
        window_size_pop(0.05), //window size for RTR is
        window_size_len(1),    //min(windowsize_pop*N,windowsize_len*n)
        
        selection(2),          //if <=1, truncation selection ratio,
        //if >1, tournament selection size (should be int)
        selection_ratio(1),    //ratio of population size selected for modeling
        
        replacement_ratio(0.5),//ratio of population size sampled and integrated
        
        
        model_complexity(1),   //model parsimony term log(N)*model_complexity
        
        terminate_if_gte(0)    //optimization is terminated if best score is >= this
    { }

    bool is_tournament_selection() {
        return selection > 1;
    }
    bool is_truncation_selection() {
        return selection <= 1;
    }


    //N=p.popsize_ratio*n^1.05
    inline int pop_size(const eda::field_set& fs) {
        return int(ceil((double(pop_size_ratio)*
                         pow(information_theoretic_bits(fs), 1.05))));
    }

    //min(windowsize_pop*N,windowsize_len*n)
    inline int rtr_window_size(const eda::field_set& fs) {
        return int(ceil(min(window_size_pop*pop_size(fs),
                            window_size_len*information_theoretic_bits(fs))));
    }

    //term_total*n
    inline int max_gens_total(const eda::field_set& fs) {
        return int(ceil(term_total*information_theoretic_bits(fs)));
    }
    //term_improv*sqrt(N/w)
    inline int max_gens_improv(const eda::field_set& fs) {
        return int(ceil(term_improv*
                        sqrt(information_theoretic_bits(fs) /
                             rtr_window_size(fs))));
    }

    double term_total;
    double term_improv;

    double pop_size_ratio;
    double window_size_pop;
    double window_size_len;
    double selection;
    double selection_ratio;
    double replacement_ratio;
    double model_complexity;
    double terminate_if_gte;
};

/**
 * This procedure generat the initial deme randomly
 *
 * @param fs the deme
 * @param n  the size of deme
 * @param out deme(where to store the instance)
 */
template<typename Out>
void generate_initial_sample(const eda::field_set& fs, int n, Out out,
                             opencog::RandGen& rng)
{
    dorepeat(n) {

        eda::instance inst(fs.packed_width());

        eda::randomize(fs, inst, rng);

        //bias towards zero
        for (eda::field_set::bit_iterator it = fs.begin_bits(inst);
                it != fs.end_bits(inst);++it)
            if (rng.randint(2) == 0)
                *it = false;
        for (eda::field_set::disc_iterator it = fs.begin_disc(inst);
                it != fs.end_disc(inst);++it)
            if (rng.randint(2) == 0)
                *it = 0;

        //add it
        *out++ = inst;
    }
    //foo42
    //caching not yet integrated
    //note: NOT YET USING RTR
}


/**
 * This procedure samples sample_size instances at distance n from the exemplar
 * (i.e., with n non-zero elements in the sequence)
 *
 *@param fs  - deme
 *@param n   - distance
 *@param sample_size  - number of instances to be generated
 *@param out - deme (where to store the instances)
 */
template<typename Out>
void sample_from_neighborhood(const eda::field_set& fs, int n,
                              int sample_size, Out out, opencog::RandGen& rng)
{
    if (n <= 0 || sample_size <= 0)
        return;

    cout << "bits size: " << fs.n_bits() << endl;
    cout << "disc size: " << fs.n_disc() << endl;
    cout << "Sampling : " << sample_size << endl;

    int dim = fs.n_bits() + fs.n_disc();

    eda::instance inst(fs.packed_width());

    // reset all fields (contin and onto fields are ignored)
    for (eda::field_set::bit_iterator it = fs.begin_bits(inst);
            it != fs.end_bits(inst); ++it)
        *it = false;

    for (eda::field_set::disc_iterator it = fs.begin_disc(inst);
            it != fs.end_disc(inst); ++it)
        *it = 0;

    dorepeat(sample_size) {

        eda::instance new_inst(inst);

        for (int i = 1;i <= n;) {
            int r = rng.randint(dim);
            eda::field_set::bit_iterator itb = fs.begin_bits(new_inst);
            eda::field_set::disc_iterator itd = fs.begin_disc(new_inst);

            if ((unsigned int)r < fs.n_bits()) {
                itb += r;
                if (*itb == false) {
                    *itb = true;
                    i++;
                }
            } else if ((unsigned int)r >= fs.n_bits()) {
                itd += r - fs.n_bits();
                if (*itd == 0) {
                    *itd = 1 + rng.randint(itd.arity() - 1);
                    i++;
                }
            }
        }

        *out++ = new_inst;
        // cout << "********** Added instance:" << fs.stream(new_inst) << endl;
    }
}

template<typename Out>
void sample_from_neighborhood_ex(const eda::field_set& fs, int n,
                                 int sample_size, Out out, opencog::RandGen& rng,
                                 const eda::instance & center_inst )
{
    if (n <= 0 || sample_size <= 0 ||
        center_inst.size() != fs.packed_width())
        return;
    

    cout << "bits size: " << fs.n_bits() << endl;
    cout << "disc size: " << fs.n_disc() << endl;
    cout << "Sampling : " << sample_size << endl;

    int dim = fs.n_bits() + fs.n_disc();


    dorepeat(sample_size) {

        eda::instance new_inst(center_inst);

        for (int i = 1;i <= n;) {
            int r = rng.randint(dim);
            eda::field_set::bit_iterator itb = fs.begin_bits(new_inst);
            eda::field_set::disc_iterator itd = fs.begin_disc(new_inst);

            if ((unsigned int)r < fs.n_bits()) {
                itb += r;
                if (*itb == false) {
                    *itb = true;
                    i++;
                }
            } else if ((unsigned int)r >= fs.n_bits()) {
                itd += r - fs.n_bits();
                if (*itd == 0) {
                    *itd = 1 + rng.randint(itd.arity() - 1);
                    i++;
                }
            }
        }

        *out++ = new_inst;
        // cout << "********** Added instance:" << fs.stream(new_inst) << endl;
    }
}

/**
 * Generates instances at distance n from the exemplar
 * (i.e., with n elements changed from 0 from the exemplar)
 * It calls a recursive function vary_n_knobs which varies
 * instance fields one by one (in all possible ways)
 *
 * @param fs  - deme
 * @param n   - distance
 * @param out - deme (where to store the instances)
 */
template<typename Out>
void generate_all_in_neighborhood(const eda::field_set& fs, int n, Out out)
{
    if (n <= 0)
        return;

    eda::instance inst(fs.packed_width());

    // reset all fields (contin and onto fields are ignored)
    for (eda::field_set::bit_iterator it = fs.begin_bits(inst);
            it != fs.end_bits(inst); ++it)
        *it = false;

    for (eda::field_set::disc_iterator it = fs.begin_disc(inst);
            it != fs.end_disc(inst); ++it)
        *it = 0;

    vary_n_knobs(fs, inst, n, 0, out);
}

/**
 * Used by the function generate_all_in_neighborhood (only)
 * for generating instances at distance n from the exemplar. It varies
 * all possible n knobs in all possible ways. It varies
 * one instance field (at the changing position starting_index and
 * calls itself for the remaining fields).
 *
 *@param fs              deme
 *@param n               distance
 *@param starting_index  position of a field to be varied
 *@param out             deme (where to store the instances)
 */
template<typename Out>
void vary_n_knobs(const eda::field_set& fs, eda::instance& inst, int n,
                  int starting_index, Out& out)
{
    if (n < 0)
        return;

    if (n == 0) {
        eda::instance i(inst);
        *out++ = i;
        return;
    }

    eda::field_set::bit_iterator itb = fs.begin_bits(inst);
    itb += starting_index;

    eda::instance current;

    if (fs.end_bits(inst) - itb > 0)  {
        // save the current version
        current = inst;
        *itb = false;
        // recursive call, moved for one position
        vary_n_knobs(fs, inst, n, starting_index + 1, out);
        // recover after the recursive calls
        inst = current;

        // save the current version
        current = inst;
        *itb = true;
        // recursive call, moved for one position
        vary_n_knobs(fs, inst, n - 1, starting_index + 1, out);
        // recover after the recursive calls
        inst = current;
    } else  {
        eda::field_set::disc_iterator itd = fs.begin_disc(inst);
        itd += starting_index - fs.n_bits();

        if (fs.end_disc(inst) - itd > 0)  {
            // save the current version
            current = inst;
            *itd = 0;
            // recursive call, moved for one position
            vary_n_knobs(fs, inst, n, starting_index + 1, out);
            // recover after the recursive calls
            inst = current;

            for (unsigned int i = 1;i <= itd.arity() - 1;i++)  {
                // save the current version
                current = inst;
                // vary all legal values
                *itd = i;
                vary_n_knobs(fs, inst, n - 1, starting_index + 1, out);
                // recover after the recursive calls
                inst = current;
            }
        }
    }
}


/**
 * Used by the function count_n_changed_knobs (only)
 * for counting instances at distance n from the exemplar. It counts
 * all possible n knobs changed in all possible ways.
 * 
 * @param fs             - deme
 * @param n              - distance
 * @param starting_index - position of a field to be varied
 */
inline long long count_n_changed_knobs_from_index(const eda::field_set& fs,
                                                  int n, int starting_index)
{
    if (n < 0)
        return 0;

    if (n == 0)
        return 1;

    eda::instance inst(fs.packed_width());
    long long number_of_instances = 0;

    eda::field_set::bit_iterator itb = fs.begin_bits(inst);
    itb += starting_index;

    if (fs.end_bits(inst) - itb > 0)  {
        // recursive call, moved for one position
        number_of_instances += count_n_changed_knobs_from_index(fs, n, starting_index + 1);
        // recursive call, moved for one position
        number_of_instances += count_n_changed_knobs_from_index(fs, n - 1, starting_index + 1);
    } else  {
        eda::field_set::disc_iterator itd = fs.begin_disc(inst);
        itd += starting_index - fs.n_bits();

        if (fs.end_disc(inst) - itd > 0)  {
            // recursive call, moved for one position
            number_of_instances += count_n_changed_knobs_from_index(fs, n, starting_index + 1);
            // vary all legal values of the knob
            number_of_instances += (itd.arity() - 1) * count_n_changed_knobs_from_index(fs, n - 1, starting_index + 1);
        }
    }

    return number_of_instances;
}



/**
 * Counts instances at distance n from the exemplar
 * (i.e., with n elements changed from the exemplar)
 * It calls a recursive function count_n_changed_knobs_from_index
 * 
 * @param fs  - deme
 * @param n   - distance
 */
inline long long count_n_changed_knobs(const eda::field_set& fs, int n)
{
    return count_n_changed_knobs_from_index(fs, n, 0);
}



struct univariate_optimization {
    univariate_optimization(opencog::RandGen& _rng,
                            const eda_parameters& p = eda_parameters())
        : rng(_rng), params(p) {}

    //return # of evaluations actually performed
    template<typename Scoring>
    int operator()(eda::instance_set<combo_tree_score>& deme,
                   const Scoring& score, int max_evals) {
        int pop_size = params.pop_size(deme.fields());
        int max_gens_total = params.max_gens_total(deme.fields());
        int max_gens_improv = params.max_gens_improv(deme.fields());
        int n_select = int(double(pop_size) * params.selection_ratio);
        int n_generate = int(double(pop_size * params.replacement_ratio));

        //adjust parameters based on the maximal # of evaluations allowed
        if (max_evals < pop_size) {
            pop_size = max_evals;
            max_gens_total = 0;
        } else {
            max_gens_total = min(max_gens_total,
                                 (max_evals - pop_size) / n_generate);
        }

        //create the initial sample
        //generate the initial sample to populate the deme
        deme.resize(pop_size);
        generate_initial_sample(deme.fields(), pop_size, deme.begin(), rng);

        if (params.is_tournament_selection()) {
            eda::cout_log_best_and_gen logger;
            return eda::optimize
                   (deme, n_select, n_generate, max_gens_total, score,
                    eda::terminate_if_gte_or_no_improv<combo_tree_score>
                    (combo_tree_score(params.terminate_if_gte,
                                worst_possible_score.second),
                     max_gens_improv),
                    opencog::tournament_selection(int(params.selection), rng),
                    eda::univariate(), eda::local_structure_probs_learning(),
                    eda::rtr_replacement(deme.fields(),
                                         params.rtr_window_size(deme.fields()),
                                         rng),
                    logger, rng);
        } else { //truncation selection
            OC_ASSERT(TRACE_INFO, false,
                             "Trunction selection not implemented. Tournament should be used instead.");
            return 42;
            /*
            return optimize(deme,n_select,n_generate,args.max_gens,score,
              terminate_if_gte_or_no_improv(params.terminate_if_gte,
                       max_gens_improv),
              //truncation selection goes here
              univariate(),local_structure_probs_learning(),
              replace_the_worst(),cout_log_best_and_gen());
            */
        }
    }

    opencog::RandGen& rng;
    eda_parameters params;
};




struct iterative_hillclimbing {
    iterative_hillclimbing(opencog::RandGen& _rng,
                           const eda_parameters& p = eda_parameters())
        : rng(_rng), params(p) {}


    //return # of evaluations actually performed
    template<typename Scoring>
    int operator()(eda::instance_set<combo_tree_score>& deme,
                   const Scoring& score, int max_evals) {
        int pop_size = params.pop_size(deme.fields());
        int max_gens_total = params.max_gens_total(deme.fields());
        // int max_gens_improv=params.max_gens_improv(deme.fields());

        cout << "bits size: " << deme.fields().n_bits() << endl;
        cout << "disc size: " << deme.fields().n_disc() << endl;
        // cout << "pop size: " << pop_size << endl;
        // cout << "max gen total: " << max_gens_total << endl;
        // cout << "max gen improv: " << max_gens_improv << endl;

        long long current_number_of_instances = 0;

        //adjust parameters based on the maximal # of evaluations allowed
        int max_number_of_instances = max_gens_total * pop_size;
        if (max_number_of_instances > max_evals)
            max_number_of_instances = max_evals;

        int number_of_fields = deme.fields().n_bits() + deme.fields().n_disc();
        eda::instance exemplar(deme.fields().packed_width());

        // set to 0 all fields (contin and onto fields are ignored) to represent the exemplar
        for (eda::field_set::bit_iterator it = deme.fields().begin_bits(exemplar);
                it != deme.fields().end_bits(exemplar); ++it)
            *it = false;
        for (eda::field_set::disc_iterator it = deme.fields().begin_disc(exemplar);
                it != deme.fields().end_disc(exemplar); ++it)
            *it = 0;

        eda::scored_instance<combo_tree_score> scored_exemplar = exemplar;
        score_t exemplar_score = score(scored_exemplar).first;
        // cout << "Exemplar:" <<deme.fields().stream(scored_exemplar) << " Score:" << exemplar_score << endl;

        // more precisely, instead of 0 there should be max_score, but this value is not passed as an argument
        if (exemplar_score == 0) {
            // found a perfect solution
            deme.resize(1);
            *(deme.begin()++) = exemplar;
        } else {

            int distance = 1;
            bool bImprovement_made = false;
            score_t best_score;

            do {
                cout << "Distance in this iteration:" << distance << endl;

                // the number of all neighbours at the distance d
                long long total_number_of_neighbours = count_n_changed_knobs(deme.fields(), distance);
                cout << "Number of possible instances:"
                     << total_number_of_neighbours << endl;

                long long number_of_new_instances;

                number_of_new_instances = (max_number_of_instances - current_number_of_instances) / FRACTION_OF_REMAINING;
                if (number_of_new_instances  < MINIMUM_DEME_SIZE)
                    number_of_new_instances = (max_number_of_instances - current_number_of_instances);

                if (number_of_new_instances < total_number_of_neighbours) {
                    //resize the deme so it can take new instances
                    deme.resize(current_number_of_instances + number_of_new_instances);
                    //sample 'number_of_new_instances' instances on the distance 'distance' from the exemplar
                    sample_from_neighborhood(deme.fields(), distance, number_of_new_instances, deme.begin() + current_number_of_instances, rng);
                } else {
                    number_of_new_instances = total_number_of_neighbours;
                    //resize the deme so it can take new instances
                    deme.resize(current_number_of_instances + number_of_new_instances);
                    //add all instances on the distance 'distance' from the exemplar
                    generate_all_in_neighborhood(deme.fields(), distance, deme.begin() + current_number_of_instances);
                }

                cout << "New size:" << current_number_of_instances + number_of_new_instances << endl;

                // score all new instances in the deme
                transform(deme.begin() + current_number_of_instances, 
                          deme.end(),
                          deme.begin_scores() + current_number_of_instances,
                          score);

                best_score = exemplar_score;
                // check if there is an instance in the deme better than the exemplar
                //foreach(const eda::scored_instance<combo_tree_score>& inst,deme) {
                for (int i = current_number_of_instances;deme.begin() + i != deme.end();i++) {

                    //score(deme.begin()+i,deme.begin_scores()+i);
                    //transform(deme.begin()+current_number_of_instances,deme.end(),deme.begin_scores()+current_number_of_instances,score);

                    const eda::scored_instance<combo_tree_score>& inst = deme[i];

                    if (get_score(inst.second) > best_score) {
                        best_score = get_score(inst.second);
                        bImprovement_made = true;

                        cout << endl << "Improvement, new best score:" << inst.second << "(distance: " << distance << ", old score: " << exemplar_score << ")" << endl;
                        cout << "Found instance:" << deme.fields().stream(inst) << endl;
                        cout << "Score:" << get_score(inst.second) << endl;
                        cout << "--------------------------------------" << endl;
                        //  break;
                    }
                }

                current_number_of_instances += number_of_new_instances;
                distance++;

            } while (!bImprovement_made &&
                     distance <= number_of_fields &&
                     distance <= MAX_DISTANCE_FROM_EXEMPLAR &&
                     current_number_of_instances < max_number_of_instances);

        }

        // cout << "poop" << endl;
        // cout << "mm " << candidates.size() << endl;
        // cout << "zz " << distance(candidates.begin(),candidates.end()) << endl;

        return current_number_of_instances;
    }

    opencog::RandGen& rng;
    eda_parameters params;
};




struct sliced_iterative_hillclimbing {

    typedef enum {
        M_INIT,
        M_BUILD_CANDIDATES,
        M_EVALUATE_CANDIDATES
    } MState;


    sliced_iterative_hillclimbing(opencog::RandGen& _rng,
                                  const eda_parameters& p = eda_parameters())
        : rng(_rng), params(p), m_state(M_INIT),
          _evals_per_slice(MAX_EVALS_PER_SLICE) {}

    void set_evals_per_slice(int evals_per_slice) {
        _evals_per_slice = evals_per_slice;
    }

    //return # of evaluations actually performed
    //WARNING: if an improvement has been made it returns
    // -number_of_eval
    //it is rather ugly and it is a temporary hack till the API of MOSES
    //is better
    template<typename Scoring>
    int operator()(eda::instance_set<combo_tree_score>& deme,
                   const Scoring& score, int max_evals) {
        switch (m_state) {

        case M_INIT:  {
            current_number_of_instances = 0;

            cout << "bits size: " << deme.fields().n_bits() << endl;
            cout << "disc size: " << deme.fields().n_disc() << endl;
            cout << "max evals : " << max_evals << endl << endl;

            max_number_of_instances = max_evals;

            number_of_fields = deme.fields().n_bits() + deme.fields().n_disc();
            exemplar = eda::instance(deme.fields().packed_width());

            // set to 0 all fields (contin and onto fields are ignored) to represent the exemplar
            for (eda::field_set::bit_iterator it = deme.fields().begin_bits(exemplar);
                    it != deme.fields().end_bits(exemplar); ++it)
                *it = false;
            for (eda::field_set::disc_iterator it = deme.fields().begin_disc(exemplar);
                    it != deme.fields().end_disc(exemplar); ++it)
                *it = 0;

            eda::scored_instance<combo_tree_score> scored_exemplar = exemplar;
            exemplar_score = score(scored_exemplar).first;
            // cout << "Exemplar:" <<deme.fields().stream(scored_exemplar) << " Score:" << exemplar_score << endl;

            distance = 1;

            // more precisely, instead of 0 there should be max_score, but this value is not passed as an argument
            if (exemplar_score == 0) {
                // found a perfect solution
                deme.resize(1);
                *(deme.begin()++) = exemplar;
                m_state = M_INIT;
            } else
                m_state = M_BUILD_CANDIDATES;

            break;
        }
        // --------------------------------------------------------------------------

        case M_BUILD_CANDIDATES:  {

            if (current_number_of_instances >= max_number_of_instances) {
                m_state = M_INIT;
                return EVALUATED_ALL_AVAILABLE; // This is ugly, see the note in moses.h
            }

            if (distance > MAX_DISTANCE_FROM_EXEMPLAR || distance > number_of_fields) {
                m_state = M_INIT;
                return EVALUATED_ALL_AVAILABLE; // This is ugly, see the note in moses.h
            }

            cout << "Distance in this iteration:" << distance << endl;

            // the number of all neighbours at the distance d
            long long total_number_of_neighbours = count_n_changed_knobs(deme.fields(), distance);
            cout << "Number of possible instances:" << total_number_of_neighbours << endl;

            if (total_number_of_neighbours == 0) {
                m_state = M_INIT;
                return EVALUATED_ALL_AVAILABLE; // This is ugly, see the note in moses.h
            }

            number_of_new_instances = (max_number_of_instances - current_number_of_instances) / FRACTION_OF_REMAINING;
            if (number_of_new_instances  < MINIMUM_DEME_SIZE)
                number_of_new_instances = (max_number_of_instances - current_number_of_instances);

            if (number_of_new_instances < total_number_of_neighbours) {
                //resize the deme so it can take new instances
                deme.resize(current_number_of_instances + number_of_new_instances);
                //sample 'number_of_new_instances' instances on the distance 'distance' from the exemplar
                sample_from_neighborhood(deme.fields(), distance, number_of_new_instances, deme.begin() + current_number_of_instances, rng);
            } else {
                number_of_new_instances = total_number_of_neighbours;
                //resize the deme so it can take new instances
                deme.resize(current_number_of_instances + number_of_new_instances);
                //add all instances on the distance 'distance' from the exemplar
                generate_all_in_neighborhood(deme.fields(), distance, deme.begin() + current_number_of_instances);
            }

            target_size = current_number_of_instances + number_of_new_instances;
            cout << "New target size:" << target_size << endl;

            m_state = M_EVALUATE_CANDIDATES;

            break;
        }
        // --------------------------------------------------------------------------

        case M_EVALUATE_CANDIDATES: {

            bool bImprovement_made = false;
            int n = 0; // number of evaluations in this chunk

            score_t best_score = exemplar_score;

            // cout << " _evals_per_slice " << _evals_per_slice << endl;

            // check if there is an instance in the deme better than the exemplar
            for (int i = 0; deme.begin() + current_number_of_instances != deme.end() &&
                    i < _evals_per_slice && !bImprovement_made; i++) {

                *(deme.begin_scores() + current_number_of_instances) = score(*(deme.begin() + current_number_of_instances));

                const eda::scored_instance<combo_tree_score>& inst = deme[current_number_of_instances];

                if (get_score(inst.second) > best_score) {

                    best_score = get_score(inst.second);
                    bImprovement_made = true;

                    cout << endl << "Improvement, new best score:" << endl;
                    cout << inst.second << "(distance: " << distance << ", old score: " << exemplar_score << ")" << endl;
                    cout << "Found instance:" << deme.fields().stream(inst) << endl;
                    cout << "Score:" << get_score(inst.second) << endl;
                    cout << "--------------------------------------" << endl;
                }

                current_number_of_instances++;
                n++;
            }

            if (bImprovement_made) {
                m_state = M_INIT;
                return -n;
            } else if (deme.begin() + current_number_of_instances < deme.end())  {
                m_state = M_EVALUATE_CANDIDATES;
            } else {
                m_state = M_BUILD_CANDIDATES;
                distance++;
            }

            return n;
            break;
        }
        // --------------------------------------------------------------------------

        default:
            break;
        }

        return 0;
    }

    int distance;
    long long number_of_new_instances;
    int number_of_fields;
    eda::instance exemplar;
    int max_number_of_instances;
    score_t exemplar_score;
    int current_number_of_instances;
    int target_size;

    opencog::RandGen& rng;
    eda_parameters params;
    MState m_state;
    int _evals_per_slice;
};

} //~namespace moses



#endif

