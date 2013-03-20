/** eval-candidate.cc --- 
 *
 * Copyright (C) 2013 OpenCog Foundation
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

/**
 * Tool to evaluate the likelihood of a candidate solution. This can
 * be used for instance in model combination to weight each candidate.
 */

#include <boost/math/special_functions/binomial.hpp>
#include <boost/range/algorithm_ext/for_each.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>

#include <opencog/comboreduct/combo/iostream_combo.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/iostreamContainer.h>
#include <opencog/comboreduct/table/table_io.h>
#include <opencog/learning/moses/moses/types.h>
#include <opencog/learning/moses/moses/complexity.h>

#include "eval-candidate.h"

using namespace std;
using namespace boost::program_options;
using namespace opencog;
using namespace moses;
using namespace combo;
using boost::str;
using boost::trim;
using boost::math::binomial_coefficient;

/**
 * Convert a string representing a combo program in a combo_tree.
 *
 * @param combo_prog_str   the string containing the combo program
 * @param labels           a vector of labels
 * @return                 the combo_tree
 */
combo_tree str2combo_tree_label(const std::string& combo_prog_str,
                                const std::vector<std::string>& labels) {
    // combo pogram with place holders
    std::string combo_prog_ph_str = l2ph(combo_prog_str, labels);
    std::stringstream ss(combo_prog_ph_str);
    combo_tree tr;
    ss >> tr;
    return tr;
}

vector<string> get_all_combo_tree_str(const eval_candidate_params& ecp)
{
    vector<string> res;
    // from files
    for (const std::string& combo_prg : ecp.combo_program_files) {
        ifstream in(combo_prg);
        if (in) {
            while (in.good()) {
                string line;
                getline(in, line);
                if(line.empty())
                    continue;
                res.push_back(line);
            }
        } else {
            logger().error("Error: file %s can not be found.",
                           combo_prg.c_str());
            exit(1);
        }
    }

    return res;
}

/**
 * Let's P(M|D) the probability of having model M explain data D, we
 * want to estimate that and use it as weight during voting.
 *
 *   P(M|D) = P(D|M)*P(M)/P(D)
 *
 * P(D) is constant so we don't care about it
 *
 * P(M) is the model distribution prior, we use the Solomonoff-ish
 * approximation:
 *
 *     P(M) = |A|^-|M|
 *          = exp(-|M|*log(|A|))
 *
 * So let's focus on P(D|M).
 *
 * The formula of P(D|M) is problem specific. Let's first define
 *
 * (x1, y1), ..., (xn, yn) be the IOs observed, i.e. D.
 *
 * Let m1, ..., mn be the outputs computed by M.
 *
 * Only the boolean case is treated here so xi, yi and mi have the domain {0,1}
 *
 * 1) For problem it
 *
 *     P(D|M) = Prod_{i=1}^n p*(mi != yi) + (1-p)*(mi == yi)
 *
 * 2) For problem prerec
 *
 * Let R be the minimum recall.
 *
 * Let p be the probability that xi is wrong (for instance x1 is
 * observed as 1 while is it in fact 0).
 *
 * Let's define
 *
 *     P(D|M) = P(D|M restricted to positive) * P(recall(M) >= R)
 *
 *     P(D|M restricted to positive) = Prod_{i=1}^n P(yi|mi)
 *
 * where
 *
 *     P(yi|mi) = if (mi == 0) then 1 else p*!yi + (1-p)*yi
 *
 * that is, the probability that model M restricted to positive data
 * points explains D. Now let's formalize P(recall(M) >= R)
 *
 *     P(recall(M) >= R) = Sum_{s in S} P(recall(s) >= R)
 *
 * where S is the set of bit-strings of size n representing whether xi
 * is true or false.
 *
 *     P(recall(s) >= R) = if (recall(s) >= R) then P(s) else 0
 *
 * Let's simplify as it is computationally intractable.
 *
 * First by convention we say that recall(s) = 1 for s=(0)^n, i.e. s
 * has only false values.
 *
 * Let be:
 *
 *    N0 = number of 0s in the target over negative data points
 *    N1 = number of 1s in the target over negative data points
 *    A0 = number of 0s in the target over positive data points
 *    A1 = number of 1s in the target over positive data points
 *
 * Let's first notice that if some s has K 1s in its negative part
 * and L 1s in its positive part, then
 *
 *    recall(s) <=> L/(K+L) >= R, by definition
 *    recall(s) <=> L >= (K+L)*R, transforming...
 *    recall(s) <=> L >= K*R+L*R, transforming...
 *    recall(s) <=> L - L*R >= K, transforming...
 *    recall(s) <=> (1-R)*L >= K, transforming...
 *    recall(s) <=> L >= K/(1-R), voila.
 *
 * Now the probability can be written down
 *
 * P(recall(M) >= R) = Sum_{0 <= i <= N0,
 *                          0 <= j <= N1,
 *                          0 <= k <= A0,
 *                          0 <= h <= A1,
 *                          (i+j)/(1-R) <= k+h}
 *          C(N0, i) * C(N1, j) * C(A0, k) * C(A1, h) *
 *          p^(i + N1-j + k + A1-h) * (1-p)^(N0-i + j + A0-k + h)
 *
 * You may note that K corresponds to i+j and L to k+h in the last
 * constraint of the sum: (i+j)/(1-R)<= k+h, which is the part that
 * allows to sum over only s that have recall above R.
 */
double likelihood(const combo_tree& tr, const Table& table,
                  eval_candidate_params ecp) {
    OTable ot_tr(tr, table.itable);
    arity_t arit = table.get_arity();
    complexity_t cplx = tree_complexity(tr);
    double p = ecp.noise;

    // P(M)
    double PM = exp(-cplx * log(arit));

    // init P(D|M)
    double PDM = 1.0;

    if (ecp.problem == it) {
        // compute P(D|M)
        boost::for_each(table.otable, ot_tr,
                        [&](const vertex& y, const vertex& m) {
                            PDM *= p*(m != y) + (1-p)*(m == y); });
    } else if (ecp.problem == prerec) {
        // compute P(D|M restricted to positive)
        double PDM_rp = 1.0;
        unsigned N0 = 0, N1, A0 = 0, A1 = 0;
        for (unsigned i = 0; i < ot_tr.size(); i++) {
            bool m = vertex_to_bool(ot_tr[i]);
            bool y = vertex_to_bool(table.otable[i]);
            // count N0, N1, A0, A1
            N0 += !m && !y;
            N1 += !m && y;
            A0 += m && !y;
            A1 += m && y;
            // update P(D|M restricted to positive)
            if (m)
                PDM_rp *= p*!y+ (1-p)*y;
        }
        // compute P(recall(M) >= R)
        double PrMR = 0.0;
        double R = ecp.min_fixed;
        for (unsigned i = 0; i <= N0; i++)
            for (unsigned j = 0; j <= N1; j++)
                for (unsigned k = 0; k <= A0; k++)
                    for (unsigned h = 0; h <= A1; h++)
                        if ((i+j)/(1-R) <= k+h)
                            PrMR += binomial_coefficient<double>(N0, i)
                                * binomial_coefficient<double>(N1, j)
                                * binomial_coefficient<double>(A0, k)
                                * binomial_coefficient<double>(A1, h)
                                * power(p, i + N1-j + k + A1-h)
                                * power(1-p, N0-i + j + A0-k + h);
        // compute P(D|M)
        PDM = PDM_rp * PrMR;
    } else {
        OC_ASSERT(false, "likelihood for problem %s is not implemented",
                  ecp.problem.c_str());
    }
    return PDM * PM;
}

int main(int argc, char** argv) {
    eval_candidate_params ecp;
    unsigned long rand_seed;

    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")

        ("r,random-seed", value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")

        // IO
        ("input-file,i", value<string>(&ecp.input_file),
         "DSV file containing inputs and outputs.\n")

        ("target-feature,u", value<string>(&ecp.target_feature_str),
         "Name of the target feature.\n")

        ("combo-program-file,C", value<vector<string>>(&ecp.combo_program_files),
         "File containing combo programs. "
         "Can be used several times for several files.\n")

        ("output-file,o", value<string>(&ecp.output_file),
         "File to write the results. If none is given it write on the stdout.\n")

        // Parameters
        ("problem,H", value<string>(&ecp.problem)->default_value(it),
         str(boost::format("Problem to solve, supported problems are:\n\n"
                           "%s, regression based on input table, maximizing accuracy\n\n"
                           "%s, regression based on input table, maximizing precision, while holding recall fixed.\n")
             % it % prerec).c_str())

        ("noise,p", value<float>(&ecp.noise)->default_value(0.0),
         "Assumes that the data is noisy.   The noisier the data, the "
         "stronger the model complexity penalty.  If the target feature "
         "is discrete, the setting should correspond to the fraction of "
         "the input data that might be wrong (i.e. the probability p "
         "that an output datum (row) is wrong).   In this case, only "
         "values 0 <= p < 0.5 are meaningful (i.e. less than half the "
         "data can be wrong). Suggested values are in the range 0.01 to "
         "0.001.  If the target feature is continuous, the value specified "
         "should correspond to the standard deviation of the (Gaussian) "
         "noise centered around each candidate's output.\n")

        ("normalize,n", value<bool>(&ecp.normalize)->default_value(false),
         "Normalize the output all weights so that they sums up to 1.\n")

        // prerec parameters
        ("min-fixed", value<float>(&ecp.min_fixed)->default_value(1.0),
         "For prerec problem set the minimum recall")

        ;

    variables_map vm;
    try {
        store(parse_command_line(argc, argv, desc), vm);
    }
    catch (error& e) {
        OC_ASSERT(false, "Fatal error: invalid or duplicated argument:\n\t%s\n",
                  e.what());
    }
    notify(vm);

    if (vm.count("help") || argc == 1) {
        cout << desc << endl;
        return 1;
    }

    // init random generator
    randGen().seed(rand_seed);

    // get all combo tree strings (from command line and file)
    vector<string> all_combo_tree_str = get_all_combo_tree_str(ecp);

    OC_ASSERT(all_combo_tree_str.size() == 1,
              "Using more than 1 combo, not implemented yet!");

    // read data ITable (using ignore_variables)
    Table table;
    if (ecp.target_feature_str.empty())
        table.itable = loadITable_optimized(ecp.input_file);
    else {
        table = loadTable_optimized(ecp.input_file,
                                    ecp.target_feature_str);
    }

    ITable& it = table.itable;

    // parse combo programs
    vector<combo_tree> trs;
    for (const string& tr_str : all_combo_tree_str) {
        combo_tree tr = str2combo_tree_label(tr_str, it.get_labels());
        if (logger().isFineEnabled()) {
            logger().fine() << "Combo str: " << tr_str;
            logger().fine() << "Parsed combo: " << tr;
        }
        trs.push_back(tr);
    }

    // Evaluate the likelihood of each program
    vector<double> ls(trs.size());          // likelihoods
    boost::transform(trs, ls.begin(), [&](const combo_tree& tr) {
            return likelihood(tr, table, ecp); });

    // Normalize
    if (ecp.normalize) {
        double sum = boost::accumulate(ls, 0.0);
        boost::transform(ls, ls.begin(), [&](const double l) { return l/sum; });
    }

    // Output the values
    if(ecp.output_file.empty())
        ostreamContainer(cout, ls, "\n");
    else {
        ofstream of(ecp.output_file.c_str());
        ostreamContainer(of, ls, "\n");
    }
}
