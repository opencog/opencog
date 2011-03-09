/** eval-table.h --- 
 *
 * Copyright (C) 2011 Nil Geisweiller
 *
 * Author: Nil Geisweiller <nilg@desktop>
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


#ifndef _OPENCOG_EVAL_TABLE_H
#define _OPENCOG_EVAL_TABLE_H

//ant_combo_vocabulary is used only for the boolean core vocabulary
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>
#include <opencog/comboreduct/combo/table.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/numeric.h>

using namespace ant_combo;

// structure containing the options for the eval-table program
struct evalTableParameters {
    std::string input_table_file;
    std::string combo_prog_str;
    bool has_labels;
    std::vector<std::string> features;
    bool compute_MI;
    bool display_output_table;
    std::string output_file;
};

template<typename Out, typename OT>
Out& output_results(Out& out, const evalTableParameters& pa, const OT& ot,
                    double mi) {
    if(pa.compute_MI)
        out << mi << std::endl; // print mutual information
    if(pa.display_output_table)
        out << ot << std::endl; // print output table
    return out;
}

template<typename OT>
void output_results(const evalTableParameters& pa, const OT& ot, double mi) {
    if(pa.output_file.empty())
        output_results(std::cout, pa, ot, mi);
    else {
        std::ofstream of(pa.output_file.c_str());
        output_results(of, pa, ot, mi);
        of.close();        
    }
}

std::set<arity_t> get_features_idx(const evalTableParameters& pa) {
    std::set<arity_t> res;
    vector<std::string> labels = read_data_file_labels(pa.input_table_file);
    foreach(const std::string& f, pa.features) {
        arity_t idx = std::distance(labels.begin(), find(labels, f));
        OC_ASSERT((size_t)idx != labels.size(),
                  "No such a feature %s in file %s",
                  f.c_str(), pa.input_table_file.c_str());
        res.insert(idx);
    }
    return res;
}

template<typename IT, typename OT>
void eval_output_results(const evalTableParameters& pa, const combo_tree& tr,
                         const IT& it, const OT& ot, opencog::RandGen& rng) {
    // evaluated tr over input table
    OT ot_tr(tr, it, rng);
    ot_tr.set_label(ot.get_label());

    // compute MI
    double mi = pa.compute_MI?
        mutualInformation(it, ot_tr, get_features_idx(pa)) : 0;
    // print results
    output_results(pa, ot_tr, mi);
}

template<typename IT, typename OT, typename Type>
void read_eval_output_results(const evalTableParameters& pa,
                              opencog::RandGen& rng) {
    IT it;
    OT ot;

    // read data table
    istreamTable<IT, OT, Type>(pa.input_table_file, it, ot);
    
    // read combo program
    combo_tree tr = str2combo_tree_label(pa.combo_prog_str,
                                         pa.has_labels, it.get_labels());

    // eval and output the results
    eval_output_results(pa, tr, it, ot, rng);
}

#endif // _OPENCOG_EVAL_TABLE_H
