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

using namespace ant_combo;

// structure containing the options for the eval-table program
struct evalTableParameters {
    evalTableParameters(std::string _input_table_file,
                        std::string _combo_prog_str,
                        bool _has_labels,
                        std::string _output_file)
        : input_table_file(_input_table_file),
          combo_prog_str(_combo_prog_str),
          has_labels(_has_labels),
          output_file(_output_file) {}
    std::string input_table_file;
    std::string combo_prog_str;
    bool has_labels;
    std::string output_file;
};

template<typename Out, typename OT>
Out& output_results(Out& out, const OT& ot) {
    out << ot << std::endl; // print output table

    // TODO print residual error and what more...

    return out;
}

template<typename OT>
void output_results(const std::string& output_file, const OT& ot) {
    if(output_file.empty())
        output_results(std::cout, ot);
    else {
        std::ofstream of(output_file.c_str());
        output_results(of, ot);
        of.close();        
    }
}

template<typename IT, typename OT>
void eval_output_results(const std::string& output_file, const combo_tree& tr,
                         const IT& it, const OT& ot, opencog::RandGen& rng) {
    // evaluated tr over input table
    OT ot_tr(tr, it, rng);
    ot_tr.set_label(ot.get_label());

    // print results
    output_results(output_file, ot_tr);
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
    eval_output_results(pa.output_file, tr, it, ot, rng);
}

#endif // _OPENCOG_EVAL_TABLE_H
