/** eval-candidate.h --- 
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

#ifndef _OPENCOG_EVAL_CANDIDATE_LIKELIHOOD_H
#define _OPENCOG_EVAL_CANDIDATE_LIKELIHOOD_H

namespace opencog { namespace moses {

// structure holding the options
struct eval_candidate_likelihood_params
{
    // IO
    std::string input_file;
    std::vector<std::string> combo_program_files;
    std::string output_file,
        target_feature_str;
    // parameters
    std::string problem;
    double noise;
    bool normalize;
    double complexity_amplifier;
    // prerec parameters
    double prerec_min_recall;
    bool prerec_simple_precision;
};

// problems
static const std::string it="it"; // regression based on input table
                                  // maximize accuracy.

static const std::string prerec="prerec"; // regression based on input table,
                                          // maximize precision, while holding
                                          // recall const.

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_EVAL_CANDIDATE_H
