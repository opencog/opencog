/** eval-candidate.h --- 
 *
 * Copyright (C) 2013 OpenCog Foundation
 * Copyright (C) 2014 Aidyia Limited
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

#ifndef _OPENCOG_EVAL_CANDIDATE_H
#define _OPENCOG_EVAL_CANDIDATE_H

namespace opencog { namespace moses {

// structure holding the options
struct eval_candidate_params
{
    // IO
    std::string input_file;
    std::vector<std::string> combo_programs;
    std::vector<std::string> combo_program_files;
    std::vector<std::string> output_files;
    bool output_with_labels;
    std::string target_feature_str;

    // parameters
    std::string problem;
    unsigned jobs;

    // problem params, for the "pre" problem.
    double activation_pressure;
    double min_activation;
    double max_activation;
    bool pre_positive;
};

// problems
static const std::string f_one="f_one";

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_EVAL_CANDIDATE_H
