/** gen-table.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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


#ifndef _OPENCOG_GEN_TABLE_H
#define _OPENCOG_GEN_TABLE_H

#include <boost/program_options.hpp>

namespace opencog { namespace combo {

using std::pair;
using std::string;
        
// program option names and abbreviations
// for their meanings see options_description in moses-exec.cc
static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> combo_program_opt("combo-program", "y");
static const pair<string, string> combo_program_file_opt("combo-program-file", "f");
static const pair<string, string> nsamples_opt("nsamples", "n");
static const pair<string, string> min_contin_opt("min-contin", "m");
static const pair<string, string> max_contin_opt("max-contin", "M");
static const pair<string, string> header_opt("header", "H");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> target_index_opt("target_index", "t");

} // ~namespace combo
} // ~namespace opencog

#endif // _OPENCOG_GEN-TABLE_H
