/** log_prog_name.cc --- 
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

#include "log_prog_name.h"
#include "oc_assert.h"

namespace opencog {

using namespace std;

/**
 * This file contains a little function to determine a log file name
 * automatically depending on its boost.program_option.
 */

string to_string(const boost::program_options::variable_value& vv,
                 string separator)
{
    string res;
    if(!(to_string<int>(vv, res, separator)
         || to_string<unsigned int>(vv, res, separator)
         || to_string<long>(vv, res, separator)
         || to_string<unsigned long>(vv, res, separator)
         || to_string<float>(vv, res, separator)
         || to_string<double>(vv, res, separator)
         || to_string<bool>(vv, res, separator)
         || to_string<string>(vv, res, separator)))
        cerr << "type not handled yet" << endl;
    return res;
}

/**
 * determine a log file name automatically depending on its
 * boost.program_option.
 */
string determine_log_name(const string& log_file_prefix,
                          const boost::program_options::variables_map& vm,
                          const set<string>& ignore_opt,
                          const string& log_file_suffix)
{
    const static unsigned int max_filename_size = 255;

    string log_file = log_file_prefix;
    for(boost::program_options::variables_map::const_iterator it = vm.begin();
        it != vm.end(); ++it)
        // we ignore the options in ignore_opt and any default one
        if(ignore_opt.find(it->first) == ignore_opt.end()
           && !it->second.defaulted()) {
            string str = string("_") + it->first + "_" + to_string(it->second);
            // this is because OSs usually do not handle file name
            // above 255 chars
            unsigned int expected_max_size =
                log_file.size()+str.size()+log_file_suffix.size();
            if(expected_max_size < max_filename_size) {
                log_file += str;
            }
        }
    log_file += log_file_suffix;
    // replace / by d because unix file name cannot have / in it
    replace(log_file.begin(), log_file.end(), '/', 'd');
    OC_ASSERT(log_file.size() <= max_filename_size);
    return log_file;
}

} // ~namespace opencog
