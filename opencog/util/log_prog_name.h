/** log_prog_name.h --- 
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


#ifndef _OPENCOG_LOG_PROG_NAME_H
#define _OPENCOG_LOG_PROG_NAME_H

#include <boost/program_options.hpp>

namespace opencog {

/**
 * This file contains a little function to determine a log file name
 * automatically depending on its boost.program_option.
 */


// used to convert program option argument to string.
// @todo: ugly, it is likely something better can be done using
// boost::program_options API
template<typename T>
bool to_string(const boost::program_options::variable_value& vv,
               std::string& str,
               std::string separator)
{
    if(vv.value().type() == typeid(T)) {
        str = boost::lexical_cast<std::string>(vv.as<T>());
        return true;
    } else if(vv.value().type() == typeid(std::vector<T>)) {
        str = opencog::containerToStr(vv.as<std::vector<T> >(), separator.c_str());
        return true;
    }
    return false;
}
std::string to_string(const boost::program_options::variable_value& vv,
                      std::string separator = "_")
{
    std::string res;
    if(!(to_string<int>(vv, res, separator)
         || to_string<unsigned int>(vv, res, separator)
         || to_string<long>(vv, res, separator)
         || to_string<unsigned long>(vv, res, separator)
         || to_string<float>(vv, res, separator)
         || to_string<double>(vv, res, separator)
         || to_string<bool>(vv, res, separator)
         || to_string<std::string>(vv, res, separator)))
        std::cerr << "type not handled yet" << std::endl;
    return res;
}

/**
 * determine a log file name automatically depending on its
 * boost.program_option.
 */
std::string determine_log_name(const std::string& log_file_prefix,
                               const boost::program_options::variables_map& vm,
                               const std::set<std::string>& ignore_opt,
                               const std::string& log_file_suffix = ".log")
{
    const static unsigned int max_filename_size = 255;

    std::string log_file = log_file_prefix;
    for(boost::program_options::variables_map::const_iterator it = vm.begin();
        it != vm.end(); it++)
        // we ignore the options in ignore_opt and any default one
        if(ignore_opt.find(it->first) == ignore_opt.end()
           && !it->second.defaulted()) {
            std::string str = std::string("_") + it->first + "_" + to_string(it->second);
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

#endif // _OPENCOG_LOG_PROG_NAME_H
