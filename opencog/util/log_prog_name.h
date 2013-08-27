/* log_prog_name.h --- 
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

#ifndef _OPENCOG_LOG_PROG_NAME_H
#define _OPENCOG_LOG_PROG_NAME_H

#include <boost/program_options.hpp>
#include "iostreamContainer.h"

namespace opencog {
/** \addtogroup grp_cogutil
 *  @{
 */

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
                      std::string separator = "_");

/**
 * determine a log file name automatically depending on its
 * boost.program_option.
 */
std::string determine_log_name(const std::string& log_file_prefix,
                               const boost::program_options::variables_map& vm,
                               const std::set<std::string>& ignore_opt,
                               const std::string& log_file_suffix = ".log");

/** @}*/
} // ~namespace opencog

#endif // _OPENCOG_LOG_PROG_NAME_H
