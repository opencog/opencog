/*
 * opencog/rest/BaseURLHandler.h
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#include "BaseURLHandler.h"

#include <map>
#include <boost/algorithm/string.hpp>

#define SERVER_PLACEHOLDER "REST_SERVER_ADDRESS"
namespace opencog {
    
std::list<std::string> BaseURLHandler::splitQueryString(char* query) {
    using namespace std;
    list<string> params;
    if (query == NULL) return params;
    string query_string(query);
    boost::split(params, query_string, boost::is_any_of("&;"));
    return params;
}

std::map<std::string,std::string> BaseURLHandler::paramsToMap(const std::list<std::string>& params) 
{
    using namespace std;
    map<string,string> query_map;
    list<string>::const_iterator p_i;
    for (p_i = params.begin(); p_i != params.end(); p_i++) {
        std::vector<string> keyAndValue;
        boost::split(keyAndValue, *p_i, boost::is_any_of("="));
        if (keyAndValue.size() != 2) continue;
        query_map.insert(pair<string,string>(keyAndValue[0], keyAndValue[1]));
    }
    return query_map;
}

std::string BaseURLHandler::replaceURL(const std::string server_string) 
{
    std::string output = request_output.str();
    boost::replace_all(output, SERVER_PLACEHOLDER,
            server_string);
    return output;
}

void BaseURLHandler::SendResult(const std::string& res) {
    request_output << res;
}

} // namespace
