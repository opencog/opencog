/*
 * opencog/util/StringManipulator.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#include <cctype>
#include <boost/regex.hpp>
#include "StringManipulator.h"
using namespace opencog;

#define isvalidtoken(ch) ((ch) == ')' || (ch)== '(' || (ch) == ',' )

std::string StringManipulator::toUpper(const std::string& str) {
    std::string result;

    for (char _c : str) {
        result.push_back(toupper(_c));
    }

    return result;
}

std::string StringManipulator::toLower(const std::string& str) {
    std::string result;

    for (char _c : str) {
        result.push_back(tolower(_c));
    }

    return result;
}

std::string StringManipulator::clean(const std::string& str) {
    std::string result;
    bool started = false;

    for (char _c : str) {
        if (isalpha(_c) || isdigit(_c) ||
                isvalidtoken(_c) ||  _c == '_' || _c == char(39) ||
                (started && _c == ' ')) {

            result.push_back((_c > 'Z' && isalpha(_c)) ? (_c - ('z'-'Z')) : _c);
            started = true;
        }
    }

    return result;
}

std::vector<std::string> StringManipulator::split(std::string& str, const std::string& delimiter)
{
    std::string::size_type cutAt;
    std::vector<std::string> tokens;

    while ((cutAt = str.find(delimiter)) != std::string::npos) {
        if (cutAt > 0) {
            tokens.push_back(str.substr(0, cutAt));
        }
        str = str.substr(cutAt + 1);
    }
    if (str.length() > 0) {
        tokens.push_back(str);
    }

    return tokens;
}

void StringManipulator::trim(std::string& str)
{
    std::string::size_type pos = str.find_last_not_of(' ');
    if (pos != std::string::npos) {
        str.erase(pos + 1);
        pos = str.find_first_not_of(' ');
        if (pos != std::string::npos) {
            str.erase(0, pos);
        }
    } else {
        str.erase(str.begin(), str.end());
    }
}

bool StringManipulator::isNumber(const std::string& str) {

    static const boost::regex re("[-\\+]?\\d*\\.?\\d+");
    return boost::regex_match(str, re);
}
