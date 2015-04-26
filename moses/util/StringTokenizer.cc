/*
 * moses/util/StringTokenizer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Carlos Lopes <dlopes@vettalabs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://moses.org/wiki/Licenses
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

#include "StringTokenizer.h"

#include <cstring>

#include <moses/util/exceptions.h>
#include <moses/util/oc_assert.h>



using namespace moses;

StringTokenizer::StringTokenizer()
{
    reset();

    str.assign("");
    delimiter.assign("");
}

StringTokenizer::StringTokenizer(const std::string &str, const std::string &delimiter)
{
    reset();

    this->str = str;
    this->delimiter = delimiter;
}

StringTokenizer::~StringTokenizer()
{
}

std::string StringTokenizer::getString()
{
    return str;
}

void StringTokenizer::setString(const std::string &str)
{
    this->str = str;
}

const std::string & StringTokenizer::getDelimiter()
{
    return delimiter;
}

void StringTokenizer::setDelimiter(const std::string &str)
{
    this->delimiter = str;
}

void StringTokenizer::reset()
{
    start = 0;
    end = 0;
}

const std::string StringTokenizer::nextToken()
{
    OC_ASSERT(str != "", "StringTokenizer - string should not be empty.");
    OC_ASSERT(delimiter != "", "StringTokenized - delimiter should not be empty.");

    // end of the string
    if (end == str.size()) {
        return "";
    }

    if (start ==  0 && end == 0) {
        end = str.find(delimiter);
        if ( end == std::string::npos ) {
            end = str.size();
        }
        return str.substr(start, end - start);
    }

    do {
        start = end + delimiterSize();
        if (start == str.size()) {
            end = start;
            return "";
        }
        end = str.find(delimiter, start);
        if (end == std::string::npos) {
            end = str.size();
        }
    } while ( str.substr(start, end - start) == delimiter || end == start);

    return str.substr(start, end - start);
}

std::string::size_type StringTokenizer::delimiterSize()
{
    return delimiter.size();
}


AltStringTokenizer::AltStringTokenizer(const std::string &rStr, const std::string &rDelimiters)
{
    std::string::size_type lastPos(rStr.find_first_not_of(rDelimiters, 0));
    std::string::size_type pos(rStr.find_first_of(rDelimiters, lastPos));
    while (std::string::npos != pos || std::string::npos != lastPos) {
        push_back(rStr.substr(lastPos, pos - lastPos));
        lastPos = rStr.find_first_not_of(rDelimiters, pos);
        pos = rStr.find_first_of(rDelimiters, lastPos);
    }
}

std::vector<std::string> AltStringTokenizer::WithoutEmpty() const
{
    std::vector<std::string> ret;

    for (unsigned int i = 0; i < this->size(); i++)
        if (!(*this)[i].empty())
            ret.push_back((*this)[i]);

    return ret;
}

