/*
 * opencog/util/Config.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#include "Config.h"

#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>

#include <errno.h>

#include <boost/filesystem/operations.hpp>

#include <opencog/util/platform.h>
#include <opencog/util/exceptions.h>

using namespace opencog;
using namespace std;

// returns a string with leading/trailing characters of a set stripped
static char const* blank_chars = " \t\f\v\n\r";
static string strip(string const& str, char const *strip_chars = blank_chars)
{
    string::size_type const first = str.find_first_not_of(strip_chars);
    return (first == string::npos) ? string() : str.substr(first, str.find_last_not_of(strip_chars) - first + 1);
}

Config::~Config()
{
}

Config::Config()
{
    reset();
}

void Config::reset()
{
    table.clear();
    // load default configuration
    for (unsigned int i = 0; DEFAULT()[i] != ""; i += 2) {
        if (table.find(DEFAULT()[i]) == table.end()) {
            table[DEFAULT()[i]] = DEFAULT()[i + 1];
        }
    }
}

static const char* DEFAULT_CONFIG_FILENAME = "opencog.conf";
static const char* DEFAULT_CONFIG_PATHS[] =
{
    // A bunch of relative paths, typical for the current opencog setup.
    "./",
    "../",
    "../../",
    "../../../",
    "./lib/",
    "../lib/",
    "../../lib/",
    "../../../lib/",
    CONFDIR,
#ifndef WIN32
    "/etc",
#endif // !WIN32
    NULL
};


// constructor
void Config::load(const char* filename)
{
    if (filename == NULL) filename = DEFAULT_CONFIG_FILENAME;

    // Reset to default values
    reset();

    // Search for the filename in a bunch of typical locations.
    ifstream fin;
    for (int i = 0; DEFAULT_CONFIG_PATHS[i] != NULL; ++i)
    {
        boost::filesystem::path configPath(DEFAULT_CONFIG_PATHS[i]);
        configPath /= filename;
        if (boost::filesystem::exists(configPath))
        {
            // Read and process the config file
            fin.open(configPath.string().c_str());
            if (fin && fin.good() && fin.is_open()) break;
        }
    }

    // Whoops, failed.
    if (!fin || !fin.good() || !fin.is_open())
        throw IOException(TRACE_INFO,
             "[ERROR] unable to open file \"%s\"", filename);

    string line;
    string name;
    string value;
    unsigned int line_number = 0;

    while (++line_number, fin.good() && getline(fin, line)) {
        string::size_type idx;
        // find comment and discard the rest of the line
        if ((idx = line.find('#')) != string::npos) {
            line.replace(idx, line.size() - idx, "");
        }
        // search for the '=' character
        if ((idx = line.find('=')) != string::npos) {
            // select name and value
            name  = line.substr(0, idx);
            value = line.substr(idx + 1);
            // strip them
            name  = strip(name);
            value = strip(value);
            value = strip(value, "\"");
            // finally, store the entries
            table[name] = value;
        } else if (line.find_first_not_of(blank_chars) != string::npos) {
            throw InvalidParamException(TRACE_INFO,
                  "[ERROR] invalid configuration entry (line %d)", line_number);
        }
    }
    fin.close();
}

const bool Config::has(const string &name) const
{
    return (table.find(name) != table.end());
}

void Config::set(const std::string &parameter_name,
                 const std::string &parameter_value)
{
    table[parameter_name] = parameter_value;
}

const string& Config::get(const string &name) const
{
    map<string, string>::const_iterator it = table.find(name);
    if (it == table.end())
       throw InvalidParamException(TRACE_INFO,
               "[ERROR] parameter not found (%s)", name.c_str());
    return it->second;
}

const string& Config::operator[](const string &name) const
{
    return get(name);
}

int Config::get_int(const string &name) const
{
    int int_val;
    errno = 0;
    int_val = strtol(get(name).c_str(), NULL, 0);
    if (errno != 0)
        throw InvalidParamException(TRACE_INFO,
               "[ERROR] invalid integer parameter (%s)", name.c_str());
    return int_val;
}

double Config::get_double(const string &name) const
{
    double int_val;
    errno = 0;
    int_val = strtod(get(name).c_str(), NULL);
    if (errno != 0)
        throw InvalidParamException(TRACE_INFO,
                "[ERROR] invalid double parameter (%s: %s)",
                 name.c_str(), get(name).c_str());
    return int_val;
}

bool Config::get_bool(const string &name) const
{
    if (strcasecmp(get(name).c_str(), "true") == 0) return true;
    else if (strcasecmp(get(name).c_str(), "false") == 0) return false;
    else throw InvalidParamException(TRACE_INFO,
                 "[ERROR] invalid double parameter (%s: %s)",
                 name.c_str(), get(name).c_str());
}

std::string Config::to_string() const
{
    std::ostringstream oss;
    oss << "{\"";
    for (map<string, string>::const_iterator it = table.begin(); it != table.end(); ++it) {
        if (it != table.begin()) oss << "\", \"";
        oss << it->first << "\" => \"" << it->second;
    }
    oss << "\"}";
    return oss.str();
}

// create and return the single instance
Config& opencog::config()
{
    static Config instance;
    return instance;
}
