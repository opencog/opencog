/*
 * opencog/util/Config.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
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

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>

#include <errno.h>
#include <unistd.h>

// For backward compatibility as from boost 1.46 filesystem 3 is the default
// as of boost 1.50 there is no version 2, and compiles will fail ;-(
#include <boost/version.hpp>
#if BOOST_VERSION > 104900
#define BOOST_FILESYSTEM_VERSION 3
#else
#define BOOST_FILESYSTEM_VERSION 2
#endif
#include <boost/filesystem/operations.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <opencog/util/platform.h>
#include <opencog/util/exceptions.h>

using namespace opencog;
using namespace std;

// Returns a string with leading/trailing characters of a set stripped
static char const* blank_chars = " \t\f\v\n\r";
static string strip(string const& str, char const *strip_chars = blank_chars)
{
    string::size_type const first = str.find_first_not_of(strip_chars);
    return (first == string::npos) ? 
        string() : 
        str.substr(first, str.find_last_not_of(strip_chars) - first + 1);
}


Config* Config::createInstance()
{
    return new Config();
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
    "../../../../",
    "./lib/",
    "../lib/",
    "../../lib/",
    "../../../lib/",
    "../../../../lib/", // yes, really needed for some test cases!
    CONFDIR,
    "", // If you don't have this, then absolute paths won't work!
#ifndef WIN32
    "/etc/opencog",
    "/etc",
#endif // !WIN32
    NULL
};


// constructor
void Config::load(const char* filename, bool resetFirst)
{
    if (filename == NULL) filename = DEFAULT_CONFIG_FILENAME;

    if (resetFirst) {
        // Reset to default values
        reset();
    }

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
            if (fin && fin.good() && fin.is_open()) 
            {
                if ('/' != configPath.string()[0])
                {
                    char buff[PATH_MAX+1];
                    char *p = getcwd(buff, PATH_MAX);
                    if (p) {
                        _path_where_found = buff;
                        _path_where_found += '/';
                    }
                }
                _path_where_found += configPath.string();
                break;
            }
        }
    }

    // Whoops, failed.
    if (!fin || !fin.good() || !fin.is_open())
        throw IOException(TRACE_INFO,
             "unable to open file \"%s\"", filename);

    string line;
    string name;
    string value;
    unsigned int line_number = 0;
    bool have_name = false;
    bool have_value = false;

    while (++line_number, fin.good() && getline(fin, line))
    {
        string::size_type idx;

        // Find comment and discard the rest of the line.
        if ((idx = line.find('#')) != string::npos) {
            line.replace(idx, line.size() - idx, "");
        }

        // Search for the '=' character.
        if ((idx = line.find('=')) != string::npos)
        {
            // Select name and value
            name  = line.substr(0, idx);
            value = line.substr(idx + 1);

            // Strip out whitespace, etc. 
            name  = strip(name);
            value = strip(value);
            value = strip(value, "\"");

            have_name = true;

            // Look for trailing comma.
            if (',' != value[value.size()-1])
                have_value = true;
        }

        // Append more to value
        else if ((string::npos != line.find_first_not_of(blank_chars)) &&
            have_name &&
            !have_value)
        {
            value += strip(line);

            // Look for trailing comma.
            if (',' != value[value.size()-1])
                have_value = true;
        }
        
        else if (line.find_first_not_of(blank_chars) != string::npos)
        {
            throw InvalidParamException(TRACE_INFO,
                  "[ERROR] invalid configuration entry (line %d)", line_number);
        }

        if (have_name && have_value)
        {
            // Finally, store the entries.
            table[name] = value;
            have_name = false;
            have_value = false;
            value = "";
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
                                   "[ERROR] parameter not found (%s)",
                                   name.c_str());
    return it->second;
}

const string& Config::operator[](const string &name) const
{
    return get(name);
}

int Config::get_int(const string &name) const
{
    try {
        return boost::lexical_cast<int>(get(name));
    } catch(boost::bad_lexical_cast) {
        throw InvalidParamException(TRACE_INFO,
                                    "[ERROR] invalid integer parameter (%s)",
                                    name.c_str());
    }
}

long Config::get_long(const string &name) const
{
    try {
        return boost::lexical_cast<long>(get(name));
    } catch(boost::bad_lexical_cast) {
        throw InvalidParamException(TRACE_INFO,
                                    "[ERROR] invalid long integer parameter (%s)",
                                    name.c_str());
    }
}

double Config::get_double(const string &name) const
{
    try {
        return boost::lexical_cast<double>(get(name));
    } catch(boost::bad_lexical_cast) {
        throw InvalidParamException(TRACE_INFO,
                                    "[ERROR] invalid double parameter (%s)",
                                    name.c_str());
    }
}

bool Config::get_bool(const string &name) const
{
    if (boost::iequals(get(name), "true")) return true;
    else if (boost::iequals(get(name), "false")) return false;
    else throw InvalidParamException(TRACE_INFO,
                                     "[ERROR] invalid bool parameter (%s: %s)",
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
Config& opencog::config(ConfigFactory* factoryFunction,
                        bool overwrite)
{
    static std::unique_ptr<Config> instance((*factoryFunction)());
    if (overwrite)
        instance.reset((*factoryFunction)());
    return *instance;
}
