/*
 * opencog/util/Config.h
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

#ifndef _OPENCOG_CONFIG_H
#define _OPENCOG_CONFIG_H

#include <string>
#include <map>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

class Config;

typedef Config* ConfigFactory(void);

//! library-wide configuration; keys and values are strings
class Config
{

protected:

    const std::string* DEFAULT();

    std::string emptyString;
    std::map<std::string, std::string> table;
    std::string _path_where_found;

public:
    //! constructor
    ~Config();
    //! destructor
    Config();

    //! Returns a new Config instance
    static Config* createInstance(void);

    //! reset configuration to default
    virtual void reset();

    //! Load passed file and redefines values for parameters.
    void load(const char* config_file, bool resetFirst = true);

    //! location of the file
    const std::string& path_where_found() { return _path_where_found; }

    //! Checks whether a parameter exists
    const bool has(const std::string &parameter_name) const;

    //! Set the value of a given parameter.
    void set(const std::string &parameter_name, const std::string &parameter_value);

    //! Return current value of a given parameter.
    const std::string& get(const std::string &parameter_name) const;
    //! Return current value of a given parameter.
    const std::string& operator[](const std::string &name) const;

    //! Return current value of a given parameter as an integer
    int get_int(const std::string &parameter_name) const;

    //! Return current value of a given parameter as an long
    long get_long(const std::string &parameter_name) const;

    //! Return current value of a given parameter as a double
    double get_double(const std::string &parameter_name) const;

    //! Return current value of a given parameter as a boolean
    bool get_bool(const std::string &parameter_name) const;

    //! Dump all configuration parameters to a string
    std::string to_string() const;
};

//! singleton instance (following meyer's design pattern)
/**
 * Nil: if overwrite is true then the static variable instance@n
 *      is changed with the createInstance provided@n
 *      it is a temporary dirty hack@n
 */
Config& config(ConfigFactory* = Config::createInstance,
               bool overwrite = false);

/** @}*/
} // namespace opencog

#endif // _OPENCOG_CONFIG_H
