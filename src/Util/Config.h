/*
 * ./src/Util/Config.h
 *
 * Copyright (c) 2008 OpenCog.org
 */

#ifndef UTIL_CONFIG_H
#define UTIL_CONFIG_H

#include <string>
#include <map>

namespace opencog {

static const std::string DEFAULT_CONFIG[] = {
    "SERVER_PORT",                "17001",
    "LOG_FILE",                   "opencog_server.log",
    "LOG_LEVEL",                  "info",
    "LOG_TO_STDOUT",              "true",
    "SERVER_CYCLE_DURATION",      "100",     // in milliseconds
    "IDLE_CYCLES_PER_TICK",       "3",
    "STARTING_STI_FUNDS",         "1000",
    "STARTING_LTI_FUNDS",         "1000",
    "STI_FUNDS_BUFFER",           "200",
    "LTI_FUNDS_BUFFER",           "200",
    "MIN_STI",                    "-400",
    "",                           ""
};

class Config {

protected:

    std::string emptyString;
    std::map<std::string, std::string> table;

public:
    // constructor and destructor
    ~Config();
    Config();

    // reset configuration to default
    void reset();

    // Load passed file and redefines values for parameters.
    void load(const char* config_file);
          
    // Return current value of a given parameter.
    const std::string& get(const std::string &parameter_name) const;
    const std::string& operator[](const std::string &name) const;

    // Return current value of a given parameter as an integer
    int get_int(const std::string &parameter_name) const;

    // Return current value of a given parameter as a double
    double get_double(const std::string &parameter_name) const;
    //
    // Return current value of a given parameter as a boolean
    bool get_bool(const std::string &parameter_name) const;

    // Dump all configuration parameters to a string
    std::string to_string() const;
};

// singleton instance (following meyer's design pattern)
Config& config();

} // namespace opencog

#endif
