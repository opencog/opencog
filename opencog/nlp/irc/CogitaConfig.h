/*
 *   Config class for La Cogita IRC chatbot
 *   Copyright (C) 2009 Joel Pitt <joel@fruitionnz.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _OPENCOG_COGITA_CONFIG_H
#define _OPENCOG_COGITA_CONFIG_H

#include <string>
#include <vector>


namespace opencog {
namespace chatbot {

/**
 * Configuration class for Cogita
 */
class CogitaConfig
{
    void createAttnVector();
    static const char* helpOutput;
public:
    std::string version_string;

    std::string ircNetwork;
    std::vector<std::string> ircChannels;
    int ircPort;
    std::string irc_nick;
    std::string irc_name;
    std::string irc_pass;
    std::vector<std::string> attn;
    bool dry_run;

    std::string cog_addr; // OpenCog cogserver IP address.
    int cog_port;         // OpenCog cogserver port number.

    CogitaConfig();

    void printHelp();
    void printVersion();
    int parseOptions(int argc, char* argv[]);
};

}} // ~namespace opencog::chatbot

#endif // _OPENCOG_COGITA_CONFIG_H
