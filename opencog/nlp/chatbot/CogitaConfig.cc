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

#include "CogitaConfig.h"

#include <getopt.h>

#include <string>
#include <vector>
#include <iostream>
#include <stdlib.h>

#include <opencog/util/StringTokenizer.h>

using std::cout;
using std::string;

using namespace std;

namespace opencog {
namespace chatbot {

CogitaConfig::CogitaConfig() :
    ircNetwork(COGITA_DEFAULT_SERVER), ircPort(COGITA_DEFAULT_PORT),
    vstring(COGITA_VSTRING), nick(COGITA_DEFAULT_NICK)
{
        const char* defaultAttns[] = COGITA_DEFAULT_ATTN;
        const char* defaultSuffixes[] = COGITA_DEFAULT_ATTN_SUFFIXES;
        const char* defaultChannels[] = COGITA_DEFAULT_CHANNELS;
        for (int i = 0; defaultAttns[i]; i++) {
            for (int i = 0; defaultSuffixes[i]; i++) {
                attn.push_back(string(defaultAttns[i]) +
                        string(defaultSuffixes[i]));
            }
        }
        for (int i = 0; defaultChannels[i]; i++) {
            ircChannels.push_back(std::string(defaultChannels[i]));
        }

}

const std::string CogitaConfig::helpOutput = 
    " Cogita - An OpenCog chatbot. \n"
    " ======\n"
    " Usage: \n"
    " -n,--nick \tSet bot nick.\n"
    " -s,--server \tIRC server to connect to.\n"
    " -p,--port \tPort of IRC server to connect to.\n"
    " -c,--channels \tComma separated list of channels (without preceding #) to join\n"
    "               \t(only single channel support implemented).\n"
    " -v,--version \tPrint version information.\n"
    " \n";

void CogitaConfig::printHelp() { cout << helpOutput; }

void CogitaConfig::printVersion() { cout << COGITA_VSTRING << endl; }

int CogitaConfig::parseOptions(int argc, char* argv[])
{
    int c = 0;
    static const char *optString =
        "n:s:p:c:vh";

    static const struct option longOptions[] = {
        {"nick", required_argument, 0, 'n'},
        {"server", required_argument, 0, 's'},
        {"port", required_argument, 0, 's'},
        {"channels", required_argument, 0, 'c'},
        {"version", 0, 0, 'v'},
        {"help", 0, 0, '?'},
        {0, 0, 0, 0}
    };

    while (1) {
        int optionIndex = 0;
        string channelsTemp;
        StringTokenizer st;
        c = getopt_long (argc, argv, optString, longOptions, &optionIndex);

        /* Detect end of options */
        if (c == -1)
            break;

        switch (c) {
        case 'n':
            nick = string(optarg);
            createAttnVector();
            break;
        case 's':
            ircNetwork = string(optarg);
            break;
        case 'p':
            ircPort = atoi(optarg);
            break;
        case 'c':
            ircChannels.clear();
            channelsTemp = optarg;
            st.setString(channelsTemp);
            st.setDelimiter(string(","));
            for (string channel = st.nextToken();
                    channel.size() > 0;
                    channel = st.nextToken()) {
                ircChannels.push_back("#" + channel);
            }
            break;
        case 'v':
            printVersion();
            return 1;
        case 'h':
            printHelp();
            return 1;
        default:
            printHelp();
            return 1;
        }
    }

    return 0;
}

void CogitaConfig::createAttnVector()
{
    const char* defaultSuffixes[] = COGITA_DEFAULT_ATTN_SUFFIXES;
    attn.clear();
    for (int i = 0; defaultSuffixes[i]; i++) {
        attn.push_back(nick + string(defaultSuffixes[i]));
    }

}

}} // ~namespace opencog::chatbot

