/*
 * LojbanModule.cc
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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

#include "LojbanModule.h"

#include <fstream>
#include <sys/stat.h>


using namespace opencog::nlp;
using namespace opencog;

DECLARE_MODULE(LojbanModule);

/**
 * The constructor for LojbanModule.
 *
 * @param cs   the OpenCog server
 */
LojbanModule::LojbanModule(CogServer& cs) :
        Module(cs) , _cs(cs) , _as(&cs.getAtomSpace()) , scmeval(_as)
{
    _wordlist = lojban_init();
}

/**
 * The destructor for LojbanModule.
 */
LojbanModule::~LojbanModule()
{
    lojban_exit(_wordlist);
    do_load_lojban_unregister();
    do_parse_lojban_unregister();
}

/**
 * The required implementation for the pure virtual init method.
 */
void LojbanModule::init(void)
{
    do_load_lojban_register();
    do_parse_lojban_register();
}


std::string LojbanModule::do_parse_lojban(Request *req, std::list<std::string> args)
{
    std::string sentence;
    for (const auto& a: args) sentence += a + " ";

    Handle * hptr = lojban_parse(_as,_wordlist,sentence.c_str());

    if (!hptr or (*hptr) == Handle::UNDEFINED)
        return "Parsing Failed";
    else
        return (*hptr)->to_string();
}

std::string LojbanModule::do_load_lojban(Request *req, std::list<std::string> args)
{
    std::cout << "------------------------------------------";
    std::string path = args.front();
    std::cout << path;

    std::ifstream file(path);
    std::string line;

    while (std::getline(file, line))
    {
        std::cout << "------------------------------------------";
        std::cout << line;
        std::cout << "------------------------------------------";
        lojban_parse(_as,_wordlist,line.c_str());
    }

    return "";
}
