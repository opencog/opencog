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

#include <boost/algorithm/string/join.hpp>

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
    const char * name = "lojban.xml";
    std::string url = "https://raw.githubusercontent.com/rTreutlein"
                      "/lojban-data/master/lojban.xml";
    struct stat buffer;
    if (stat (name,&buffer) != 0) {
        int i = system(("wget " + url).c_str());
        std::cout << i << "\n";
    }

    _wordlist = lojban_init(name);

    scmeval.eval("(use-modules (opencog) (opencog nlp))");
    scmeval.eval("(use-modules (opencog nlp chatbot))");
    scmeval.eval("(use-modules (opencog) (opencog nlp relex2logic))");
    scmeval.eval("(use-modules (opencog) (opencog nlp fuzzy))");
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
    std::string sentence = boost::algorithm::join(args," ");

    Handle * hptr = lojban_parse(_as,_wordlist,sentence.c_str());

    if (!hptr or (*hptr) == Handle::UNDEFINED)
        return "Parsing Failed";
    else
        return h_to_string((*hptr));
}

std::string LojbanModule::do_load_lojban(Request *req, std::list<std::string> args)
{
    std::string path = args.front();

    std::ifstream file(path);
    std::string line;

    std::string english;
    std::string lojban;


    while (std::getline(file, line))
    {
        std::size_t pos = line.find(",");
        lojban = line.substr(0,pos);
        english  = line.substr(pos+1,std::string::npos);

        Handle answer = scmeval.eval_h("(car (nlp-parse \"" + english + "\"))");

        Handle * hptr = lojban_parse(_as,_wordlist,lojban.c_str());

        if (!hptr or (*hptr) == Handle::UNDEFINED)
            continue;
        else
            _as->add_link(EQUIVALENCE_LINK,answer,(*hptr));
    }

    return "";
}
