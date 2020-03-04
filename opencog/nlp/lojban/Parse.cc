/*
 * Parser.cc
 *
 * Copyright (C) 2020 SingularityNET Foundation
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <fstream>
#include <string>


#include <opencog/atoms/base/Handle.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/nlp/types/atom_types.h>

#include "LojbanParser.h"

using namespace std;
using namespace opencog;

int main(int argc, char *argv[])
{
    if(argc != 2) {
        printf("Usage: %s \"lojban sentence\"\n", argv[0]);
        return 1;
    }

    AtomSpace* as = new AtomSpace();
    SchemeEval* eval = new SchemeEval(as);
    eval->eval("(use-modules (opencog))");
    eval->eval("(use-modules (opencog nlp))");
    HsStablePtr wordlist = lojban_init();

    Handle* hptr = lojban_parse(as, wordlist, argv[1]);
    if (!hptr or (*hptr) == nullptr) {
        printf("Parsing Failed on \"%s\" \n", argv[1]);
        return 1;
    } else {
        printf("%s", (*hptr)->to_short_string().c_str());
        return 0;
    }
}
