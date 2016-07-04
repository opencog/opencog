/*
 * loadscm.cc
 *
 * Copyright (C) 2016 OpenCog Foundation
 *
 * Author: Andre Senna <https://github.com/andre-senna>
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

#include "SCMLoader.h"
#include <opencog/guile/SchemeEval.h>

using namespace opencog;

bool loadFile(char *fileName)
{
    SchemeEval::init_scheme();
    AtomSpace atomSpace;
    SchemeEval::set_scheme_as(&atomSpace);
    SchemeEval *schemeEval = new SchemeEval(&atomSpace);
    std::string output;

    schemeEval->eval("(add-to-load-path \"/usr/local/share/opencog/scm\")");
    schemeEval->eval("(add-to-load-path \"/opencog/build/opencog/scm/opencog/\")");
    schemeEval->eval("(add-to-load-path \".\")");
    schemeEval->eval("(use-modules (ice-9 readline))");
    schemeEval->eval("(activate-readline)");
    schemeEval->eval("(use-modules (opencog))");
    schemeEval->eval("(use-modules (opencog nlp) (opencog nlp lg-dict) (opencog nlp relex2logic) (opencog nlp chatbot))");

    bool exitValue = SCMLoader::load(fileName, atomSpace);

    output = schemeEval->eval("(count-all)");
    printf("%s\n", output.c_str());

    //output = schemeEval->eval("(count-all)");
    //printf("%s\n", output.c_str());
    //output = schemeEval->eval("(EvaluationLink (PredicateNode \"predicate\") (ConceptNode \"concept\"))");
    //printf("%s\n", output.c_str());
    //output = schemeEval->eval("(count-all)");
    //printf("%s\n", output.c_str());


    return exitValue;
}

int main(int argc, char *argv[]) {

    int exitValue = 0;

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <SCM file>\n", argv[0]);
        exitValue = 1;
    } else {
        if (! loadFile(argv[1])) {
            exitValue = 1;
        }
    }

    return exitValue;
}

