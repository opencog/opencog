/*
 * PGStorageDelegate.cc
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

#include "PGStorageDelegate.h"
#include "SCMLoader.h"
#include <opencog/guile/SchemeEval.h>

using namespace opencog;

PGStorageDelegate::PGStorageDelegate(const char *dbName, const char *userName, const char *password)
{
    _dbName = dbName;
    _userName = userName;
    _password = password;
}

PGStorageDelegate::~PGStorageDelegate()
{
}

bool PGStorageDelegate::loadSCMFile(const char *fileName)
{
    bool exitValue = true;

    SchemeEval::init_scheme();
    AtomSpace atomSpace;
    SchemeEval::set_scheme_as(&atomSpace);
    SchemeEval *schemeEval = new SchemeEval(&atomSpace);
    schemeEval->eval("(add-to-load-path \"/usr/local/share/opencog/scm\")");
    schemeEval->eval("(add-to-load-path \"/opencog/build/opencog/scm/opencog/\")");
    schemeEval->eval("(add-to-load-path \".\")");
    schemeEval->eval("(use-modules (ice-9 readline))");
    schemeEval->eval("(activate-readline)");
    schemeEval->eval("(use-modules (opencog))");
    schemeEval->eval("(use-modules (opencog nlp) (opencog nlp lg-dict) (opencog nlp relex2logic) (opencog nlp chatbot) (opencog persist-pgsql))");
    schemeEval->eval("(use-modules (opencog persist-pgsql))");

    exitValue = SCMLoader::load(fileName, atomSpace);

    if (! exitValue) {
        logger().info("Storing AtomSpace into DB...");
        std::string commandLine = "(pgsql-open \"" + _dbName + "\" \"" + _userName + "\" \"" + _password + "\")";
        std::string answer;
        answer = schemeEval->eval(commandLine);
        printf("%s\n", answer.c_str());
        answer = schemeEval->eval("(pgsql-store)");
        printf("%s\n", answer.c_str());
        answer = schemeEval->eval("(pgsql-close)");
        printf("%s\n", answer.c_str());
        logger().info("Finished storing AtomSpace into DB.");
    }

    return exitValue;
}
