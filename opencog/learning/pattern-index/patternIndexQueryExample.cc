/*
 * patternIndexExample.cc
 *
 * Copyright (C) 2017 OpenCog Foundation
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

#include <chrono>

#include <opencog/guile/SchemeEval.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/util/Config.h>

#include "PatternIndexAPI.h"

using namespace opencog;

int main(int argc, char *argv[]) {

    int exitValue = 0;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <SCM file> <config file>\n", argv[0]);
        exitValue = 1;
    } else {

        // Load configuration file
        config().load(argv[2]);

        // AtomSpace setup
        AtomSpace atomSpace;
        SchemeEval::init_scheme();
        SchemeEval::set_scheme_as(&atomSpace);
        SchemeEval *schemeEval = new SchemeEval(&atomSpace);

        // Create a new index given a SCM file
        Handle indexKey = patternindex().createIndex(argv[1]);

        // Optional setup of parameters which are relevant to
        // PatternIndexAPI::query()
        //
        // Sensible defaults are provided in
        // PatternIndexAPI::setDefaultProperties() but some tuning may be
        // required to adjust quality X memory usage X time performance.
        //
        // All parameters are described in
        // PatternIndexAPI::setDefaultProperties(). 
        patternindex().setProperty(indexKey,
                                   "PatternCountCacheEnabled",
                                   "true");
        patternindex().setProperty(indexKey,
                                   "DifferentAssignmentForDifferentVars",
                                   "true");
        patternindex().setProperty(indexKey,
                                   "PermutationsOfVarsConsideredEquivalent",
                                   "true");
        
        // Query
        std::string queryStr1 = "(AndLink\
                                   (SimilarityLink\
                                     (VariableNode \"X\")\
                                     (VariableNode \"Y\")\
                                   )\
                                   (SimilarityLink\
                                     (VariableNode \"Y\")\
                                     (VariableNode \"Z\")\
                                   )\
                                 )";
        Handle queryHandle = schemeEval->eval_h(queryStr1);
        Handle resultHandle = patternindex().query(indexKey, queryHandle);
        printf("Query 1:\n");
        printf("Result: %s", resultHandle->to_string().c_str());

        //
        // optionally, query() may be called passing the SCM string
        //

        std::string queryStr2 = "(AndLink\
                                   (SimilarityLink\
                                     (VariableNode \"X\")\
                                     (VariableNode \"Y\")\
                                   )\
                                   (NotLink\
                                     (AndLink\
                                       (InheritanceLink\
                                         (VariableNode \"X\")\
                                         (VariableNode \"Z\")\
                                       )\
                                       (InheritanceLink\
                                         (VariableNode \"Y\")\
                                         (VariableNode \"Z\")\
                                       )\
                                     )\
                                   )\
                                 )";
        std::vector<PatternIndexAPI::QueryResult> queryResults;
        patternindex().query(queryResults, indexKey, queryStr2);
        printf("\n\nQuery 2:\n");
        printf("%zu results\n", queryResults.size());
        for (unsigned int i = 0; i < queryResults.size(); i++) {
            printf("Result #%u:\n\n", i + 1);
            for (const Handle& result : queryResults.at(i).first) {
                printf("%s\n", result->to_string().c_str());
            }
            printf("Mapping:\n\n");
            for (const auto& vargnd : queryResults.at(i).second) {
                printf("%s%s\n", 
                       vargnd.first->to_string().c_str(),
                       vargnd.second->to_string().c_str());
            }
        }
    }

    return exitValue;
}
