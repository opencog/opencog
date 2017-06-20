/*
 * patternIndexExample.cc
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

#include <chrono>

#include <opencog/guile/SchemeEval.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Handle.h>

#include "PatternIndexAPI.h"
#include <opencog/guile/SchemeEval.h>

using namespace opencog;

int main(int argc, char *argv[]) {

    int exitValue = 0;

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <SCM file>\n", argv[0]);
        exitValue = 1;
    } else {

        // AtomSpace setup
        AtomSpace atomSpace;
        SchemeEval::init_scheme();
        SchemeEval::set_scheme_as(&atomSpace);
        SchemeEval *schemeEval = new SchemeEval(&atomSpace);

        // Create a new index given a SCM file
        //int indexKey = PatternIndexAPI::getInstance().createNewIndex(argv[1]);
        Handle indexKey = PatternIndexAPI::getInstance().createNewIndex_h(argv[1]);

        // Optional setup of parameters which are relevant to PatternIndexAPI::query()
        //
        // Sensible defaults are provided in PatternIndexAPI::setDefaultProperties() but
        // some tuning may be required to adjust quality X memory usage X time
        // performance.
        //
        // All parameters are described in PatternIndexAPI::setDefaultProperties(). 
        PatternIndexAPI::getInstance().setProperty(indexKey, "PatternCountCacheEnabled", "true");
        PatternIndexAPI::getInstance().setProperty(indexKey, "DifferentAssignmentForDifferentVars", "true");
        PatternIndexAPI::getInstance().setProperty(indexKey, "PermutationsOfVarsConsideredEquivalent", "true");
        
        // Query
        std::string queryStr = "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")))";
        Handle queryHandle = schemeEval->eval_h(queryStr);
        std::vector<PatternIndexAPI::QueryResult> queryResult;
        //PatternIndexAPI::getInstance().query(queryResult, indexKey, queryStr);
        Handle resultHandle = PatternIndexAPI::getInstance().query(indexKey, queryHandle);

        /*
        printf("Query: %s\n", queryStr.c_str());
        printf("%lu results\n", queryResult.size());
        for (unsigned int i = 0; i < queryResult.size(); i++) {
            printf("Result #%u:\n\n", i + 1);
            for (unsigned int j = 0; j < queryResult.at(i).first.size(); j++) {
                printf("%s\n", queryResult.at(i).first.at(j)->toString().c_str());
            }
            printf("Mapping:\n\n");
            for (unsigned int j = 0; j < queryResult.at(i).second.size(); j++) {
                printf("%s%s\n", queryResult.at(i).second.at(j).first->toString().c_str(), queryResult.at(i).second.at(j).second->toString().c_str());
            }
        }
        */
        printf("Result: %s", resultHandle->toString().c_str());

    }

    return exitValue;
}
