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

        // Create a new index given a SCM file
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        int indexKey = PatternIndexAPI::getInstance().createNewIndex(argv[1]);
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        if (indexKey == -1) {
            throw std::runtime_error("Error creating pattern index\n");
        }

        //
        // Optional setup of parameters which are relevant to PatternIndexAPI::minePatterns()
        //
        // PatternIndexAPI have sensible defaults but some tuning may
        // be required to adjust quality X memory usage X time performance.
        //
        // All parameters are described in PatternIndexAPI::setDefaultProperties(). 
        //
        PatternIndexAPI::getInstance().setProperty(indexKey, "PatternCountCacheEnabled", "false");
        PatternIndexAPI::getInstance().setProperty(indexKey, "NumberOfEvaluationThreads", "4");
        PatternIndexAPI::getInstance().setProperty(indexKey, "MinimalFrequencyToComputeQualityMetric", "5");
        PatternIndexAPI::getInstance().setProperty(indexKey, "MaximumSizeOfCompoundFramesQueue", "5000000");
        PatternIndexAPI::getInstance().setProperty(indexKey, "CoherenceFunction", "const1");
        PatternIndexAPI::getInstance().setProperty(indexKey, "CoherenceModulatorG", "oneOverCoherence");
        PatternIndexAPI::getInstance().setProperty(indexKey, "CoherenceModulatorH", "coherence");
        PatternIndexAPI::getInstance().setProperty(indexKey, "PatternsGram", "3");
        PatternIndexAPI::getInstance().setProperty(indexKey, "MaximumNumberOfMiningResults", "10");
        PatternIndexAPI::getInstance().setProperty(indexKey, "PatternRankingMetric", "nisurprisingness");
        
        // Pattern mining
        std::vector<PatternIndexAPI::MiningResult> resultPatterns;
        std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
        PatternIndexAPI::getInstance().minePatterns(resultPatterns, indexKey);
        std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();

        printf("Top %lu results\n", resultPatterns.size());
        for (unsigned int i = 0; i < resultPatterns.size(); i++) {
            printf("%f: %s", resultPatterns.at(i).first, resultPatterns.at(i).second->toString().c_str());
        }
        unsigned int delta1 = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
        unsigned int delta2 = std::chrono::duration_cast<std::chrono::seconds>(t4 - t3).count();
        printf("Time to build index: %u seconds\n", delta1);
        printf("Time to mine patterns: %u seconds\n", delta2);
    }

    return exitValue;
}
