/*
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * Written by Joel Pitt <joel@fruitionnz.com>
 * All Rights Reserved
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

#include "HopfieldServer.h"

#include <fstream>
#include <iomanip>

using namespace opencog;

std::vector< Pattern > getPatterns();
void testHopfieldNetworkRolling();
void testHopfieldNetworkRollingOld();
void testHopfieldNetworkInterleave();

HopfieldServer hServer;
HopfieldOptions *o = hServer.options;

int main(int argc, char *argv[])
{
    //int patternArray[] = { 0, 1, 0, 1, 1, 1, 0, 1, 0 };
    //std::vector<int> pattern1(patternArray, patternArray + 9);
    o->parseOptions(argc, argv);

    if (o->showConfigFlag) {
        o->printConfiguration();
        exit(0);
    }

    /* setup logging */
    logger().setPrintToStdoutFlag(true);
    if (o->verboseFlag == 1) {
        logger().setLevel(Logger::DEBUG);
    } else if (o->verboseFlag == 2) {
        logger().setLevel(Logger::FINE);
    } else {
        logger().setLevel(Logger::WARN);
    }

    logger().info("Init HopfieldServer");
    hServer.init(-1, -1, -1);

    if (o->recordToFile) o->openOutputFiles();

    if (o->interleaveFlag)
        testHopfieldNetworkInterleave();
    else
        testHopfieldNetworkRolling();

    if (o->recordToFile) o->closeOutputFiles();
}

std::vector< Pattern > getPatterns()
{
    if (o->fileTraining.size() > 0) {
        std::vector< Pattern > ps;
        ps = Pattern::loadPatterns(o->fileTraining, hServer.height);
        o->nPatterns = ps.size();
        return ps;
    } else
        return Pattern::generateRandomPatterns(o->nPatterns, hServer.width, hServer.height, o->genPatternDensity);

}

std::vector< Pattern > getCuePatterns(std::vector< Pattern > ps)
{
    std::vector< Pattern > cs;
    if (ps.size() == 0) {
        cout << "Load original patterns before cue patterns!" << endl;
        return cs;
    }

    if (o->fileCue.size() > 0) {
        cs = Pattern::loadPatterns(o->fileCue, hServer.height);
        assert((unsigned int) o->nPatterns == cs.size());

    } else {
        for (std::vector< Pattern >::iterator i = ps.begin();
                i != ps.end(); i++) {
            cs.push_back( (*i).mutatePattern(o->cueErrorRate));
        }

    }
    return cs;
}

void testHopfieldNetworkInterleave()
{
# define HDEMO_NORESULT -100
    std::vector< Pattern > patterns;
    std::vector< Pattern > cuePatterns;
    std::vector< std::vector < float > > results;
    int totalCycles;

    patterns = getPatterns();
    cuePatterns = getCuePatterns(patterns);

    totalCycles = (o->interleaveAmount * (o->nPatterns - 1)) + o->imprintCycles;

    if (o->showMatrixFlag) {
        hServer.printMatrixResult(patterns);
    }

    for (int i = 0; i < totalCycles; i++) {
        int startPattern, endPattern;
        std::vector< float > cycleResults;
        std::vector< Pattern > toPrint;

        if (!o->verboseFlag) cout << "cycle " << i << " ,\t";

        // Find first pattern for imprinting
        if (o->interleaveAmount > 0) {
            startPattern = ( i / o->interleaveAmount ) - (o->imprintCycles);
            if (startPattern < 0) startPattern = 0;
        } else
            startPattern = 0;

        // Last pattern for imprinting
        if (o->interleaveAmount > 0) {
            endPattern = i / o->interleaveAmount;
            if ((unsigned int) endPattern >= patterns.size()) endPattern = patterns.size() - 1;
        } else
            endPattern = patterns.size() - 1;

        // Imprint each of them.
        for (unsigned int j = startPattern; j <= (unsigned int) endPattern; j++) {
            hServer.resetNodes();
            hServer.imprintPattern(patterns[j], 1);
            //if (!verboseFlag) cout << ".";
        }

        for (unsigned int j = 0; j < patterns.size(); j++) {
            float rSim;
            if (j <= (unsigned int) endPattern) {
                Pattern c(0,0);
                if (!o->cueGenerateOnce) {
                    if (o->fileCue.size() > 0) {
                        c = cuePatterns[j].mutatePattern(o->cueErrorRate);
                    } else {
                        c = patterns[j].mutatePattern(o->cueErrorRate);
                    }
                } 
                Pattern rPattern = hServer.retrievePattern(cuePatterns[j],
                        o->retrieveCycles, o->spreadCycles);
                rSim = patterns[j].hammingSimilarity(rPattern);
                cycleResults.push_back(rSim);
                cout << setw(4) << rSim << "(" << setw(5) << rSim - patterns[j].hammingSimilarity(cuePatterns[j]) << ")" << ", ";
                if (o->showMatrixFlag) {
                    toPrint.push_back(rPattern);
                }
            } else {
                cycleResults.push_back(HDEMO_NORESULT);
                cout << setw(11) << "-" << ", ";
            }

        }
        if (!o->verboseFlag) cout << endl;
        if (o->showMatrixFlag) hServer.printMatrixResult(toPrint);
        results.push_back(cycleResults);

    }


}

void testHopfieldNetworkRolling()
{
    std::vector< Pattern > patterns;
    std::vector< Pattern > cuePatterns;
    std::vector< std::vector<float> > results;
    std::vector<float> diffs;

    patterns = getPatterns();
    cuePatterns = getCuePatterns(patterns);

    for (unsigned int i = 0; i < patterns.size(); i++) {
        if (o->resetFlag)
            hServer.reset();
        results.push_back(hServer.imprintAndTestPattern(patterns[i], o->imprintCycles, o->retrieveCycles, cuePatterns[i], o->cueErrorRate));
        if (!o->verboseFlag) cout << " - pattern " << i << " done" << endl;
        if (o->recordToFile) {
            o->beforeFile << endl;
            o->afterFile << endl;
            o->diffFile << endl;
        }
    }
    for (unsigned int i = 0; i < patterns.size(); i++) {
        std::vector< float > r = results[i];
        // Add final performance to diffs vector
        diffs.push_back(*(r.rbegin()));

    }

    if (o->totalFlag) {
        // output mean performace
        float total = 0.0f;
        for (unsigned int i = 0; i < diffs.size(); i++)
            total += diffs[i];
        cout << total / diffs.size() << endl;

    }

}

