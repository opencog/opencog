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

#include <fstream>
#include <iomanip>
#include <iostream>

#include <unistd.h>
#include <math.h>
#include <boost/foreach.hpp>

#include <opencog/util/Config.h>

#include "HopfieldServer.h"
#include "HopfieldOptions.h"

using namespace opencog;
using namespace std;
#define foreach BOOST_FOREACH
#define HDEMO_NORESULT -100

std::vector< Pattern > getPatterns();
void testHopfieldNetworkRolling();
void testHopfieldNetworkRollingOld();
void testHopfieldNetworkInterleave();
void testHopfieldNetworkPalimpsest();
void testHopfieldNetworkPalimpsestNeighbours();

HopfieldOptions *o;

int main(int argc, char *argv[])
{
    server(HopfieldServer::derivedCreateInstance);

    o = (static_cast<HopfieldServer&>(server())).options;

    o->parseOptions(argc, argv);

    (static_cast<HopfieldServer&>(server())).init(-1, -1, -1);

    /* setup logging */
    logger().set_print_to_stdout_flag(config().get_bool("LOG_TO_STDOUT"));
    switch (o->verboseLevel) {
    case 1:
        logger().set_level(Logger::INFO);
        logger().set_print_to_stdout_flag(true);
        break;
    case 2:
        logger().set_level(Logger::DEBUG);
        logger().set_print_to_stdout_flag(true);
        break;
    case 3:
        logger().set_level(Logger::FINE);
        logger().set_print_to_stdout_flag(true);
        break;
    default:
        logger().set_level(Logger::WARN);
    }

    if (o->showConfigFlag) {
        o->printConfiguration();
        exit(0);
    }

    if (o->recordToFile) o->openOutputFiles();

    switch (o->learningScheme) {
    case HopfieldOptions::INTERLEAVE:
        testHopfieldNetworkInterleave();
        break;
    case HopfieldOptions::SEQUENCE:
        testHopfieldNetworkRolling();
        break;
    case HopfieldOptions::PALIMPSEST:
        testHopfieldNetworkPalimpsest();
        break;
    case HopfieldOptions::PALIMPSEST_NEIGHBOURS:
        testHopfieldNetworkPalimpsestNeighbours();
        break;
    }

    if (o->recordToFile) o->closeOutputFiles();

    return 0;
}

std::vector< Pattern > getPatterns()
{
    if (o->fileTraining.size() > 0) {
        std::vector< Pattern > ps;
        ps = Pattern::loadPatterns(o->fileTraining, (static_cast<HopfieldServer&>(server())).height);
        o->nPatterns = ps.size();
        return ps;
    } else
        return Pattern::generateRandomPatterns(o->nPatterns, (static_cast<HopfieldServer&>(server())).width, (static_cast<HopfieldServer&>(server())).height, o->genPatternDensity);

}

std::vector< Pattern > getCuePatterns(std::vector< Pattern > ps)
{
    std::vector< Pattern > cs;
    if (ps.size() == 0) {
        cerr << "Load original patterns before cue patterns!" << endl;
        return cs;
    }

    if (o->fileCue.size() > 0) {
        // If there is a file to load patterns from...
        cs = Pattern::loadPatterns(o->fileCue, (static_cast<HopfieldServer&>(server())).height);
        assert((unsigned int) o->nPatterns == cs.size());
    } else {
        // otherwise, just copy imprint patterns.
        for (std::vector< Pattern >::iterator i = ps.begin();
                i != ps.end(); ++i) {
            // Cue patterns shouldn't be mutated unless cueGenerateOnce is set.
            cs.push_back( (*i) );
        }
    }
    // If cueGenerateOnce is set, then mutate all patterns
    // (they won't be mutated later)
    if (o->cueGenerateOnce) {
        for (std::vector< Pattern >::iterator i = cs.begin();
            i != cs.end(); ++i) {
            (*i) = (*i).mutatePattern(o->cueErrorRate);
        }
    }
    return cs;
}

void testHopfieldNetworkPalimpsestNeighbours()
{
    std::vector< Pattern > patterns;
    std::vector< Pattern > cuePatterns;
    std::vector< std::vector < float > > results;

    int totalCycles;

    patterns = getPatterns();
    totalCycles = patterns.size();

    for (int i = 0; i < totalCycles; i++) {
        int memory = 0;
        std::vector< float > cycleResults;
        std::vector< int > bitErrors;
        std::vector< Pattern > toPrint;

        if (o->showMatrixFlag) {
            std::vector< Pattern > toPrint;
            cout << endl;
            toPrint.push_back(patterns[i]);
            (static_cast<HopfieldServer&>(server())).printMatrixResult(toPrint);
        }
        (static_cast<HopfieldServer&>(server())).resetNodes();
        (static_cast<HopfieldServer&>(server())).imprintPattern(patterns[i], o->imprintCycles);

        // Go back until patterns are not recalled withing palimpsest tolerance
        // level...
        for (int j = i; j >= 0; j--) {
            float rSim = 0.0f;
            // palimpsest learning measured by checking
            // the stability of neighbouring states (1 bit errors).
            std::vector<bool> stableBit =
                (static_cast<HopfieldServer&>(server())).checkNeighbourStability(
                        patterns[j], o->palimpsestTolerance / 100.0f);

            foreach (bool b, stableBit) {
            //    cout << b << " ";
                if (b) rSim += 1;
            }
            //cout << endl;
            rSim /= stableBit.size();
            //cout << "rSim = " << rSim << endl;

            //logger().debug(" Similarity %.2f ( diff: %.2f )", rSim, rSim - patterns[j].hammingSimilarity(cuePatterns[j]));

            if ( rSim < 1) {
                break;
            }
            memory++;

        }
        cout  << ", " << memory << flush;
        results.push_back(cycleResults);

    }
    cout << endl;


}
void testHopfieldNetworkPalimpsest()
{
    std::vector< Pattern > patterns;
    std::vector< Pattern > cuePatterns;
    std::vector< std::vector < float > > results;

    int totalCycles;

    patterns = getPatterns();
    cuePatterns = getCuePatterns(patterns);

    totalCycles = patterns.size();

    for (int i = 0; i < totalCycles; i++) {
        int memory = 0;
        std::vector< float > cycleResults;
        std::vector< int > bitErrors;
        std::vector< Pattern > toPrint;

        if (o->showMatrixFlag) {
            std::vector< Pattern > toPrint;
            cout << endl;
            toPrint.push_back(patterns[i]);
            (static_cast<HopfieldServer&>(server())).printMatrixResult(toPrint);
        }
        (static_cast<HopfieldServer&>(server())).resetNodes();
        (static_cast<HopfieldServer&>(server())).imprintPattern(patterns[i], o->imprintCycles);

        // Go back until patterns are not recalled withing palimpsest tolerance
        // level...
        for (int j = i; j >= 0; j--) {
            float rSim = 0.0f;
            Pattern c(0,0);
            if (!o->cueGenerateOnce) {
                if (o->fileCue.size() > 0) {
                    c = cuePatterns[j].mutatePattern(o->cueErrorRate);
                } else {
                    c = patterns[j].mutatePattern(o->cueErrorRate);
                }
            } else {
                c = cuePatterns[j];
            }
            Pattern rPattern = (static_cast<HopfieldServer&>(server())).retrievePattern(c,
                    o->retrieveCycles, o->spreadCycles);
            if (o->showMatrixFlag) {
                std::vector< Pattern > toPrint;
                toPrint.push_back(patterns[j]);
                toPrint.push_back(c);
                toPrint.push_back(rPattern);
                cout << "original | cue | retrieved" << endl;
                (static_cast<HopfieldServer&>(server())).printMatrixResult(toPrint);
                cout << "---" << endl;
            }
            rSim = patterns[j].hammingSimilarity(rPattern);
            cycleResults.push_back(rSim);

            logger().debug(" Similarity %.2f ( diff: %.2f )", rSim, rSim - patterns[j].hammingSimilarity(cuePatterns[j]));

            if ( (rSim * 100) < (100 - o->palimpsestTolerance) ) break;
            memory++;

        }
        cout  << ", " << memory << flush;
        if (o->showMatrixFlag)
            (static_cast<HopfieldServer&>(server())).printMatrixResult(toPrint);
        results.push_back(cycleResults);

    }
    cout << endl;


}

void testHopfieldNetworkInterleave()
{
    std::vector< Pattern > patterns;
    std::vector< Pattern > cuePatterns;
    std::vector< std::vector < float > > results;
    int totalCycles;

    patterns = getPatterns();
    cuePatterns = getCuePatterns(patterns);

    totalCycles = (o->interleaveAmount * (o->nPatterns - 1)) + o->imprintCycles;

    if (o->showMatrixFlag) {
        (static_cast<HopfieldServer&>(server())).printMatrixResult(patterns);
    }

    for (int i = 0; i < totalCycles; i++) {
        int startPattern, endPattern;
        std::vector< float > cycleResults;
        std::vector< Pattern > toPrint;

        if (!o->verboseLevel) cout << "cycle " << i << " ,\t";

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
            (static_cast<HopfieldServer&>(server())).resetNodes();
            (static_cast<HopfieldServer&>(server())).imprintPattern(patterns[j], 1);
            //if (!verboseLevel) cout << ".";
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
                } else {
                    c = cuePatterns[j];
                }
                Pattern rPattern = (static_cast<HopfieldServer&>(server())).retrievePattern(c,
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
        if (!o->verboseLevel) cout << endl;
        if (o->showMatrixFlag) (static_cast<HopfieldServer&>(server())).printMatrixResult(toPrint);
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

#ifdef HAVE_UBIGRAPH
    if (o->visualize) sleep(10 * o->visDelay);
#endif //HAVE_UBIGRAPH
    for (unsigned int i = 0; i < patterns.size(); i++) {
        if (o->resetFlag)
            (static_cast<HopfieldServer&>(server())).reset();
        results.push_back((static_cast<HopfieldServer&>(server())).imprintAndTestPattern(patterns[i], o->imprintCycles, o->retrieveCycles, cuePatterns[i], o->cueErrorRate));
        if (!o->verboseLevel) cout << " - pattern " << i << " done" << endl;
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
