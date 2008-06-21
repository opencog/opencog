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
#include "HopfieldOptions.h"

#include <iostream>
#include <getopt.h>

using namespace std;
using namespace opencog;

HopfieldOptions::HopfieldOptions()
{
    // Set defaults

    verboseFlag = HDEMO_DEFAULT_VERBOSE;
    interleaveFlag = HDEMO_DEFAULT_INTERLEAVE;
    interleaveAmount = HDEMO_DEFAULT_INTERLEAVEAMOUNT;
    showMatrixFlag = HDEMO_DEFAULT_SHOW_MATRIX;
    totalFlag = HDEMO_DEFAULT_SHOW_TOTAL;
    nPatterns = HDEMO_DEFAULT_NPATTERNS;
    genPatternDensity = HDEMO_DEFAULT_PATTERN_DENSITY;
    retrieveCycles = HDEMO_DEFAULT_RETRIEVE_CYCLES;
    imprintCycles = HDEMO_DEFAULT_IMPRINT_CYCLES;
    cueErrorRate = HDEMO_DEFAULT_CUE_ERROR;
    cueGenerateOnce = HDEMO_DEFAULT_CUE_GENERATE_ONCE;
    spreadThreshold = HDEMO_DEFAULT_SPREAD_THRESHOLD;
    importanceSpreadingMultiplier = HDEMO_DEFAULT_SPREAD_MULTIPLIER;
    recordToFile = HDEMO_DEFAULT_RECORD_TO_FILE;
    vizThreshold = HDEMO_DEFAULT_VIZ_THRESHOLD;
    resetFlag = false;
	showConfigFlag = false;

}

void HopfieldOptions::printHelp()
{

    const std::string helpOutput =
        "Hopfield network emulator using attention dynamics of OpenCog.\n"
        "Joel Pitt, March 2008.\nCopyright Singularity Institute for Artificial Intelligence\n"
        "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n" "Options:\n"
        "   -? --help   \t Output this screen and exit.\n" "   == Mode ==\n"
        "   [none]      \t Default mode is to imprint, test and repeat.\n"
        "   -R --reset  \t Same as above mode, but completely reset the network between\n"
        "                  \t each imprint and test.\n"
        "   -i --interleave [N] \t Interleave training of each pattern. Optional spacing\n"
        "                  \t between imprinting start.\n" "   == Output ==\n"
        "   -v, --verbose \t Set to verbose output (log level \"DEBUG\")\n"
        "   -d, --debug   \t Set debug output, traces dynamics (log level \"FINE\")\n"
        "   -m --show-matrix \t Show matrices of stored/cue/retrieved pattern at end.\n"
        "   -o --total   \t Report the mean performance increase between cue and retrieval.\n"
        "   -a --log-performance [prefix] log the change cue/retrieval/diff similarity to\n"
        "                  \t separate files, optionally beginning with [prefix].\n"
        "      --result-file <x> \t output retrieved patterns to this file.\n"
        "   -C --show-config\t Print configuration based on options to stdout and exit.\n"
        "   == Network ==\n" "   -w --width N \t Set width of Hopfield network\n"
        "   -h --height N \t Set height of Hopfield network\n"
        "   -n --size N   \t Set width and height of Hopfield network to N\n"
        "   -l --links N \t Add N random Hebbian links to network\n"
        "   -d --density N \t Set the number of links to a ratio of the total possible links\n"
        "                  \t   numLinks = density *  sum(1:(( width * height ) - 1 ))\n"
        "  == Dynamics ==\n"
        "   -s --stimulus N\t Amount of stimulus to give an atom during imprinting or retrieval\n"
        "   -f --focus N \t Attentional focus boundary.\n"
        "   -z --viz-threshold N\t The atom STI needed for an atom to be considered \"on\" when\n"
        "                  \t   compared to original stored pattern.\n"
        "   -c --imprint N \t Number of cycles to imprint pattern.\n"
        "   -r --retrieve N \t Number of cycles to retrieve pattern for.\n"
        "   -t --spread-threshold N  The minimum threshold of atom STI before it gets spread.\n"
        "   -q --spread-multiplier N  multiplier for importance spread, if 0 then evenly\n"
        "                  \t spread across links.\n"
        "   == Pattern commands ==\n"
        "   -p --patterns N \t Number of patterns to test.\n"
        "   -g --gen-density N \t Density of generated patterns (active/inactive nodes).\n"
        "   -e --error N \t Probability of error in each bit of cue pattern.\n"
        "   -E --one-cue \t Only generate a cue file once for a pattern and reuse it.\n"
        "      --train-file <x> \t load patterns from file, must use -n to specify pattern size.\n"
        "      --cue-file <x> \t load patterns from file, must use -n to specify pattern size.\n";
    cout << helpOutput;


}

void HopfieldOptions::parseOptions (int argc, char *argv[])
{
    int c;

    while (1) {
        static const char *optString =
            "vDw:h:n:l:d:s:t:z:f:p:g:c:r:mi:e:oq:a:CR?E";

		static const struct option longOptions[] = {
                {"verbose", 0, &verboseFlag, 1},
                {"debug", 0, &verboseFlag, 2},
                {"width", required_argument, 0, 'w'},
                {"height", required_argument, 0, 'h'},
                {"size", required_argument, 0, 'n'},
                {"links", required_argument, 0, 'l'},
                {"density", required_argument, 0, 'd'},
                {"stimulus", required_argument, 0, 's'},
                {"spread-threshold", required_argument, 0, 't'},
                {"viz-threshold", required_argument, 0, 'z'},
                {"focus", required_argument, 0, 'f'},
                {"patterns", required_argument, 0, 'p'},    // number of patterns to test
                {"gen-denisty", required_argument, 0, 'g'},
                {"imprint", required_argument, 0, 'c'}, // # of imprint _c_ycles
                {"retrieve", required_argument, 0, 'r'},    // # of retrieve iterations
                {"show-matrix", 0, &showMatrixFlag, 1}, // show pattern/cue/result
                {"interleave", optional_argument, 0, 'i'},  // interleave imprint of each pattern
                {"error", required_argument, 0, 'e'},   // cue error rate
                {"total", 0, &totalFlag, 1},    // t_o_tal, reports mean, suitable for batch output
                {"spread-multiplier", required_argument, 0, 'q'},   // multiplier for importance spread, if 0 then evenly spread across links
                {"log-performance", optional_argument, 0, 'a'},
                {"train-file", required_argument, 0, '3'},
                {"cue-file", required_argument, 0, '4'},
                {"result-file", required_argument, 0, '5'},
                {"show-config", 0, &showConfigFlag, 1},
                {"reset", 0, &resetFlag, 1},
                {"one-cue", 0, &cueGenerateOnce, 1},
                {"help", 0, 0, '?'},
                {0, 0, 0, 0}
            };

        int optionIndex = 0;
		bool vizSet = false;
        c = getopt_long (argc, argv, optString, longOptions, &optionIndex);

        /* Detect end of options */
        if (c == -1)
            break;

        switch (c) {
        case 'v':
            verboseFlag = 1;
            break;
        case 'D':
            verboseFlag = 2;
            hServer->importUpdateAgent->getLogger()->enable();
            hServer->importUpdateAgent->getLogger()->setLevel (Logger::FINE);
            hServer->importUpdateAgent->getLogger()->
            setPrintToStdoutFlag (true);
            break;
        case 'w':
            hServer->width = atoi(optarg);
            break;
        case 'h':
            hServer->height = atoi(optarg);
            break;
        case 'n':
            hServer->height = atoi(optarg);
            hServer->width = atoi(optarg);
            break;
        case 'l':
            hServer->links = atoi(optarg);
            break;
        case 'd':
            hServer->density = (float) atof(optarg);
            break;
        case 's':
            hServer->patternStimulus = atoi(optarg);
            break;
        case 't':
            spreadThreshold = atoi(optarg);
            break;
        case 'z':
            vizThreshold = atoi(optarg);
			vizSet = true;
            break;
        case 'f':
            AttentionValue::sti_t f;
            f = (AttentionValue::sti_t) atoi(optarg);
            hServer->getAtomSpace()->setAttentionalFocusBoundary(f);
			// If vizThreshold hasn't been manually set, then update to be
			// the same as the AF boundary.
			if (!vizSet) vizThreshold = (int) f;
            break;
        case 'p':
            nPatterns = atoi(optarg);
            break;
        case 'g':
            genPatternDensity = (float) atof(optarg);
            break;
        case 'c':
            imprintCycles = atoi(optarg);
            break;
        case 'r':
            retrieveCycles = atoi(optarg);
            break;
        case 'e':
            cueErrorRate = (float) atof(optarg);
            break;
        case 'm':
            showMatrixFlag = 1;
            break;
        case 'i':
            interleaveFlag = 1;
            if (optarg) {
                cout << "interleave amount " << optarg << endl;
                interleaveAmount = atoi(optarg);
            }
            break;

        case 'o':
            totalFlag = 1;
            break;
        case 'q':
            importanceSpreadingMultiplier = (float) atof(optarg);
            break;
        case 'a':
            recordToFile = true;
            recordToFilePrefix = optarg;
            break;
        case '3':
            fileTraining = optarg;
            break;
        case '4':
            fileCue = optarg;
            break;
        case '5':
            fileResult = optarg;
            break;
        case 'C':
            showConfigFlag = 1;
            break;
        case 'R':
            resetFlag = 1;
            break;
        case 'E':
            cueGenerateOnce = true;
            break;
        case '?':
            printHelp();
            exit (0);
            break;
        default:
            break;
        }
    }

}

void HopfieldOptions::printConfiguration()
{

    if (interleaveFlag) {
        cout << "Continuous interleaved ";
        cout << "learning, amount/gap = " << interleaveAmount;
        cout << endl;
    } else if (resetFlag) {
        cout << "Pattern by pattern learning (network reset after each)";
        cout << endl;
    } else {
        cout << "Rolling/Continuous learning";
        cout << endl;
    }

    cout << "Network size " << hServer->width << "x" << hServer->height << endl;
    if (hServer->density != -1.0f)
        cout << "Link density " << hServer->density << endl;
    else
        cout << "Links " << hServer->links << endl;

    if (fileTraining.size() > 0) {
        cout << "Loading patterns from " << fileTraining << endl;
    } else {
        cout << "Generating " << nPatterns << " patterns with density "
			<< genPatternDensity << " to store in network." << endl;
    }
    if (fileCue.size() > 0) {
        cout << "Loading cue patterns from " << fileCue << endl;
    } else {
        cout << "Cue patterns generate with error rate " << cueErrorRate <<
        endl;
    }
    if (fileResult.size() > 0) {
        cout << "Saving retrieved patterns to " << fileResult << endl;
    }
    if (showMatrixFlag)
        cout << "Showing grid after each cycle" << endl;
    if (totalFlag)
        cout << "Showing summary of performance at end" << endl;
    if (recordToFile)
        cout << "Recording training performance to file "
			<< recordToFilePrefix << endl;

    cout << "Cycles: " << retrieveCycles << " retrieval, " <<
		imprintCycles << " imprint." << endl;

    cout << "STI spread threshold " << spreadThreshold << endl;
    if (importanceSpreadingMultiplier > 0.0f) {
        cout << "Spread multiplier " << importanceSpreadingMultiplier << endl;
    } else {
        cout << "Spread multiplier is 0, all extra STI will be spread "
			"from an atom." << endl;
    }

    cout << "Stimulus for pattern retrieval " << hServer->patternStimulus
		<< endl;
    cout << "Attention Focus Boundary " <<
		hServer->getAtomSpace()->getAttentionalFocusBoundary() << endl;

    cout << "Visualization threshold " << vizThreshold <<
		"(atoms with STI < then are not \"active\")" << endl;

}

void HopfieldOptions::closeOutputFiles()
{
    beforeFile.close();
    afterFile.close();
    diffFile.close();
}

void HopfieldOptions::openOutputFiles()
{
    beforeFile.open ((recordToFilePrefix + "before.csv").c_str());
    afterFile.open ((recordToFilePrefix + "after.csv").c_str());
    diffFile.open ((recordToFilePrefix + "diff.csv").c_str());
}
