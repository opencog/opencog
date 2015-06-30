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

#include "HopfieldServer.h"

using namespace std;
using namespace opencog;

HopfieldOptions::HopfieldOptions()
{
    // Set defaults
    verboseLevel = HDEMO_DEFAULT_VERBOSE;
    learningScheme = HDEMO_DEFAULT_SCHEME;
    updateMethod = HDEMO_DEFAULT_UPDATE_METHOD;
    interleaveAmount = HDEMO_DEFAULT_INTERLEAVEAMOUNT;
    palimpsestTolerance = HDEMO_DEFAULT_PALIMPSEST_TOLERANCE;
    showMatrixFlag = HDEMO_DEFAULT_SHOW_MATRIX;
    totalFlag = HDEMO_DEFAULT_SHOW_TOTAL;
    nPatterns = HDEMO_DEFAULT_NPATTERNS;
    keyNodes = HDEMO_DEFAULT_KEY_NODES;
    genPatternDensity = HDEMO_DEFAULT_PATTERN_DENSITY;
    retrieveCycles = HDEMO_DEFAULT_RETRIEVE_CYCLES;
    spreadCycles = HDEMO_DEFAULT_SPREAD_CYCLES;
    imprintCycles = HDEMO_DEFAULT_IMPRINT_CYCLES;
    cueErrorRate = HDEMO_DEFAULT_CUE_ERROR;
    cueGenerateOnce = HDEMO_DEFAULT_CUE_GENERATE_ONCE;
    spreadThreshold = HDEMO_DEFAULT_SPREAD_THRESHOLD;
    importanceSpreadingMultiplier = HDEMO_DEFAULT_SPREAD_MULTIPLIER;
    recordToFile = HDEMO_DEFAULT_RECORD_TO_FILE;
    vizThreshold = HDEMO_DEFAULT_VIZ_THRESHOLD;
    resetFlag = false;
	showConfigFlag = false;

    visualize = HDEMO_DEFAULT_VISUALIZE;
    visDelay = HDEMO_DEFAULT_VIS_DELAY;
    visLabel = HDEMO_DEFAULT_VIS_PROCESS_LABELS;

    forgetPercent = HDEMO_DEFAULT_FORGET_PERCENT;

    diffusionThreshold = HDEMO_DEFAULT_DIFFUSION_THRESHOLD;
    maxSpreadPercentage = HDEMO_DEFAULT_MAX_SPREAD_PERCENTAGE;
    deciderFunctionShape = HDEMO_DEFAULT_DECIDER_SHAPE;
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
        "   -i --interleave <N> \t Interleave training of each pattern with N spacing\n"
        "                  \t between imprinting start.\n"
        "   -P --palimpsest <N> \t Palimpsest testing using one cue pattern with N \% tolerance\n"
        "      --pneighbours <N>\t Palimpsest testing using all neighbours with N \% tolerance\n"
        "   == Output ==\n"
        "   -v, --verbose \t Set to verbose output, can be used more than once for more\n"
        "                 \t detail (log level \"INFO\" -> \"DEBUG\" -> \"FINE\")\n"
        "   -m --show-matrix \t Show matrices of stored/cue/retrieved pattern at end.\n"
        "   -o --total   \t Report the mean performance increase between cue and retrieval.\n"
        "   -a --log-performance [prefix] log the change cue/retrieval/diff similarity to\n"
        "                  \t separate files, optionally beginning with [prefix].\n"
        "      --result-file <x> \t output retrieved patterns to this file.\n"
        "   -C --show-config\t Print configuration based on options to stdout and exit.\n"
        "      --visualize\t Visualise dynamics using Ubigraph (server must be running).\n"
        "      --vdelay N \t The float multiplier for delays between learning steps to make\n"
        "                 \t dynamics easier to follow. Default: 0 (no delays).\n"
        "      --vlabel  \t Display labels that update to reflect the current learning step.\n"
        "   == Network ==\n" "   -w --width N \t Set width of Hopfield network\n"
        "   -h --height N \t Set height of Hopfield network\n"
        "   -n --size N   \t Set width and height of Hopfield network to N\n"
        "   -l --links N \t Add N random Hebbian links to network\n"
        "   -d --density N \t Set the number of links to a ratio of the total possible links\n"
        "                  \t   numLinks = density *  sum(1:(( width * height ) - 1 ))\n"
        "  == Dynamics ==\n"
        "   -u --update-rule <x>\t The rule used to update the connections, one of [conjunction,\n"
        "                       \t storkey].\n"
        "   -s --stimulus N\t Amount of stimulus to give an atom during imprinting or retrieval\n"
        "   -f --focus N \t Attentional focus boundary.\n"
        "   -z --viz-threshold N\t The atom STI needed for an atom to be considered \"on\" when\n"
        "                  \t   compared to original stored pattern.\n"
        "   -c --imprint N \t Number of cycles to imprint pattern.\n"
        "   -r --retrieve N \t Number of cycles to retrieve pattern for.\n"
        "   -y --spread N \t Number of times to spread importance during each retrieve cycle.\n"
        "   -t --spread-threshold N  The minimum threshold of atom STI before it gets spread.\n"
        "   -q --spread-multiplier N  multiplier for importance spread, if 0 then evenly\n"
        "                  \t spread across links.\n"
        "   -F --forget N \t percentage of links to forget.\n"
        "   -k --key N    \t number of key nodes for glocal storage.\n"
        "      --diffuse-t N\t Threshold of diffusion [-1..1], default 0==AF.\n"
        "      --diffuse-max N\t Maximum diffusion percentage [0..1], default 1==100\%.\n"
        "      --diffuse-s N\t Shape of hyperbolic diffusion decision function, default 30.\n"
        "   == Pattern commands ==\n"
        "   -p --patterns N \t Number of patterns to test.\n"
        "   -g --gen-density N \t Density of generated patterns (active/inactive nodes).\n"
        "   -e --error N \t Probability of error (if N < 1) in each bit of cue pattern or\n"
        "                \t number of bit errors (if N >= 1) in cue pattern.\n"
        "   -E --one-cue \t Only generate a cue once for a pattern and reuse it.\n"
        "      --train-file <x> \t load patterns from file, must use -n to specify pattern size.\n"
        "      --cue-file <x> \t load patterns from file, must use -n to specify pattern size.\n";
    cout << helpOutput;
}

void HopfieldOptions::parseOptions (int argc, char *argv[])
{
    int c;
    bool vizSet = false;
    static const char *optString =
        "vw:h:n:l:d:s:u:t:z:f:p:g:c:r:y:mi:P:e:oq:F:a:k:CR?E";

    static const struct option longOptions[] = {
        {"verbose", 0, &verboseLevel, 1},
        {"debug", 0, &verboseLevel, 3},
        {"width", required_argument, 0, 'w'},
        {"height", required_argument, 0, 'h'},
        {"size", required_argument, 0, 'n'},
        {"links", required_argument, 0, 'l'},
        {"density", required_argument, 0, 'd'},
        {"update-rule", required_argument, 0, 'u'},
        {"stimulus", required_argument, 0, 's'},
        {"spread-threshold", required_argument, 0, 't'},
        {"viz-threshold", required_argument, 0, 'z'},
        {"focus", required_argument, 0, 'f'},
        {"patterns", required_argument, 0, 'p'},    // number of patterns to test
        {"gen-density", required_argument, 0, 'g'},
        {"imprint", required_argument, 0, 'c'}, // # of imprint _c_ycles
        {"retrieve", required_argument, 0, 'r'},    // # of retrieve iterations
        {"spread", required_argument, 0, 'y'},    // # of spread iterations
        {"show-matrix", 0, &showMatrixFlag, 1}, // show pattern/cue/result
        {"interleave", optional_argument, 0, 'i'},  // interleave imprint of each pattern
        {"palimpsest", optional_argument, 0, 'P'},  // set Palimpsest testing
        // Palimpsest testing of neighbours
        {"pneighbours", optional_argument, 0, '7'},
        {"error", required_argument, 0, 'e'},   // cue error rate
        {"total", 0, &totalFlag, 1},    // t_o_tal, reports mean, suitable for batch output
        {"spread-multiplier", required_argument, 0, 'q'},   // multiplier for importance spread, if 0 then evenly spread across links
        {"forget", required_argument, 0, 'F'},
        {"key", required_argument, 0, 'k'},
        {"log-performance", optional_argument, 0, 'a'},
        {"train-file", required_argument, 0, '3'},
        {"cue-file", required_argument, 0, '4'},
        {"result-file", required_argument, 0, '5'},
        {"show-config", 0, &showConfigFlag, 1},
        {"reset", 0, &resetFlag, 1},
        {"one-cue", 0, &cueGenerateOnce, 1},
        {"visualize", 0, &visualize, 1},
        {"vdelay", required_argument, 0, '6'},
        {"vlabel", 0, &visLabel, 1},
        {"diffuse-t", required_argument, 0, '8'},
        {"diffuse-max", required_argument, 0, '9'},
        {"diffuse-s", required_argument, 0, '2'},
        {"help", 0, 0, '?'},
        {0, 0, 0, 0}
    };

    while (1) {
        int optionIndex = 0;
        c = getopt_long (argc, argv, optString, longOptions, &optionIndex);

        /* Detect end of options */
        if (c == -1)
            break;

        switch (c) {
        case 'v':
            verboseLevel++;
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
        case 'u':
            if (strcasecmp("storkey",optarg) == 0) {
                updateMethod = STORKEY;
            } else if (strcasecmp("conjunction",optarg) == 0) {
                updateMethod = CONJUNCTION;
            } else {
                cout << "Unknown update method, using conjunction.\n";
                updateMethod = CONJUNCTION;
            }
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
            hServer->getAtomSpace().set_attentional_focus_boundary(f);
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
        case 'y':
            spreadCycles = atoi(optarg);
            break;
        case 'e':
            cueErrorRate = (float) atof(optarg);
            break;
        case 'm':
            showMatrixFlag = 1;
            break;
        case 'i':
            learningScheme = INTERLEAVE;
            if (optarg) {
                cout << "interleave amount " << optarg << endl;
                interleaveAmount = atoi(optarg);
            }
            break;
        case 'P':
            learningScheme = PALIMPSEST;
            if (optarg) {
                //cout << "palimpsest tolerance " << optarg << endl;
                palimpsestTolerance = atoi(optarg);
            }
            break;
        case '7':
            learningScheme = PALIMPSEST_NEIGHBOURS;
            if (optarg) {
                //cout << "palimpsest tolerance " << optarg << endl;
                palimpsestTolerance = atoi(optarg);
            }
            break;
        case 'o':
            totalFlag = 1;
            break;
        case 'q':
            importanceSpreadingMultiplier = (float) atof(optarg);
            break;
        case 'F':
            forgetPercent = (float) atof(optarg);
            break;
        case 'k':
            keyNodes = atoi(optarg);
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
        case '6':
            visDelay = atof(optarg);
            break;
        case '8':
            diffusionThreshold = atof(optarg);
            break;
        case '9':
            maxSpreadPercentage = atof(optarg);
            break;
        case '2':
            deciderFunctionShape = atof(optarg);
            break;
        case '?':
            printHelp();
            exit (0);
            break;
        default:
            break;
        }
    }
#ifndef HAVE_UBIGRAPH
    if (visualize) {
        cout << "Warning: Ubigraph visualisation not enabled on compilation."
            << endl << "Ignoring visualisation options" << endl;
    }
#endif

}

void HopfieldOptions::printConfiguration()
{

    if (learningScheme == INTERLEAVE) {
        cout << "Continuous interleaved ";
        cout << "learning, amount/gap = " << interleaveAmount;
        cout << endl;
    } else if (learningScheme == PALIMPSEST) {
        cout << "Palimpsest learning. ";
        cout << "With tolerance = " << palimpsestTolerance << "\%";
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
		hServer->getAtomSpace().get_attentional_focus_boundary() << endl;

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
