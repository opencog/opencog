#include "HopfieldOptions.h"

#include <iostream>
#include <getopt.h>

using namespace std;

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
    spreadThreshold = HDEMO_DEFAULT_SPREAD_THRESHOLD;
    importanceSpreadingMultiplier = HDEMO_DEFAULT_SPREAD_MULTIPLIER;
    recordToFile = HDEMO_DEFAULT_RECORD_TO_FILE;
    vizThreshold = HDEMO_DEFAULT_VIZ_THRESHOLD;

}

void HopfieldOptions::printHelp()
{

    const std::string helpOutput = "Hopfield network emulator using attention dynamics of OpenCog.\n"
"Joel Pitt, March 2008.\nCopyright Singularity Institute for Artificial Intelligence\n"
"-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n"
"Options:\n"
"   -v, --verbose \t Set to verbose output (log level \"DEBUG\")\n"
"   -d, --debug   \t Set debug output, traces dynamics (log level \"FINE\")\n"
"--\n"
"   -w --width N \t Set width of Hopfield network\n"
"   -h --height N \t Set height of Hopfield network\n"
"   -n --size N   \t Set width and height of Hopfield network to N\n"
"   -l --links N \t Add N random Hebbian links to network\n"
"   -d --density N \t Set the number of links to a ratio of the total possible links\n"
"                  \t   numLinks = density *  sum(1:(( width * height ) - 1 ))\n"
"   -s --stimulus N\t Amount of stimulus to give an atom during imprinting or retrieval\n"
"   -t --spread-threshold N  The minimum threshold of atom STI before it gets spread.\n"
"   -z --viz-threshold N\t The atom STI needed for an atom to be considered \"on\" when\n"
"                  \t   compared to original stored pattern.\n"
"   -f --focus N \t Attentional focus boundary.\n"
"   -p --patterns N \t Number of patterns to test.\n"
"   -g --gen-density N \t Density of generated patterns (active/inactive nodes).\n"
"   -c --imprint N \t Number of _C_ycles to imprint pattern.\n"
"   -r --retrieve N \t Number of cycles to retrieve pattern for.\n"
"   -m --show-matrix \t Show matrices of stored/cue/retrieved pattern at end.\n"
"   -i --interleave [N] \t Interleave training of each pattern. Optional spacing\n"
"                  \t between imprinting start.\n"
"   -e --error N \t probability of error in each bit of cue pattern.\n"
"   -o --total   \t Report the mean performance increase between cue and retrieval.\n"
"   -a --log-performance [prefix] log the change cue/retrieval/diff similarity to\n"
"                  \t separate files, optionally beginning with [prefix].\n"
"   -q --spread-multiplier N  multiplier for importance spread, if 0 then evenly\n"
"                  \t spread across links.\n";
    cout << helpOutput;


}

void HopfieldOptions::parseOptions(int argc, char *argv[])
{
    int c;

    while (1) {
	static const char *optString = "vDw:h:n:l:d:s:t:z:f:p:g:c:r:mi:e:oq:a:?";

	static const struct option longOptions[] = {
	    /* These options are flags */
	    {"verbose", 0, &verboseFlag, 1},
	    {"debug", 0, &verboseFlag, 2},
	    /* These options set variables */
	    {"width", required_argument, 0, 'w'},
	    {"height", required_argument, 0, 'h'},
	    {"size", required_argument, 0, 'n'},
	    {"links", required_argument, 0, 'l'},
	    {"density", required_argument, 0, 'd'},
	    {"stimulus", required_argument, 0, 's'},
	    {"spread-threshold", required_argument, 0, 't'},
	    {"viz-threshold", required_argument, 0, 'z'},
	    {"focus", required_argument, 0, 'f'},
	    {"patterns", required_argument, 0, 'p'}, // number of patterns to test
	    {"gen-denisty", required_argument, 0, 'g'},
	    {"imprint", required_argument, 0, 'c'}, // # of imprint _c_ycles
	    {"retrieve", required_argument, 0, 'r'}, // # of retrieve iterations
	    {"show-matrix", 0, &showMatrixFlag, 1}, // show pattern/cue/result
	    {"interleave", optional_argument, 0, 'i'}, // interleave imprint of each pattern
	    {"error", required_argument, 0, 'e'}, // cue error rate
	    {"total", 0, &totalFlag, 1}, // t_o_tal, reports mean, suitable for batch output
	    {"spread-multiplier", required_argument, 0, 'q'}, // multiplier for importance spread, if 0 then evenly spread across links
	    {"log-performance", optional_argument, 0, 'a'},
	    {0,0,0,0}
	};

	int optionIndex = 0;
	c = getopt_long(argc, argv, optString, longOptions, &optionIndex);

	/* Detect end of options */
	if (c==-1) break;

	switch (c) {
	   case 'v':
		verboseFlag = 1; break;
	   case 'D':
		verboseFlag = 2;
		hServer->importUpdateAgent->getLogger()->enable();
		hServer->importUpdateAgent->getLogger()->setLevel(Util::Logger::FINE);
		hServer->importUpdateAgent->getLogger()->setPrintToStdoutFlag(true);
		break;
	    case 'w':
		hServer->width=atoi(optarg); break;
	    case 'h':
		hServer->height=atoi(optarg); break;
	    case 'n':
		hServer->height=atoi(optarg);
		hServer->width=atoi(optarg);
		break;
	    case 'l':
		hServer->links=atoi(optarg); break;
	    case 'd':
		hServer->density=atof(optarg); break;
	    case 's':
		hServer->perceptStimUnit=atoi(optarg); break;
	    case 't':
		spreadThreshold=atoi(optarg); break;
	    case 'z':
		vizThreshold=atoi(optarg); break;
	    case 'f':
		AttentionValue::sti_t f;
		f = (AttentionValue::sti_t) atoi(optarg);
		hServer->getAtomSpace()->setAttentionalFocusBoundary(f);
		break;
	    case 'p':
		nPatterns = atoi(optarg); break;
	    case 'g':
		genPatternDensity = atof(optarg); break;
	    case 'c':
		imprintCycles = atoi(optarg); break;
	    case 'r':
		retrieveCycles = atoi(optarg); break;
	    case 'e':
		cueErrorRate = atof(optarg); break;
	    case 'm':
		showMatrixFlag = 1; break;
	    case 'i':
		interleaveFlag = 1;
		if (optarg) {
		    cout << "interleave amount " << optarg << endl;
		    interleaveAmount = atoi(optarg);
		}
		break;

	    case 'o':
		totalFlag = 1; break;
	    case 'q':
		importanceSpreadingMultiplier = atof(optarg); break;
	    case 'a':
		recordToFile = true;
		recordToFilePrefix = optarg;
		break;

	    case '?':
		printHelp();
		exit(0);
		break;
	    default:
		break;
	}
    }

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
