
#include <fstream>

#include "HopfieldServer.h"

void testHopfieldNetworkRolling();
void testHopfieldNetworkRollingOld();
void testHopfieldNetworkInterleave();

HopfieldServer hDemo;
HopfieldOptions *o = hDemo.options;

int main(int argc, char *argv[])
{
    //int patternArray[] = { 0, 1, 0, 1, 1, 1, 0, 1, 0 };
    //std::vector<int> pattern1(patternArray, patternArray + 9);
    o->parseOptions(argc, argv);

    /* setup logging */
    MAIN_LOGGER.setPrintToStdoutFlag(true);
    if (o->verboseFlag == 1) {
	MAIN_LOGGER.setLevel(Util::Logger::DEBUG);
    } else if (o->verboseFlag == 2) {
	MAIN_LOGGER.setLevel(Util::Logger::FINE);
    } else {
	MAIN_LOGGER.setLevel(Util::Logger::WARNING);
    }
	
    MAIN_LOGGER.log(Util::Logger::INFO,"Init HopfieldServer");
    hDemo.init(-1, -1, -1);

    if (o->recordToFile) o->openOutputFiles();

    if (o->interleaveFlag)
	testHopfieldNetworkInterleave();
    else
        testHopfieldNetworkRolling();

    if (o->recordToFile) o->closeOutputFiles();
}

void testHopfieldNetworkInterleave()
{
# define HDEMO_NORESULT -100
    std::vector< Pattern > patterns;
    std::vector< std::vector < float > > results;
    int totalCycles = (o->interleaveAmount * (o->nPatterns - 1)) + o->imprintCycles;

    patterns = Pattern::generateRandomPatterns(o->nPatterns,hDemo.width,hDemo.height,o->genPatternDensity);

    if (o->showMatrixFlag) {
	hDemo.printMatrixResult(patterns);
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
	    endPattern = i/o->interleaveAmount;
	    if ((unsigned int) endPattern >= patterns.size()) endPattern = patterns.size() - 1;
	} else 
	    endPattern = patterns.size() - 1;

	// Imprint each of them.
	for (unsigned int j = startPattern; j <= (unsigned int) endPattern; j++) {
	    hDemo.resetNodes();
	    hDemo.imprintPattern(patterns[j],1);
	    //if (!verboseFlag) cout << ".";
	}

	for (unsigned int j = 0; j < patterns.size(); j++) {
	    float rSim;
	    if (j <= (unsigned int) endPattern) {
		Pattern c = patterns[j].mutatePattern(o->cueErrorRate);
		Pattern rPattern = hDemo.retrievePattern(c,o->retrieveCycles);
		rSim = patterns[j].hammingSimilarity(rPattern);
		cycleResults.push_back(rSim);
		cout << rSim << "(" << rSim-patterns[j].hammingSimilarity(c) <<")" << ", ";
		if (o->showMatrixFlag) {
		    toPrint.push_back(rPattern);
		}
	    } else {
		cycleResults.push_back(HDEMO_NORESULT);
		cout << "-" << ", ";
	    }

	}
	if (!o->verboseFlag) cout << endl; 
	if (o->showMatrixFlag) hDemo.printMatrixResult(toPrint);
	results.push_back(cycleResults);

    }


}

void testHopfieldNetworkRolling()
{
    std::vector< Pattern > patterns;
    std::vector< std::vector<float> > results;
    std::vector<float> diffs;

    patterns = Pattern::generateRandomPatterns(o->nPatterns,hDemo.width,hDemo.height,o->genPatternDensity);

    for (unsigned int i = 0; i< patterns.size(); i++) {
	results.push_back(hDemo.imprintAndTestPattern(patterns[i],o->imprintCycles,o->retrieveCycles,o->cueErrorRate));
	if (!o->verboseFlag) cout << " - pattern " << i << " done" << endl;
	if (o->recordToFile) {
	    o->beforeFile << endl;
	    o->afterFile << endl;
	    o->diffFile << endl;
	}
    }
    for (unsigned int i = 0; i< patterns.size(); i++) {
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

void testHopfieldNetworkRollingOld()
{
    std::vector< Pattern > patterns;
    std::vector< Pattern > cuePatterns;
    std::vector< Pattern > rPatterns;
    Pattern result(hDemo.width, hDemo.height);
    std::vector<float> diffs;

    patterns = Pattern::generateRandomPatterns(o->nPatterns,hDemo.width,hDemo.height,o->genPatternDensity);
    cuePatterns = Pattern::mutatePatterns(patterns, o->cueErrorRate);

    for (unsigned int i = 0; i< patterns.size(); i++) {
	hDemo.imprintPattern(patterns[i],o->imprintCycles);
	MAIN_LOGGER.log(Util::Logger::INFO,"Encoded pattern and ran server for %d loops",o->imprintCycles);
    }

    for (unsigned int i = 0; i< patterns.size(); i++) {
	result = hDemo.retrievePattern(cuePatterns[i],o->retrieveCycles);
	MAIN_LOGGER.log(Util::Logger::INFO,"Updated Atom table for retrieval");
	rPatterns.push_back(result);
    }

    if (! o->totalFlag) cout << "-----------------------" << endl;
    for (unsigned int i = 0; i< patterns.size(); i++) {
	float before, after, diff;

	if (o->showMatrixFlag) {
	    std::vector< Pattern > toPrint;
	    toPrint.push_back(patterns[i]);
	    toPrint.push_back(cuePatterns[i]);
	    toPrint.push_back(rPatterns[i]);
	    hDemo.printMatrixResult(toPrint);
	}

	before = patterns[i].hammingSimilarity(cuePatterns[i]); 
	after = patterns[i].hammingSimilarity(rPatterns[i]);
	diff = after-before;
	diffs.push_back(diff);
	
	if (! o->totalFlag) cout << "=== similarity before/after/diff: " <<
	    before << "/" << after << "/" << diff << endl;
    }

    if (o->totalFlag) {
	float total = 0.0f;
	for (unsigned int i = 0; i < diffs.size(); i++)
	    total += diffs[i];
	cout << total / diffs.size() << endl;
	
    }

}

