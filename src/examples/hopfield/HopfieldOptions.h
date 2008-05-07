/**
 * Options.h
 *
 * options for hopfield network emulator
 *
 * Author: Joel Pitt
 * Creation: Wed Apr 4 11:14:22 GMT+12 2008
 */
#ifndef HDEMO_OPTIONS_H
#define HDEMO_OPTIONS_H

#include <fstream>

#include <AttentionValue.h>

#include "HopfieldServer.h"

#define HDEMO_DEFAULT_VERBOSE 0
#define HDEMO_DEFAULT_INTERLEAVE false
#define HDEMO_DEFAULT_INTERLEAVEAMOUNT 5
#define HDEMO_DEFAULT_SHOW_MATRIX false
#define HDEMO_DEFAULT_SHOW_TOTAL false
#define HDEMO_DEFAULT_NPATTERNS 1
#define HDEMO_DEFAULT_PATTERN_DENSITY 0.2
#define HDEMO_DEFAULT_RETRIEVE_CYCLES 10
#define HDEMO_DEFAULT_IMPRINT_CYCLES 15
#define HDEMO_DEFAULT_CUE_ERROR 0.1
#define HDEMO_DEFAULT_CUE_GENERATE_ONCE false
#define HDEMO_DEFAULT_SPREAD_MULTIPLIER 10.0f
#define HDEMO_DEFAULT_RECORD_TO_FILE false
#define HDEMO_DEFAULT_VIZ_THRESHOLD 5
#define HDEMO_DEFAULT_SPREAD_THRESHOLD 4

class HopfieldServer;

class HopfieldOptions {
    private:

	HopfieldServer* hServer;

    public:
	int verboseFlag;
	int resetFlag;
	int interleaveFlag;
	int interleaveAmount;
	int showMatrixFlag;
	int showConfigFlag;
	int totalFlag;
	int nPatterns;
	float genPatternDensity;
	int retrieveCycles;
	int imprintCycles;
	float cueErrorRate;
	int cueGenerateOnce;
	float importanceSpreadingMultiplier;
	AttentionValue::sti_t spreadThreshold;
	int recordToFile;
	std::string recordToFilePrefix;
	AttentionValue::sti_t vizThreshold;

	HopfieldOptions();

	void parseOptions(int argc, char *argv[]);
	void printHelp();
	void printConfiguration();
	void setServer(HopfieldServer* s) {hServer=s;};

	void openOutputFiles();
	void closeOutputFiles();
	std::ofstream beforeFile;
	std::ofstream afterFile;
	std::ofstream diffFile;

	std::string fileTraining;
	std::string fileCue;
	std::string fileResult;

};

#endif // HDEMO_OPTIONS_H
