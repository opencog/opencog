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

/**
 * Options.h
 *
 * options for hopfield network emulator
 */
#ifndef _OPENCOG_HDEMO_OPTIONS_H
#define _OPENCOG_HDEMO_OPTIONS_H

#include <sys/types.h>
#include <fstream>

#include <opencog/atoms/truthvalue/AttentionValue.h>

#define HDEMO_DEFAULT_VERBOSE 0
#define HDEMO_DEFAULT_SCHEME SEQUENCE
#define HDEMO_DEFAULT_UPDATE_METHOD CONJUNCTION
#define HDEMO_DEFAULT_INTERLEAVEAMOUNT 5
#define HDEMO_DEFAULT_PALIMPSEST_TOLERANCE 5
#define HDEMO_DEFAULT_SHOW_MATRIX false
#define HDEMO_DEFAULT_SHOW_TOTAL false
#define HDEMO_DEFAULT_NPATTERNS 1
#define HDEMO_DEFAULT_PATTERN_DENSITY 0.2f
#define HDEMO_DEFAULT_RETRIEVE_CYCLES 10
#define HDEMO_DEFAULT_IMPRINT_CYCLES 15
#define HDEMO_DEFAULT_CUE_ERROR 0.1f
#define HDEMO_DEFAULT_CUE_GENERATE_ONCE false
#define HDEMO_DEFAULT_SPREAD_MULTIPLIER 10.0f
#define HDEMO_DEFAULT_RECORD_TO_FILE false
#define HDEMO_DEFAULT_VIZ_THRESHOLD 1
#define HDEMO_DEFAULT_SPREAD_THRESHOLD 0
#define HDEMO_DEFAULT_SPREAD_CYCLES 1
#define HDEMO_DEFAULT_KEY_NODES 0
#define HDEMO_DEFAULT_VISUALIZE 0
#define HDEMO_DEFAULT_VIS_DELAY 1.0f
#define HDEMO_DEFAULT_VIS_PROCESS_LABELS false

#define HDEMO_DEFAULT_FORGET_PERCENT 0.05 

#define HDEMO_DEFAULT_DIFFUSION_THRESHOLD 0.0f
#define HDEMO_DEFAULT_MAX_SPREAD_PERCENTAGE 1.0f
#define HDEMO_DEFAULT_DECIDER_SHAPE 30

namespace opencog
{

class HopfieldServer;

/** HopfieldOptions parses and stores command line arguments
 *
 * @todo replace usage of getopt with tclap http://tclap.sourceforge.net/ -
 * package libtclap-dev in Ubuntu.
 * @todo get default values from Config/opencog.conf
 */
class HopfieldOptions
{
private:

    HopfieldServer* hServer;

public:
    enum learningScheme_t {SEQUENCE = 0, INTERLEAVE, PALIMPSEST,
        PALIMPSEST_NEIGHBOURS};
    learningScheme_t learningScheme;

    enum updateMethod_t {CONJUNCTION = 0, STORKEY };
    updateMethod_t updateMethod;

    int verboseLevel;
    int resetFlag;
    int interleaveAmount;
    int palimpsestTolerance;
    int showMatrixFlag;
    int showConfigFlag;

    //! Whether to connect to ubigraph server and visualise
    int visualize;
    //! Speed of visualisation (all in build delays are multiplied
    //! by this).
    float visDelay; 
    //! Should there be a labelled metavertex whose label shows
    //! what step of the process the network is carrying out?
    int visLabel;

    int totalFlag;
    int nPatterns;
    uint keyNodes;
    float genPatternDensity;

    int retrieveCycles;
    int spreadCycles;
    int imprintCycles;

    //! Importance diffusion options
    float diffusionThreshold;
    float maxSpreadPercentage;
    float deciderFunctionShape;

    //! Percentage of links to forget
    float forgetPercent;

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
    void setServer(HopfieldServer* s) {
        hServer = s;
    };

    void openOutputFiles();
    void closeOutputFiles();
    std::ofstream beforeFile;
    std::ofstream afterFile;
    std::ofstream diffFile;

    std::string fileTraining;
    std::string fileCue;
    std::string fileResult;

};

} // namespace opencog

#endif // _OPENCOG_HDEMO_OPTIONS_H
