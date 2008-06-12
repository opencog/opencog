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
 * HopfieldServer.h
 *
 * Emulates a hopfield network using OpenCog dynamics
 */

#ifndef HOPFIELDSERVER_H
#define HOPFIELDSERVER_H

#include <CogServer.h>
#include <RandGen.h>
#include <ImportanceUpdatingAgent.h>
#include <HebbianLearningAgent.h>
#include <ImportanceSpreadingAgent.h>
#include <ForgettingAgent.h>

#include <vector>
#include <math.h>

#include "HopfieldOptions.h"
#include "Pattern.h"

#define HDEMO_DEFAULT_WIDTH 3
#define HDEMO_DEFAULT_HEIGHT 3
#define HDEMO_DEFAULT_LINKS 15
#define HDEMO_DEFAULT_PATTERN_STIM 1000

namespace opencog
{

namespace opencog
{

namespace opencog
{

class HopfieldOptions;

class HopfieldServer : public opencog::CogServer
{

private:

    /* Nodes in the Hopfield network can be referenced
     * through HGrid.
     */
    std::vector<Handle> hGrid;

    opencog::RandGen* rng;

public:
    // Amount of stimulus to apply across a pattern
    stim_t patternStimulus;

    opencog::ImportanceUpdatingAgent *importUpdateAgent;
    opencog::HebbianLearningAgent *hebLearnAgent;
    opencog::ImportanceSpreadingAgent *spreadAgent;
    opencog::ForgettingAgent *forgetAgent;

    HopfieldOptions *options;

    int width, height, links;
    float density;

    ~HopfieldServer();
    HopfieldServer();

    /**
     * Initialise the demo with a lattice of width * height nodes.
     *
     * @param width number of nodes across
     * @param height number of nodes vertically
     * @param numLinks number of links randomly connecting nodes
     */
    void init(int width, int height, int numLinks);

    /**
     * Encode a pattern onto the network.
     *
     * @param pattern a vector of boolean values. Each position mapping
     * to the node in hGrid.
     * @param stimulus amount of stimulus to multiply values in pattern by
     */
    void encodePattern(Pattern pattern, stim_t stimulus);

    /**
     * Retrieve the the closest matching pattern in the network.
     *
     * @param pattern is the pattern to match
     * @param numCycles is the number of lobe cycles to wait for retrieval
     */
    Pattern retrievePattern(Pattern pattern, int numCycles);

    /**
     *
     */
    void updateAtomTableForRetrieval(int spreadCycles);

    template<typename Number> std::string patternToString(std::vector<Number> p) {
        std::stringstream ss;
        Number col = 0;

        typename std::vector<Number>::iterator it = p.begin();

        while (it != p.end()) {
            ss << *it << " ";
            col++;
            if (col == width) {
                ss << endl;
                col = 0;
            }
            it++;
        }
        return ss.str();
    }

    Pattern getGridSTIAsPattern();
    std::vector<stim_t> getGridStimVector();

    /**
     * Remove all links and replace.
     */
    void reset();
    /**
     * Reset STI of all nodes to zero.
     */
    void resetNodes(bool toDefault=false);
    void imprintPattern(Pattern pattern, int cycles);

    void doForgetting(float proportion);
    void addRandomLinks();

    // Make this retrieve data from the grid rather than from argument
    std::string printMatrixResult(std::vector< Pattern > p1);

    /**
     * imprint a pattern for imprint cycles.
     *
     * @param p pattern to imprint
     * @param imprint number of cycles to imprint pattern for
     * @param retrieve number of cycles to retrieve pattern after each imprint
     * @param mutate amount of mutation to apply to pattern to reapply after
     * each imprint.
     * @return vector with the hamming similarity of the retrieved pattern after
     * each imprint
     */
    std::vector<float> imprintAndTestPattern(Pattern p, int imprint, int retrieve, float mutate);

    /**
     * imprint a pattern once. After, try
     * retrieving the pattern with retrieve cycles, optionally mutated.
     *
     * @param p pattern to imprint
     * @param imprint number of cycles to imprint pattern for
     * @param retrieve number of cycles to retrieve pattern after each imprint
     * @param mutate amount of mutation to apply to pattern to reapply after
     * each imprint.
     * @return vector with the hamming similarity of the retrieved pattern after
     * each imprint
     */
    float singleImprintAndTestPattern(Pattern p, int retrieve, float mutate, Pattern c);

    void printStatus();
};

} // namespace opencog

#endif // HOPFIELDSERVER_H
