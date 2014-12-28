/*
 * examples/hopfield/HopfieldServer.h
 *
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

#ifndef _OPENCOG_HOPFIELD_SERVER_H
#define _OPENCOG_HOPFIELD_SERVER_H

#include <sstream>
#include <vector>

#include <math.h>

#include <opencog/dynamics/attention/ForgettingAgent.h>
#include <opencog/dynamics/attention/HebbianUpdatingAgent.h>
#include <opencog/dynamics/attention/ImportanceSpreadingAgent.h>
#include <opencog/dynamics/attention/ImportanceUpdatingAgent.h>
#include <opencog/dynamics/attention/ImportanceDiffusionAgent.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/RandGen.h>

#include "StorkeyAgent.h"
#include "ImprintAgent.h"
#include "Pattern.h"

#define HDEMO_DEFAULT_WIDTH 3
#define HDEMO_DEFAULT_HEIGHT 3
#define HDEMO_DEFAULT_LINKS 15
#define HDEMO_DEFAULT_PATTERN_STIM 1000

namespace opencog
{

class HopfieldOptions;
class ForgettingAgent;
class HebbianUpdatingAgent;
class ImportanceDiffusionAgent;
class ImportanceSpreadingAgent;
class ImportanceUpdatingAgent;
class StorkeyAgent;
class ImprintAgent;
#ifdef HAVE_UBIGRAPH
class HopfieldUbigrapher;
#endif //HAVE_UBIGRAPH

/** Emulates a hopfield network using OpenCog dynamics */
class HopfieldServer : public CogServer
{

private:

    RandGen* rng;
    //! Find which key Node the current distribution of STI is closest to.
    Handle findKeyNode();
    //! Update the key node so that links exist to \arg density percent of the
    //! pattern nodes.
    void updateKeyNodeLinks(Handle keyHandle, float density = 1.0f);

    //! choose which grid nodes will be replaced by key nodes
    void chooseKeyNodes();
    //! get a map of destinations to links from src (when following links
    //! that inherit from linkType.
    std::map<Handle,Handle> getDestinationsFrom(Handle src, Type linkType);

#ifdef HAVE_UBIGRAPH
    //! Ubigrapher for visualising
    HopfieldUbigrapher* ubi;
#endif //HAVE_UBIGRAPH

    HandleSeq recentlyAddedLinks;
public:

    static opencog::BaseServer* derivedCreateInstance(void);

    //! Amount of stimulus to apply across a pattern
    stim_t patternStimulus;

    ForgettingAgentPtr forgetAgent;
    HebbianUpdatingAgentPtr hebUpdateAgent;
    StorkeyAgentPtr storkeyAgent;
    ImprintAgentPtr imprintAgent;
    ImportanceDiffusionAgentPtr diffuseAgent;
    ImportanceSpreadingAgentPtr spreadAgent;
    ImportanceUpdatingAgentPtr importUpdateAgent;
    HopfieldOptions *options;

    int width, height, links;
    float density;

    //! Nodes in the Hopfield network can be referenced through HGrid.
    std::vector<Handle> hGrid;
    //! Indication of whether observation nodes in grid are active or key
    //! nodes.
    std::vector<bool> hGridKey;

    //! Key nodes for glocal training
    std::vector<Handle> keyNodes;

    HopfieldServer();
    virtual ~HopfieldServer();

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
    
    /** Calculate the total energy of the network.
     *
     * Total energy is calculated from STI values and Hebbian Link weights...
     * thus the energy is in terms of STI 
     *
     * @return total energy.
     * @todo implement as an AtomSpace function or AtomTable statistic for total
     * free energy based on STI and that handles HebbianLinks with arity != 2.
     * @bug this isn't implemented properly!! Results are erroneous. Should
     * take into account diffusion/spread threshold of agents.
     */
    float totalEnergy();

    /**
     * Retrieve the the closest matching pattern in the network.
     *
     * @param pattern is the pattern to match
     * @param numCycles is the number of lobe cycles to wait for retrieval
     * @param spreadCycles is the number of times to spread importance per
     * retrieval cycle.
     * @param originalPattern for visualisation only.
     * @return closest matching pattern in network.
     */
    Pattern retrievePattern(Pattern pattern, int numCycles, int spreadCycles,
            Pattern originalPattern = Pattern(0,0));

    /**
     * Check the stability of all 1 bit changes to the Pattern. In other words
     * check the local neighbourhood of the network to see if pattern is a
     * local minima.
     *
     * @param pattern is the pattern to tests
     * @return whether each bit, when flipped, returns to the original.
     */
    std::vector<bool> checkNeighbourStability(Pattern p, float tolerance);

    /**
     * Perform a single retrieval update on the AtomSpace
     *
     * @param spreadCycles is the number of times to spread importance per
     * retrieval cycle.
     */
    void updateAtomSpaceForRetrieval(int spreadCycles, Pattern originalPattern
            = Pattern(0,0));

    /**
     * Convert a vector of Number into a String representing a grid of outputs.
     *
     * @param p vector to translate.
     * @return string representing p.
     */
    template<typename Number> std::string patternToString(std::vector<Number> p) {
        std::stringstream ss;
        Number col = 0;

        typename std::vector<Number>::iterator it = p.begin();

        while (it != p.end()) {
            ss << *it << " ";
            col++;
            if (col == width) {
                ss << std::endl;
                col = 0;
            }
            ++it;
        }
        return ss.str();
    }

    Pattern getGridSTIAsPattern(bool blankKeys = true);
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
    std::vector<float> imprintAndTestPattern(Pattern p, int imprint, int retrieve, Pattern c, float mutate);

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
    void printLinks();
};

} // namespace opencog

#endif // _OPENCOG_HOPFIELD_SERVER_H
