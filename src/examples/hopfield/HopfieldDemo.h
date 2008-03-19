/**
 * HopfieldDemo.h
 *
 * Emulates a hopfield network using OpenCog dynamics
 *
 * Author: Joel Pitt
 * Creation: Wed Mar 12 11:14:22 GMT+12 2008
 */

#ifndef HOPFIELDDEMO_H
#define HOPFIELDDEMO_H

#include <CogServer.h>
#include <RandGen.h>
#include <ImportanceUpdatingAgent.h>

#include <vector>

class HopfieldDemo : public opencog::CogServer {

    private:
	
	/* Nodes in the Hopfield network can be referenced
	 * through HGrid.
	 */
	std::vector<Handle> hGrid;

	Util::RandGen* rng; 

	stim_t perceptStimUnit;
	int width, height;

	opencog::ImportanceUpdatingAgent *agent;

	AttentionValue::sti_t spreadThreshold;

    public:

        ~HopfieldDemo();
        HopfieldDemo();

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
	 */
	void encodePattern(std::vector<int> pattern);

	/**
	 * Retrieve the the closest matching pattern in the network.
	 *
	 * @param pattern is the pattern to match
	 * @param numCycles is the number of lobe cycles to wait for retrieval
	 */
	std::vector<int> retrievePattern(std::vector<int> pattern, int numCycles);

	/**
	 *
	 */
	std::vector<int> updateAtomTableForRetrieval();

	/**
	 *
	 */
	void spreadImportance();

	void spreadAtomImportance(Handle h);

	std::string patternToString(std::vector<int> p);
};

#endif // HOPFIELDDEMO_H
