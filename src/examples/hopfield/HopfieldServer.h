/**
 * HopfieldServer.h
 *
 * Emulates a hopfield network using OpenCog dynamics
 *
 * Author: Joel Pitt
 * Creation: Wed Mar 12 11:14:22 GMT+12 2008
 */

#ifndef HOPFIELDSERVER_H
#define HOPFIELDSERVER_H

#include <CogServer.h>
#include <RandGen.h>
#include <ImportanceUpdatingAgent.h>

#include <vector>

class HopfieldServer : public opencog::CogServer {

    private:
	
	/* Nodes in the Hopfield network can be referenced
	 * through HGrid.
	 */
	std::vector<Handle> hGrid;

	Util::RandGen* rng; 



    public:
	stim_t perceptStimUnit;
	stim_t stimForSpread;
	opencog::ImportanceUpdatingAgent *agent;

	int width, height, links;
	float density;
	
	AttentionValue::sti_t spreadThreshold;
	AttentionValue::sti_t vizThreshold;


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
	void updateAtomTableForRetrieval();

	/**
	 *
	 */
	void spreadImportance();

	void spreadAtomImportance(Handle h);

	template<typename Number> std::string patternToString(std::vector<Number> p)
	{
	    std::stringstream ss;
	    Number col = 0;

	    typename std::vector<Number>::iterator it = p.begin();

	    while(it != p.end()) {
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

	std::vector<int> binariseArray(std::vector<float>);
	std::vector<float> getGridAsFloatVector();
	std::vector<stim_t> getGridStimVector();

	void hebbianLearningUpdate();
	void resetNodes();
	float targetConjunction(std::vector<Handle> handles);
	float getNormSTI(AttentionValue::sti_t s);
	std::vector<Handle> moveSourceToFront(std::vector<Handle> outgoing);
	void imprintPattern(std::vector<int> pattern, int cycles);

	std::vector< std::vector<int> > generateRandomPatterns(int amount);
	std::vector< std::vector<int> > mutatePatterns( std::vector< std::vector<int> > patterns, float error);

	float hammingSimilarity(std::vector<int>,std::vector<int>);
	void doForgetting(float proportion);
	void addRandomLinks(int amount);

	void printStatus();
};

struct ImportanceSpreadSTISort
{
     bool operator()(const Handle& h1, const Handle& h2)
     {
          return TLB::getAtom(h1)->getAttentionValue().getSTI() > TLB::getAtom(h2)->getAttentionValue().getSTI();
     }
};

struct ImportanceSpreadLTIAndTVAscendingSort
{
    bool operator()(const Handle& h1, const Handle& h2)
    {
	AttentionValue::lti_t lti1, lti2;
	float tv1, tv2;

	tv1 = fabs(TLB::getAtom(h1)->getTruthValue().getMean());
	tv2 = fabs(TLB::getAtom(h2)->getTruthValue().getMean());

	lti1 = TLB::getAtom(h1)->getAttentionValue().getLTI();
	lti2 = TLB::getAtom(h2)->getAttentionValue().getLTI();

	if (lti1 < 0)
	    tv1 = lti1 * (1.0f - tv1);
	else
	    tv1 = lti1 * tv1;

	if (lti2 < 0)
	    tv2 = lti2 * (1.0f - tv2);
	else
	    tv2 = lti2 * tv2;

	 
	return tv1 < tv2;
    }

};

#endif // HOPFIELDSERVER_H
