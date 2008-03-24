/**
 * HopfieldServer.cc
 *
 * Emulates a hopfield network using OpenCog dynamics
 *
 * Author: Joel Pitt
 * Creation: Wed Mar 12 11:14:22 GMT+12 2008
 */

#include <sstream>
#include <math.h>

#include <mt19937ar.h>
#include <Logger.h>
#include <SimpleTruthValue.h>

#include "HopfieldServer.h"

using namespace opencog;
using namespace std;

#define HDEMO_DEFAULT_WIDTH 2
#define HDEMO_DEFAULT_HEIGHT 3
#define HDEMO_DEFAULT_LINKS 10 
#define HDEMO_DEFAULT_PERCEPT_STIM 10
#define HDEMO_DEFAULT_SPREAD_THRESHOLD 4
#define HDEMO_DEFAULT_VIZ_THRESHOLD 5
#define HDEMO_DEFAULT_SPREAD_STIM 1

#define HDEMO_LOG_LEVEL Util::Logger::FINE

void testHopfieldNetwork();
void testHopfieldNetworkRolling();
std::vector<int> testPattern(std::vector<int> p, int imprint, std::vector<int> c, int retrieve);

HopfieldServer hDemo;

int main(int argc, char *argv[])
{
    //int patternArray[] = { 0, 1, 0, 1, 1, 1, 0, 1, 0 };
    //std::vector<int> pattern1(patternArray, patternArray + 9);

    testHopfieldNetworkRolling();

}

void testHopfieldNetworkRolling()
{
    std::vector< std::vector<int> > patterns;
    std::vector< std::vector<int> > cuePatterns;
    std::vector< std::vector<int> > rPatterns;
    std::vector<int> result;

    MAIN_LOGGER.setLevel(HDEMO_LOG_LEVEL);
    MAIN_LOGGER.setPrintToStdoutFlag(true);
    MAIN_LOGGER.log(Util::Logger::INFO,"Init HopfieldServer");
	
    hDemo.init(HDEMO_DEFAULT_WIDTH, HDEMO_DEFAULT_HEIGHT, HDEMO_DEFAULT_LINKS);

    patterns = hDemo.generateRandomPatterns(2);
    cuePatterns = hDemo.mutatePatterns(patterns, 0.2);

    for (unsigned int i = 0; i< patterns.size(); i++) {
	hDemo.imprintPattern(patterns[i],15);
	MAIN_LOGGER.log(Util::Logger::INFO,"Encoded pattern and ran server for 5 loops");
    }

    for (unsigned int i = 0; i< patterns.size(); i++) {
	result = hDemo.retrievePattern(cuePatterns[i],5);
	MAIN_LOGGER.log(Util::Logger::INFO,"Updated Atom table for retrieval");
	rPatterns.push_back(result);
    }


    cout << "-----------------------" << endl;
    for (unsigned int i = 0; i< patterns.size(); i++) {
	float before, after, diff;

	cout << hDemo.patternToString(patterns[i]) << endl;
	cout << hDemo.patternToString(cuePatterns[i]) << endl;
	cout << hDemo.patternToString(rPatterns[i]) << endl;

	before = hDemo.hammingSimilarity(patterns[i], cuePatterns[i]); 
	after = hDemo.hammingSimilarity(patterns[i], rPatterns[i]);
	diff = after-before;
	cout << "=== similarity before/after/diff: " <<
	    before << "/" << after << "/" << diff << endl;
	cout << "-----------------------" << endl;
    }

}

void testHopfieldNetwork()
{
    std::vector< std::vector<int> > patterns;
    std::vector< std::vector<int> > cuePatterns;
    std::vector< std::vector<int> > rPatterns;

    MAIN_LOGGER.setLevel(HDEMO_LOG_LEVEL);
    MAIN_LOGGER.setPrintToStdoutFlag(true);
    MAIN_LOGGER.log(Util::Logger::INFO,"Init HopfieldServer");
	
    hDemo.init(HDEMO_DEFAULT_WIDTH, HDEMO_DEFAULT_HEIGHT, HDEMO_DEFAULT_LINKS);

    patterns = hDemo.generateRandomPatterns(1);
    cuePatterns = hDemo.mutatePatterns(patterns, 0.2);

    for (unsigned int i = 0; i< patterns.size(); i++) {
	rPatterns.push_back(testPattern(patterns[i],25,cuePatterns[i],10));
    }

    cout << "-----------------------" << endl;
    for (unsigned int i = 0; i< patterns.size(); i++) {
	float before, after, diff;

	cout << hDemo.patternToString(patterns[i]) << endl;
	cout << hDemo.patternToString(cuePatterns[i]) << endl;
	cout << hDemo.patternToString(rPatterns[i]) << endl;

	before = hDemo.hammingSimilarity(patterns[i], cuePatterns[i]); 
	after = hDemo.hammingSimilarity(patterns[i], rPatterns[i]);
	diff = after-before;
	cout << "=== similarity before/after/diff: " <<
	    before << "/" << after << "/" << diff << endl;
	cout << "-----------------------" << endl;
    }

}

std::vector<int> testPattern(std::vector<int> p, int imprint, std::vector<int> c, int retrieve)
{
    std::vector<int> result;

    hDemo.imprintPattern(p,imprint);
    MAIN_LOGGER.log(Util::Logger::INFO,"Encoded pattern and ran server for 5 loops");

    result = hDemo.retrievePattern(c,retrieve);
    MAIN_LOGGER.log(Util::Logger::INFO,"Updated Atom table for retrieval");
    //hDemo.printStatus();

    return result;
    
}

float HopfieldServer::hammingSimilarity(std::vector<int> a,std::vector<int> b)
{
    float diff = 0.0f;
    if (a.size() != b.size())
	return -1.0f;

    for (unsigned int i = 0; i < a.size(); i++) {
	if (a[i] != b[i]) diff++;
    }

    return 1.0f - (diff / a.size());

}

template <class T>
inline string to_string (const T& t)
{
    stringstream ss;
    ss << t;
    return ss.str();
}

HopfieldServer::HopfieldServer()
{
    perceptStimUnit = HDEMO_DEFAULT_PERCEPT_STIM;
    stimForSpread = HDEMO_DEFAULT_SPREAD_STIM;
    spreadThreshold = HDEMO_DEFAULT_SPREAD_THRESHOLD;
    vizThreshold = HDEMO_DEFAULT_VIZ_THRESHOLD;
    rng = new Util::MT19937RandGen(time(NULL));

    agent = new ImportanceUpdatingAgent();
    agent->getLogger()->enable();
    agent->getLogger()->setLevel(Util::Logger::FINE);
    agent->getLogger()->setPrintToStdoutFlag(true);
    plugInMindAgent(agent, 1);
}

HopfieldServer::~HopfieldServer()
{
    delete agent;
    delete rng;

}

void HopfieldServer::init(int width, int height, int numLinks)
{
    AtomSpace* atomSpace = getAtomSpace();
    string nodeName = "Hopfield_";
    this->width = width;
    this->height = height;

    // Create nodes
    for (int i=0; i < width; i++) {
	for (int j=0; j < height; j++) {
	    nodeName = "Hopfield_";
	    nodeName += to_string(i) + "_" + to_string(j);
	    Handle h = atomSpace->addNode(CONCEPT_NODE, nodeName.c_str());
	    // We don't want the forgetting process to remove
	    // the atoms perceiving the patterns
	    atomSpace->setVLTI(h,AttentionValue::NONDISPOSABLE);
	    hGrid.push_back(h);
	}
    }
    
    // If only 1 node, don't try and connect it
    if (hGrid.size() < 2) {
	MAIN_LOGGER.log(Util::Logger::WARNING,"Only one node, this is kind of silly but I shall follow instructions.");
	return;
    }

    addRandomLinks(numLinks);

}

void HopfieldServer::addRandomLinks(int amount)
{
    AtomSpace* atomSpace = getAtomSpace();

    // Link nodes randomly with numLinks links
    while (amount > 0) {
	int source, target;
	HandleSeq outgoing;

	source = rng->randint(hGrid.size());
	target = source;
	while (target == source) target = rng->randint(hGrid.size());

	outgoing.push_back(hGrid[target]);
	outgoing.push_back(hGrid[source]);
	atomSpace->addLink(SYMMETRIC_HEBBIAN_LINK, outgoing);

	amount--;
    }


}

void HopfieldServer::hebbianLearningUpdate()
{
    HandleEntry *links, *current_l;

    // tc affects the truthvalue
    float tc, old_tc;
    float tcDecayRate = 0.2;

    AtomSpace* a = getAtomSpace();
    MAIN_LOGGER.log(Util::Logger::DEBUG,"----------- Hebbian Learning update");

    // Go through all Hebbian links and update TVs
    links = a->getAtomTable().getHandleSet(SYMMETRIC_HEBBIAN_LINK, true);
    // TODO: process assymmetric hebbian links too

    // Add links if less than desired number and to replace forgotten links
    addRandomLinks(HDEMO_DEFAULT_LINKS - links->getSize());
    delete links;
    
    // get links again to include the new ones
    links = a->getAtomTable().getHandleSet(SYMMETRIC_HEBBIAN_LINK, true);

    for (current_l = links; current_l; current_l = current_l->next) {
	// for each hebbian link, find targets, work out conjunction and convert
	// that into truthvalue change. the change should be based on existing TV.
	Handle h;
	std::vector<Handle> outgoing;

	h = current_l->handle;

	// get out going set
	outgoing = TLB::getAtom(h)->getOutgoingSet();
	tc = targetConjunction(outgoing);
	// old link strength decays 
	old_tc = a->getTV(h).getMean();
	tc = (tcDecayRate * tc) + ( (1.0-tcDecayRate) * old_tc);
	// Update TV
	a->setMean(h,tc);
	
	MAIN_LOGGER.log(Util::Logger::FINE,"HebLearn: %s old tv %f", TLB::getAtom(h)->toString().c_str(), old_tc);
	
    }
    // if not enough links, try and create some more either randomly
    // or by looking at what nodes are in attentional focus

    delete links;

}

void HopfieldServer::resetNodes()
{
    AtomSpace* a = getAtomSpace();
    HandleEntry *nodes, *n;
    
    nodes = a->getAtomTable().getHandleSet(NODE, true);

    // Set all nodes to sti 0 and default LTI
    for (n = nodes; n; n = n->next) {
	a->setSTI(n->handle, 0);
	a->setLTI(n->handle, AttentionValue::DEFAULTATOMLTI);
    }
    delete nodes;

    MAIN_LOGGER.log(Util::Logger::DEBUG,"Nodes Reset");
}

float HopfieldServer::targetConjunction(std::vector<Handle> handles)
{
    // TODO: this won't work for Hebbian Links with arity > 2

    // indicates whether at least one of the source/target are
    // in the attentional focus
    bool inAttention = false;
    int normaliser;
    std::vector<Handle>::iterator h_i;
    Handle h;
    AtomSpace *a;
    AttentionValue::sti_t sti;
    float tc = 0.0f, normsti;
    std::vector<float> normsti_v;
    bool tcInit = true;

    a = getAtomSpace();
    // get normalizer (maxSTI - attention boundary)
    normaliser = a->getRecentMaxSTI() - a->getAttentionalFocusBoundary();

    for (h_i = handles.begin();
	    h_i != handles.end();
	    h_i++) {
	h = *h_i;
	sti = a->getSTI(h); 

	// if none in attention return 0 at end
	if (sti > a->getAttentionalFocusBoundary()) inAttention = true;

	// normalise each sti and multiple each
	normsti = (sti - a->getAttentionalFocusBoundary()) / (float) normaliser;

	// For debugging:
	normsti_v.push_back(normsti);

	if (tcInit) {
	    tc = normsti;
	    tcInit = false;
	} else tc *= normsti;

    }

    MAIN_LOGGER.log(Util::Logger::FINE,"TC: normaliser %d, normstis [%f,%f]", normaliser, normsti_v[0],normsti_v[1]);

    if (!inAttention) return 0.0f;
    
    // cap conjunction to range [-1,1]
    if (tc > 1.0f) return 1.0f;
    if (tc < -1.0f) return -1.0f;
    return tc;
	
}

void HopfieldServer::imprintPattern(std::vector<int> pattern, int cycles)
{
    bool first=true;

    // loop for number of imprinting cyles
    for (; cycles > 0; cycles--) {
        // for each encode pattern
	MAIN_LOGGER.log(Util::Logger::FINE,"---Encoding pattern");
	encodePattern(pattern);
	printStatus();
	// then update with learning
	// TODO: move below to separate function updateWithLearning();
	
	// ImportanceUpdating with links
	MAIN_LOGGER.log(Util::Logger::FINE,"---Running Importance update");
	agent->run(this);
	printStatus();
	
	hebbianLearningUpdate();

	spreadImportance();
	
	if (first) 
	    first = false;
	else
	    doForgetting(0.10);
	//----------
	printStatus();
	
	resetNodes();
    }

}

void HopfieldServer::doForgetting(float proportion = 0.10)
{
    HandleEntry *atoms;
    std::vector<Handle> atomsVector;
    int count = 0;
    int removalAmount;

    atoms = getAtomSpace()->getAtomTable().getHandleSet(ATOM,true);
    // Sort atoms by lti, remove the lowest unless vlti is NONDISPOSABLE
    //
    atoms->toHandleVector(atomsVector);
    std::sort(atomsVector.begin(), atomsVector.end(), ImportanceSpreadLTIAscendingSort());
    delete atoms;

    removalAmount = (int) (atomsVector.size() * proportion);

    for (unsigned int i = 0; i < atomsVector.size() &&
	    count < removalAmount; i++) {
	if (getAtomSpace()->getVLTI(atomsVector[i]) != AttentionValue::NONDISPOSABLE) {
	    cout << "Removing atom " <<  TLB::getAtom(atomsVector[i])->toString().c_str() << endl;
	    if (!getAtomSpace()->removeAtom(atomsVector[i])) {
		cout << "Error removing atom" << endl;
		return;
	    }
	    count++;
	}
    }
    cout << "-" << endl;

}

void HopfieldServer::encodePattern(std::vector<int> pattern)
{
    std::vector<Handle>::iterator it = hGrid.begin();
    std::vector<int>::iterator p_it = pattern.begin();
    
    while (it!=hGrid.end() && p_it != pattern.end()) {
	Handle h = (*it);
	int value = (*p_it);
	getAtomSpace()->stimulateAtom(h,perceptStimUnit * value);
	it++; p_it++;
    }

}

std::vector<int> HopfieldServer::retrievePattern(std::vector<int> partialPattern, int numCycles)
{
    std::string logString;
    
    logString += "Initialising " + to_string(numCycles) + " cycle pattern retrieval process.";
    MAIN_LOGGER.log(Util::Logger::INFO, logString.c_str());

    while (numCycles > 0) {
	encodePattern(partialPattern);
	printStatus();
	updateAtomTableForRetrieval();
	printStatus();
	updateAtomTableForRetrieval();
	printStatus();


	numCycles--;
	MAIN_LOGGER.log(Util::Logger::INFO, "Cycles left %d",numCycles);
    }
    
    logString = "Cue pattern: \n" + patternToString(partialPattern); 
    MAIN_LOGGER.log(Util::Logger::INFO, logString.c_str());

    MAIN_LOGGER.log(Util::Logger::INFO, "Pattern retrieval process ended.");
    
    return binariseArray(getGridAsFloatVector());
}

std::vector<int> HopfieldServer::binariseArray(std::vector<float> in)
{
    std::vector<int> out;
    std::vector<float>::iterator i;

    for (i = in.begin(); i != in.end(); i++) {
	float val = *i;
	out.push_back( (int) (val >= vizThreshold) );
    }
    
    return out;
}

std::vector<float> HopfieldServer::getGridAsFloatVector()
{
    std::vector<float> out;
    std::vector<Handle>::iterator i;

    for (i = hGrid.begin(); i != hGrid.end(); i++) {
	Handle h = *i;
	float val;
	val = getAtomSpace()->getSTI(h); // / getAtomSpace()->getRecentMaxSTI();
	out.push_back( val );
    }
    
    return out;
}

std::vector<stim_t> HopfieldServer::getGridStimVector()
{
    std::vector<stim_t> out;
    std::vector<Handle>::iterator i;

    for (i = hGrid.begin(); i != hGrid.end(); i++) {
	Handle h = *i;
	stim_t val;
	val = getAtomSpace()->getAtomStimulus(h); // / getAtomSpace()->getRecentMaxSTI();
	out.push_back( val );
    }
    
    return out;
}

void HopfieldServer::updateAtomTableForRetrieval()
{
//   run ImportanceUpdatingAgent once without updating links

    bool oldLinksFlag = agent->getUpdateLinksFlag();
    agent->setUpdateLinksFlag(false);
    
    MAIN_LOGGER.log(Util::Logger::INFO, "===updateAtomTableForRetreival===");
    MAIN_LOGGER.log(Util::Logger::INFO, "Running Importance updating agent");
    agent->run(this);

    MAIN_LOGGER.log(Util::Logger::INFO, "Spreading Importance");
    spreadImportance();

    agent->setUpdateLinksFlag(oldLinksFlag);

}


void HopfieldServer::spreadImportance()
{
// TODO: This should be a mind agent, convert later...

    AttentionValue::sti_t current;

    AtomSpace *a;
    std::vector<Handle> atoms;
    std::vector<Handle>::iterator hi;
    std::back_insert_iterator< std::vector<Handle> > out_hi(atoms);
    
    a = getAtomSpace();
    a->getHandleSet(out_hi,NODE,true);
    MAIN_LOGGER.log(Util::Logger::FINE, "---------- Spreading importance for atoms with threshold above %d", spreadThreshold);

    hi = atoms.begin();
    while (hi != atoms.end()) {
	Handle h = *hi;

	current = a->getSTI(h);
	/* spread if STI > spread threshold */
	if (current > spreadThreshold )
	    // spread fraction of importance to nodes it's linked to
	    spreadAtomImportance(h);
	
	hi++;
    }

    

}

void HopfieldServer::spreadAtomImportance(Handle h)
{
    HandleEntry *neighbours, *links, *he;
    AtomSpace *a;
    float maxTransferAmount,totalRelatedness;
    int totalTransferred;
    AttentionValue::sti_t importanceSpreadingQuantum = 10;
    float importanceSpreadingFactor = 0.4;

    totalRelatedness = 0.0f;
    totalTransferred = 0;

    a = getAtomSpace();
    
    MAIN_LOGGER.log(Util::Logger::FINE, "Spreading importance for atom %s", TLB::getAtom(h)->toString().c_str());

    neighbours = TLB::getAtom(h)->getNeighbors(true,true,SYMMETRIC_HEBBIAN_LINK);
    links = TLB::getAtom(h)->getIncomingSet()->clone();
    // TODO: process assymmetric hebbian links too, those that have h as a source
    links = HandleEntry::filterSet(links, SYMMETRIC_HEBBIAN_LINK, true);
    MAIN_LOGGER.log(Util::Logger::FINE, "Hebbian links found %d", links->getSize());
    
    maxTransferAmount = a->getSTI(h) * importanceSpreadingFactor;

    // sum total relatedness
    he = links;
    while (he) {
	float val;
	val = fabs(a->getTV(he->handle).toFloat());
	totalRelatedness += val;
	he = he->next;
    }

    if (totalRelatedness > 0.0f) {
	std::vector<Handle> linksVector;
	std::vector<Handle>::iterator linksVector_i;

	// Find order of links based on their STI
	// I.e. links with higher STI are more likely to get sti passed along
	// them before available sti for spreading runs out.
	links->toHandleVector(linksVector);
	std::sort(linksVector.begin(), linksVector.end(), ImportanceSpreadSTISort());

	for (linksVector_i = linksVector.begin();
		linksVector_i != linksVector.end() &&
		totalTransferred <= maxTransferAmount; linksVector_i++) {
	    double transferWeight, transferAmount;
	    std::vector<Handle> targets;
	    std::vector<Handle>::iterator t;
	    Handle lh = *linksVector_i;
	    const TruthValue &linkTV = a->getTV(lh);


	    targets = TLB::getAtom(lh)->getOutgoingSet();
	    transferWeight = linkTV.toFloat();

	    // amount to spread dependent on weight and quantum - needs to be
	    // divided by (targets->size - 1)
	    transferAmount = transferWeight * importanceSpreadingQuantum;
	    
	    // Allow at least one hebbian link to spread importance, even if it's
	    // less than the amount the weight would indicate
	    if (transferAmount > maxTransferAmount)
		transferAmount = maxTransferAmount;
	    
	    transferAmount = transferAmount / (targets.size()-1.0f);
	    if (transferAmount == 0.0f) continue;

	    MAIN_LOGGER.log(Util::Logger::FINE, "weight %f, quanta %d, size %d, Transfer amount %f, maxTransfer %f", transferWeight, importanceSpreadingQuantum, targets.size(), transferAmount, maxTransferAmount);

	    for (t = targets.begin();
		    t != targets.end() &&
		    totalTransferred + transferAmount <= maxTransferAmount;
		    t++) {
		Handle target_h = *t;

		// Then for each target of link (except source)...
		if ( TLB::getAtom(target_h) == TLB::getAtom(h) )
		    continue;

		// Check removing STI doesn't take node out of attentional
		// focus...
		// TODO: precalculate this in loop conditional
		if (a->getSTI(h) >= a->getAttentionalFocusBoundary() && \
		    a->getSTI(h) - transferAmount < a->getAttentionalFocusBoundary())
		    break;

		totalTransferred += (int) transferAmount;
		a->setSTI( h, a->getSTI(h) - (AttentionValue::sti_t) transferAmount );
		a->setSTI( target_h, a->getSTI(target_h) + (AttentionValue::sti_t) transferAmount );
		
		// stimulate link 
		if ( agent->getUpdateLinksFlag() )
		    a->stimulateAtom(lh,stimForSpread);
	    }


	}
    }
    else {
	MAIN_LOGGER.log(Util::Logger::FINE, "Total relatedness = 0, spreading nothing");
    }
    delete links;

}

void HopfieldServer::printStatus()
{
    // Print out Node STIs in grid pattern.
    // Also print out binarised version of grid.
    std::vector<float> nodeSTI = getGridAsFloatVector();
    std::vector<int> pattern = binariseArray(nodeSTI);
    std::vector<stim_t> nodeStim = getGridStimVector();
    HandleEntry *links, *current_l;

    int i;

    int col;

    for (i = 0; i<height; i++) {	
	for (col = 0; col < width; col++) {
	    cout << nodeStim[i*width + col] << " ";
	}
	cout << " |  ";
	for (col = 0; col < width; col++) {
	    cout << nodeSTI[i*width + col] << " ";
	}
	cout << " |  ";

	for (col = 0; col < width; col++) {
	    cout << pattern[i*width + col] << " ";
	}
	cout << endl;
    }
    
    //
    // Print out links.
    links = getAtomSpace()->getAtomTable().getHandleSet(SYMMETRIC_HEBBIAN_LINK, true);

    for (current_l = links; current_l; current_l = current_l->next) {
	Handle h = current_l->handle;

	cout << TLB::getAtom(h)->toString() << endl;
	
    }
    cout << "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=" <<endl;
}

std::vector< std::vector<int> > HopfieldServer::generateRandomPatterns(int amount)
{
    std::vector<std::vector<int> > patterns;

    for (; amount>0; amount--) {
	std::vector<int> p;
	for (int i = 0; i < width*height; i++) {
	    p.push_back(rng->randbool());
	}
	patterns.push_back(p);
    }
    return patterns;
}

std::vector<std::vector<int> > HopfieldServer::mutatePatterns(std::vector<std::vector<int> > patterns, float error)
{
    std::vector<std::vector<int> >::iterator i;
    std::vector<std::vector<int> > mutants;

    for (i = patterns.begin(); i != patterns.end(); i++) {
	std::vector<int> pattern; 
	std::vector<int>::iterator p;
	for (p = (*i).begin(); p != (*i).end(); p++) {
	    int val = *p;
	    // Flip bit with error probability
	    if (rng->randfloat() < error) { val = !val; }
	    pattern.push_back(val);
	}
	mutants.push_back(pattern);
    }
    return mutants;


}

