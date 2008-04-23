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
#include <sys/time.h>

#include <mt19937ar.h>
#include <Logger.h>
#include <Link.h>

#include "HopfieldServer.h"

using namespace opencog;
using namespace std;

std::vector<float> HopfieldServer::imprintAndTestPattern(Pattern p, int imprint, int retrieve=10, float mutate=0.0f)
{
    std::vector<float> result;
    std::vector<int> rPattern;

    for (int i = 0; i < imprint; i++) {

	result.push_back(singleImprintAndTestPattern(p,retrieve,mutate));
	if (!options->verboseFlag) cout << ".";

    }

    return result;
    
}

float HopfieldServer::singleImprintAndTestPattern(Pattern p, int retrieve=10, float mutate=0.0f)
{
    float result;
    float before;
    Pattern rPattern(width,height);
    Pattern c(width,height);

    imprintPattern(p,1);
    MAIN_LOGGER.log(Util::Logger::FINE,"Encoded pattern for 1 loop");

    c = p.mutatePattern(mutate);
    MAIN_LOGGER.log(Util::Logger::FINE,"Mutated pattern");
    if (options->recordToFile) {
	before = p.hammingSimilarity(c); 
	options->beforeFile << before << ", ";
    }
    
    rPattern = retrievePattern(c,retrieve);
    result = p.hammingSimilarity(rPattern);
    if (options->recordToFile) {
	options->afterFile << result << ", ";
	options->diffFile << (result - before) << ", ";
    }
    // Nodes are left with STI after retrieval
    resetNodes();

    MAIN_LOGGER.log(Util::Logger::FINE,"Retrieved pattern");

    return result;
    
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
    struct timeval tv;
    struct timezone tz;
    struct tm *tm;
    gettimeofday(&tv, &tz);
    tm=localtime(&tv.tv_sec);

    perceptStimUnit = HDEMO_DEFAULT_PERCEPT_STIM;
    imprintStimUnit = HDEMO_DEFAULT_IMPRINT_STIM;
    width = HDEMO_DEFAULT_WIDTH;
    height = HDEMO_DEFAULT_HEIGHT;
    links = HDEMO_DEFAULT_LINKS;
    density = -1.0f;
    rng = new Util::MT19937RandGen(tv.tv_usec);
    options = new HopfieldOptions();
    options->setServer(this);

    importUpdateAgent = new ImportanceUpdatingAgent();
    hebLearnAgent = new HebbianLearningAgent();
    spreadAgent = new ImportanceSpreadingAgent();
    plugInMindAgent(importUpdateAgent, 1);
    plugInMindAgent(hebLearnAgent, 1);
}

HopfieldServer::~HopfieldServer()
{
    delete importUpdateAgent;
    delete hebLearnAgent;
    delete rng;

}

void HopfieldServer::init(int width, int height, int numLinks)
{
    AtomSpace* atomSpace = getAtomSpace();
    string nodeName = "Hopfield_";
    if (width > 0) {
	this->width = width;
    }
    if (height > 0) {
	this->height = height;
    }

    if (numLinks > 0)
	this->links = numLinks;
    else if (density != -1.0f) {
	int maxLinks = 0, n;
	n = this->width * this->height - 1;
	maxLinks = n * (n+1) / 2;
	this->links = (int) (density * maxLinks);
	MAIN_LOGGER.log(Util::Logger::INFO ,"Density of %.2f gives %d links out of %d", density, this->links, maxLinks);
    }

    // Tune mind agents from command line options
    spreadAgent->setSpreadThreshold(options->spreadThreshold);
    spreadAgent->setImportanceSpreadingMultiplier(options->importanceSpreadingMultiplier);
    

    // Create nodes
    for (int i=0; i < this->width; i++) {
	for (int j=0; j < this->height; j++) {
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
	MAIN_LOGGER.log(Util::Logger::WARNING,"Only %d node(s), <2 so this is kind of silly but I shall follow instructions.", hGrid.size());
	return;
    }

    addRandomLinks();

}

void HopfieldServer::addRandomLinks()
{
    AtomSpace* atomSpace = getAtomSpace();
    HandleEntry *links;
    int amount;
    
    // Go through all Hebbian links and update TVs
    links = atomSpace->getAtomTable().getHandleSet(HEBBIAN_LINK, true);
    // TODO: process assymmetric hebbian links too

    // Add links if less than desired number and to replace forgotten links
    amount = this->links - links->getSize();
    delete links;

    MAIN_LOGGER.log(Util::Logger::FINE,"Adding %d random Hebbian Links.", amount);
    // Link nodes randomly with amount links
    while (amount > 0) {
	int source, target;
	HandleSeq outgoing;
	HandleEntry* he;

	source = rng->randint(hGrid.size());
	target = rng->randint(hGrid.size()-1);
	if (target >= source) target++;

	outgoing.push_back(hGrid[source]);
	outgoing.push_back(hGrid[target]);
	he = atomSpace->getAtomTable().getHandleSet(outgoing, (Type*) NULL, (bool*) NULL, outgoing.size(), SYMMETRIC_HEBBIAN_LINK, false);
	if (he) {
	    //MAIN_LOGGER.log(Util::Logger::FINE,"Trying to add %d -> %d, but already exists %s", source, target, TLB::getAtom(he->handle)->toString().c_str());
	    delete he;
	} else {
	    atomSpace->addLink(SYMMETRIC_HEBBIAN_LINK, outgoing);
	    amount--;
	}
    }


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

void HopfieldServer::imprintPattern(Pattern pattern, int cycles)
{
    bool first=true;

    // loop for number of imprinting cyles
    for (; cycles > 0; cycles--) {
        // for each encode pattern
	MAIN_LOGGER.log(Util::Logger::FINE,"---Encoding pattern");
	encodePattern(pattern,imprintStimUnit);
	printStatus();
	// then update with learning
	// TODO: move below to separate function updateWithLearning();
	
	// ImportanceUpdating with links
	MAIN_LOGGER.log(Util::Logger::FINE,"---Running Importance update");
	importUpdateAgent->run(this);
	printStatus();
	
	addRandomLinks();

	hebLearnAgent->run(this);

	spreadAgent->run(this);
	
	if (first) 
	    first = false;
	else
	    doForgetting(0.10);
	//----------
	printStatus();
	
	resetNodes();
    }

}

void HopfieldServer::doForgetting(float proportion = 0.05)
{
    HandleEntry *atoms;
    std::vector<Handle> atomsVector;
    int count = 0;
    int removalAmount;

    atoms = getAtomSpace()->getAtomTable().getHandleSet(ATOM,true);
    // Sort atoms by lti, remove the lowest unless vlti is NONDISPOSABLE
    //
    atoms->toHandleVector(atomsVector);
    std::sort(atomsVector.begin(), atomsVector.end(), ImportanceSpreadLTIThenTVAscendingSort());
    delete atoms;

    removalAmount = (int) (atomsVector.size() * proportion);

    for (unsigned int i = 0; i < atomsVector.size() ; i++) {
	if (getAtomSpace()->getVLTI(atomsVector[i]) != AttentionValue::NONDISPOSABLE && count < removalAmount) {
	    //cout << "Removing atom " <<  TLB::getAtom(atomsVector[i])->toString().c_str() << endl;
	    MAIN_LOGGER.log(Util::Logger::FINE,"Removing atom %s", TLB::getAtom(atomsVector[i])->toString().c_str());
	    if (!getAtomSpace()->removeAtom(atomsVector[i])) {
		cout << "Error removing atom" << endl;
		return;
	    }
	    count++;
	} else
	    MAIN_LOGGER.log(Util::Logger::FINE,"Not removing atom %s", TLB::getAtom(atomsVector[i])->toString().c_str());
    }

}

void HopfieldServer::encodePattern(Pattern pattern,stim_t stimulus)
{
    std::vector<Handle>::iterator it = hGrid.begin();
    std::vector<int>::iterator p_it = pattern.begin();
    
    while (it!=hGrid.end() && p_it != pattern.end()) {
	Handle h = (*it);
	int value = (*p_it);
	getAtomSpace()->stimulateAtom(h,stimulus * value);
	it++; p_it++;
    }

}

Pattern HopfieldServer::retrievePattern(Pattern partialPattern, int numCycles)
{
    std::string logString;
    
    logString += "Initialising " + to_string(numCycles) + " cycle pattern retrieval process.";
    MAIN_LOGGER.log(Util::Logger::INFO, logString.c_str());

    resetNodes();

    while (numCycles > 0) {
	encodePattern(partialPattern,perceptStimUnit);
	printStatus();
	updateAtomTableForRetrieval(5);
	printStatus();

	numCycles--;
	MAIN_LOGGER.log(Util::Logger::INFO, "Cycles left %d",numCycles);
    }
    
    logString = "Cue pattern: \n" + patternToString(partialPattern); 
    MAIN_LOGGER.log(Util::Logger::INFO, logString.c_str());

    MAIN_LOGGER.log(Util::Logger::INFO, "Pattern retrieval process ended.");
    
    return getGridSTIAsPattern().binarisePattern(options->vizThreshold);
}

Pattern HopfieldServer::getGridSTIAsPattern()
{
    Pattern out(width, height);
    Pattern::iterator out_i = out.begin();
    std::vector<Handle>::iterator i;

    for (i = hGrid.begin(); i != hGrid.end(); i++) {
	Handle h = *i;
	AttentionValue::sti_t val;
	val = getAtomSpace()->getSTI(h); // / getAtomSpace()->getRecentMaxSTI();
	*out_i = val;
	out_i++;
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

void HopfieldServer::updateAtomTableForRetrieval(int spreadCycles = 1)
{
//   run ImportanceUpdatingAgent once without updating links

    bool oldLinksFlag = importUpdateAgent->getUpdateLinksFlag();
    importUpdateAgent->setUpdateLinksFlag(false);
    
    MAIN_LOGGER.log(Util::Logger::INFO, "===updateAtomTableForRetreival===");
    MAIN_LOGGER.log(Util::Logger::INFO, "Running Importance updating agent");
    importUpdateAgent->run(this);

    MAIN_LOGGER.log(Util::Logger::INFO, "Spreading Importance %d times", spreadCycles);
    for (int i = 0; i< spreadCycles; i++) {
	MAIN_LOGGER.log(Util::Logger::FINE, "Spreading Importance - cycle %d", i);
	spreadAgent->run(this);
    }


    importUpdateAgent->setUpdateLinksFlag(oldLinksFlag);

}

void HopfieldServer::printStatus()
{
    // Print out Node STIs in grid pattern.
    // Also print out binarised version of grid.
    Pattern nodeSTI = getGridSTIAsPattern();
    Pattern pattern = nodeSTI.binarisePattern(options->vizThreshold);
    std::vector<stim_t> nodeStim = getGridStimVector();
    HandleEntry *links, *current_l;

    int i;

    int col;
    if (!options->verboseFlag) return;

    for (i = 0; i<height; i++) {	
	for (col = 0; col < width; col++) {
	    printf("%3d", nodeStim[i*width + col]);
	}
	cout << " | ";
	for (col = 0; col < width; col++) {
	    printf("%3d", nodeSTI[i*width + col]);
	}
	cout << " | ";

	for (col = 0; col < width; col++) {
	    printf("%3d", pattern[i*width + col]);
	}
	cout << endl;
    }
    
    //
    // Print out links.
    if (options->verboseFlag > 1) {
	links = getAtomSpace()->getAtomTable().getHandleSet(HEBBIAN_LINK, true);

	for (current_l = links; current_l; current_l = current_l->next) {
	    Handle h = current_l->handle;

	    cout << TLB::getAtom(h)->toString() << endl;
	    
	}
    }
    cout << "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=" <<endl;
}

std::string HopfieldServer::printMatrixResult(std::vector< Pattern > patterns) {
    int i, col;

    for (i = 0; i<height; i++) {	
	for (unsigned int j = 0; j < patterns.size(); j++) {
	    Pattern current = patterns[j];
	    for (col = 0; col < width; col++) {
		printf("%2d", current[i*width + col]);
	    }
	    if (j != (patterns.size() - 1)) cout << " | ";
	}
	cout << endl;
    }
    return std::string();
    
}

