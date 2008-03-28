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
#include <getopt.h>
#include <sys/time.h>

#include <mt19937ar.h>
#include <Logger.h>
#include <SimpleTruthValue.h>
#include <Link.h>

#include "HopfieldServer.h"

using namespace opencog;
using namespace std;

#define HDEMO_DEFAULT_WIDTH 3
#define HDEMO_DEFAULT_HEIGHT 3
#define HDEMO_DEFAULT_LINKS 15 
#define HDEMO_DEFAULT_PERCEPT_STIM 10
#define HDEMO_DEFAULT_SPREAD_THRESHOLD 4
#define HDEMO_DEFAULT_VIZ_THRESHOLD 5
#define HDEMO_DEFAULT_SPREAD_STIM 1

void parseOptions(int argc, char *argv[]);
void printHelp();
void testHopfieldNetwork();
void testHopfieldNetworkRolling();
std::vector<int> testPattern(std::vector<int> p, int imprint, std::vector<int> c, int retrieve);
void printMatrixResult(std::vector< int > p1, std::vector< int > p2, std::vector< int > p3);

HopfieldServer hDemo;
int verboseFlag = 0;
int interleaveFlag = 0;
int showMatrixFlag = 0;
int totalFlag = 0;
int nPatterns = 1;
int retrieveCycles = 10;
int imprintCycles = 15;
float cueErrorRate = 0.1;
float importanceSpreadingMultiplier = 10.0f;

int main(int argc, char *argv[])
{
    //int patternArray[] = { 0, 1, 0, 1, 1, 1, 0, 1, 0 };
    //std::vector<int> pattern1(patternArray, patternArray + 9);
    parseOptions(argc, argv);

    /* setup logging */
    MAIN_LOGGER.setPrintToStdoutFlag(true);
    if (verboseFlag == 1) {
	MAIN_LOGGER.setLevel(Util::Logger::DEBUG);
    } else if (verboseFlag == 2) {
	MAIN_LOGGER.setLevel(Util::Logger::FINE);
    } else {
	MAIN_LOGGER.setLevel(Util::Logger::WARNING);
    }
	
    MAIN_LOGGER.log(Util::Logger::INFO,"Init HopfieldServer");
    hDemo.init(-1, -1, -1);

    testHopfieldNetworkRolling();

}

void printHelp()
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
"                  \t   numLinks = density * ( ( width * height ) - 1 )!\n"
"   -s --stimulus N\t Amount of stimulus to give an atom during imprinting or retrieval\n"
"   -t --spread-threshold N  The minimum threshold of atom STI before it gets spread.\n"
"   -z --viz-threshold N\t The atom STI needed for an atom to be considered \"on\" when\n"
"                  \t   compared to original stored pattern.\n"
"   -f --focus N \t Attentional focus boundary.\n"
"   -p --patterns N \t Number of patterns to test.\n"
"   -c --imprint N \t Number of _C_ycles to imprint pattern.\n"
"   -r --retrieve N \t Number of cycles to retrieve pattern for.\n"
"   -m --show-matrix \t Show matrices of stored/cue/retrieved pattern at end.\n"
"   -i --interleave \t Interleave training of each pattern.\n"
"   -e --error N \t probability of error in each bit of cue pattern.\n"
"   -o --total   \t Report the mean performance increase between cue and retrieval.\n"
"   -q --spread-multiplier N  multiplier for importance spread, if 0 then evenly\n"
"                  \t spread across links\n";
    cout << helpOutput;


}

void parseOptions(int argc, char *argv[])
{

    int c;

    while (1) {
	static const char *optString = "vDw:h:n:l:d:s:t:z:f:p:c:r:mie:oq:?";

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
	    {"patterns", no_argument, 0, 'p'}, // number of patterns to test
	    {"imprint", required_argument, 0, 'c'}, // # of imprint _c_ycles
	    {"retrieve", required_argument, 0, 'r'}, // # of retrieve iterations
	    {"show-matrix", 0, &showMatrixFlag, 1}, // show pattern/cue/result
	    {"interleave", 0, &interleaveFlag, 1}, // interleave imprint of each pattern
	    {"error", required_argument, 0, 'e'}, // cue error rate
	    {"total", 0, &totalFlag, 1}, // t_o_tal, reports mean, suitable for batch output
	    {"spread-multiplier", required_argument, 0, 'q'}, // multiplier for importance spread, if 0 then evenly spread across links
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
		hDemo.agent->getLogger()->enable();
		hDemo.agent->getLogger()->setLevel(Util::Logger::FINE);
		hDemo.agent->getLogger()->setPrintToStdoutFlag(true);
		break;
	    case 'w':
		hDemo.width=atoi(optarg); break;
	    case 'h':
		hDemo.height=atoi(optarg); break;
	    case 'n':
		hDemo.height=atoi(optarg);
		hDemo.width=atoi(optarg);
		break;
	    case 'l':
		hDemo.links=atoi(optarg); break;
	    case 'd':
		hDemo.density=atof(optarg); break;
	    case 's':
		hDemo.perceptStimUnit=atoi(optarg); break;
	    case 't':
		hDemo.spreadThreshold=atoi(optarg); break;
	    case 'z':
		hDemo.vizThreshold=atoi(optarg); break;
	    case 'f':
		AttentionValue::sti_t f;
		f = (AttentionValue::sti_t) atoi(optarg);
		hDemo.getAtomSpace()->setAttentionalFocusBoundary(f);
		break;
	    case 'p':
		nPatterns = atoi(optarg); break;
	    case 'c':
		imprintCycles = atoi(optarg); break;
	    case 'r':
		retrieveCycles = atoi(optarg); break;
	    case 'e':
		cueErrorRate = atof(optarg); break;
	    case 'm':
		showMatrixFlag = 1; break;
	    case 'i':
		interleaveFlag = 1; break;
	    case 'o':
		totalFlag = 1; break;
	    case 'q':
		importanceSpreadingMultiplier = atof(optarg); break;


	    case '?':
		printHelp();
		exit(0);
		break;
	    default:
		break;
	}
    }

}

void testHopfieldNetworkRolling()
{
    std::vector< std::vector<int> > patterns;
    std::vector< std::vector<int> > cuePatterns;
    std::vector< std::vector<int> > rPatterns;
    std::vector<int> result;
    std::vector<float> diffs;

    patterns = hDemo.generateRandomPatterns(nPatterns);
    cuePatterns = hDemo.mutatePatterns(patterns, cueErrorRate);

    for (unsigned int i = 0; i< patterns.size(); i++) {
	hDemo.imprintPattern(patterns[i],imprintCycles);
	MAIN_LOGGER.log(Util::Logger::INFO,"Encoded pattern and ran server for %d loops",imprintCycles);
    }

    for (unsigned int i = 0; i< patterns.size(); i++) {
	result = hDemo.retrievePattern(cuePatterns[i],retrieveCycles);
	MAIN_LOGGER.log(Util::Logger::INFO,"Updated Atom table for retrieval");
	rPatterns.push_back(result);
    }

    if (! totalFlag) cout << "-----------------------" << endl;
    for (unsigned int i = 0; i< patterns.size(); i++) {
	float before, after, diff;

	if (showMatrixFlag)
	    printMatrixResult(patterns[i],cuePatterns[i],rPatterns[i]);

	before = hDemo.hammingSimilarity(patterns[i], cuePatterns[i]); 
	after = hDemo.hammingSimilarity(patterns[i], rPatterns[i]);
	diff = after-before;
	diffs.push_back(diff);
	
	if (! totalFlag) cout << "=== similarity before/after/diff: " <<
	    before << "/" << after << "/" << diff << endl;
    }

    if (totalFlag) {
	float total = 0.0f;
	for (unsigned int i = 0; i < diffs.size(); i++)
	    total += diffs[i];
	cout << total / diffs.size() << endl;
	
    }

}

void printMatrixResult(std::vector< int > p1, std::vector< int > p2, std::vector< int > p3) {
    int i, col;

    for (i = 0; i<hDemo.height; i++) {	
	for (col = 0; col < hDemo.width; col++) {
	    printf("%2d", p1[i*hDemo.width + col]);
	}
	cout << " | ";
	for (col = 0; col < hDemo.width; col++) {
	    printf("%2d", p2[i*hDemo.width + col]);
	}
	cout << " | ";

	for (col = 0; col < hDemo.width; col++) {
	    printf("%2d", p3[i*hDemo.width + col]);
	}
	cout << endl;
    }
    
}

void testHopfieldNetwork()
{
    std::vector< std::vector<int> > patterns;
    std::vector< std::vector<int> > cuePatterns;
    std::vector< std::vector<int> > rPatterns;

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
    struct timeval tv;
    struct timezone tz;
    struct tm *tm;
    gettimeofday(&tv, &tz);
    tm=localtime(&tv.tv_sec);

    perceptStimUnit = HDEMO_DEFAULT_PERCEPT_STIM;
    stimForSpread = HDEMO_DEFAULT_SPREAD_STIM;
    spreadThreshold = HDEMO_DEFAULT_SPREAD_THRESHOLD;
    vizThreshold = HDEMO_DEFAULT_VIZ_THRESHOLD;
    width = HDEMO_DEFAULT_WIDTH;
    height = HDEMO_DEFAULT_HEIGHT;
    links = HDEMO_DEFAULT_LINKS;
    density = -1.0f;
    rng = new Util::MT19937RandGen(tv.tv_usec);

    agent = new ImportanceUpdatingAgent();
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
    if (width > 0) this->width = width;
    if (height > 0)this->height = height;

    if (numLinks > 0)
	this->links = numLinks;
    else if (density != -1.0f)
	this->links = (int) (density * (this->width * this->height));
    

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

    addRandomLinks(this->links);

}

void HopfieldServer::addRandomLinks(int amount)
{
    AtomSpace* atomSpace = getAtomSpace();

    // Link nodes randomly with numLinks links
    while (amount > 0) {
	int source, target;
	HandleSeq outgoing;
	HandleEntry* he;

	source = rng->randint(hGrid.size());
	target = source;

	while (target == source) target = rng->randint(hGrid.size());

	outgoing.push_back(hGrid[source]);
	outgoing.push_back(hGrid[target]);
	he = atomSpace->getAtomTable().getHandleSet(outgoing, (Type*) NULL, (bool*) NULL, outgoing.size(), SYMMETRIC_HEBBIAN_LINK, false);
	if (he) {
	    delete he;
	    continue;
	}
	atomSpace->addLink(SYMMETRIC_HEBBIAN_LINK, outgoing);

	amount--;
    }


}

void HopfieldServer::hebbianLearningUpdate()
{
    HandleEntry *links, *current_l;

    // tc affects the truthvalue
    float tc, old_tc, new_tc;
    float tcDecayRate = 0.2;

    AtomSpace* a = getAtomSpace();
    MAIN_LOGGER.log(Util::Logger::DEBUG,"----------- Hebbian Learning update");

    // Go through all Hebbian links and update TVs
    links = a->getAtomTable().getHandleSet(HEBBIAN_LINK, true);
    // TODO: process assymmetric hebbian links too

    // Add links if less than desired number and to replace forgotten links
    addRandomLinks(HDEMO_DEFAULT_LINKS - links->getSize());
    delete links;
    
    // get links again to include the new ones
    links = a->getAtomTable().getHandleSet(HEBBIAN_LINK, true);

    for (current_l = links; current_l; current_l = current_l->next) {
	// for each hebbian link, find targets, work out conjunction and convert
	// that into truthvalue change. the change should be based on existing TV.
	Handle h;
	std::vector<Handle> outgoing;

	h = current_l->handle;

	// get out going set
	outgoing = TLB::getAtom(h)->getOutgoingSet();
	new_tc = targetConjunction(outgoing);

	// old link strength decays 
	old_tc = a->getTV(h).getMean();
	tc = (tcDecayRate * new_tc) + ( (1.0-tcDecayRate) * old_tc);
	if (tc < 0.0f && new_tc < 0.0f ) {
	    Handle source = outgoing[0];
	    AttentionValue::sti_t s_sti = a->getSTI(source);

	    // check link type. if symmetric, delete and replace with
	    // asymmetric.
	    // if asymmetric then check link goes from +ve to -ve atom
	    // and fix if it doesn't
	    if (TLB::getAtom(h)->getType() == SYMMETRIC_HEBBIAN_LINK ||
		    (TLB::getAtom(h)->getType() == ASYMMETRIC_HEBBIAN_LINK
		     && getNormSTI(s_sti) < 0) ) {

		a->removeAtom(h);
		outgoing = moveSourceToFront(outgoing);
		// add asymmetric link
		// if it already exists then the truth values will be mergerd.
		h = a->addLink(ASYMMETRIC_HEBBIAN_LINK, outgoing, SimpleTruthValue(tc, 1));
	    } 
	} else {
	    // Update TV
	    a->setMean(h,tc);
	}
	
	MAIN_LOGGER.log(Util::Logger::FINE,"HebLearn: %s old tv %f", TLB::getAtom(h)->toString().c_str(), old_tc);
	
    }
    // if not enough links, try and create some more either randomly
    // or by looking at what nodes are in attentional focus

    delete links;

}

std::vector<Handle> HopfieldServer::moveSourceToFront(std::vector<Handle> outgoing)
{
    // find source, atom with negative norm_sti, and make first in outgoing
    std::vector<Handle>::iterator outgoing_i;
    AtomSpace *a = getAtomSpace();
    Handle theSource = NULL;
    for (outgoing_i = outgoing.begin(); outgoing_i < outgoing.end();) {
	float normsti;
	Handle oh = *outgoing_i;
	normsti = getNormSTI(a->getSTI(oh));
	if (normsti > 0.0f) {
	    theSource = oh;
	    outgoing_i = outgoing.erase(outgoing_i);
	} else
	    outgoing_i++;
    }
    if (theSource) {
	outgoing.insert(outgoing.begin(),theSource);
    } else {
	MAIN_LOGGER.log(Util::Logger::ERROR,"Can't find source atom for new Asymmetric Hebbian Link");
    }
    return outgoing;

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
    std::vector<Handle>::iterator h_i;
    Handle h;
    AtomSpace *a;
    AttentionValue::sti_t sti;
    float tc = 0.0f, normsti;
    std::vector<float> normsti_v;
    bool tcInit = true;

    a = getAtomSpace();

    for (h_i = handles.begin();
	    h_i != handles.end();
	    h_i++) {
	h = *h_i;
	sti = a->getSTI(h); 

	// if none in attention return 0 at end
	if (sti > a->getAttentionalFocusBoundary()) inAttention = true;

	// normalise each sti and multiple each
	normsti = getNormSTI(sti);

	// For debugging:
	normsti_v.push_back(normsti);

	if (tcInit) {
	    tc = normsti;
	    tcInit = false;
	} else tc *= normsti;

    }

    MAIN_LOGGER.log(Util::Logger::FINE,"TC: normstis [%f,%f]", normsti_v[0],normsti_v[1]);

    if (!inAttention) return 0.0f;
    
    // cap conjunction to range [-1,1]
    if (tc > 1.0f) return 1.0f;
    if (tc < -1.0f) return -1.0f;
    return tc;
	
}

float HopfieldServer::getNormSTI(AttentionValue::sti_t s)
{
    // get normalizer (maxSTI - attention boundary)
    AtomSpace *a = getAtomSpace();
    int normaliser = a->getRecentMaxSTI() - a->getAttentionalFocusBoundary();
    return (s - a->getAttentionalFocusBoundary()) / (float) normaliser;
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
    std::sort(atomsVector.begin(), atomsVector.end(), ImportanceSpreadLTIAndTVAscendingSort());
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

    resetNodes();

    while (numCycles > 0) {
	encodePattern(partialPattern);
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
    HandleEntry *links, *he;
    AtomSpace *a;
    float maxTransferAmount,totalRelatedness;
    int totalTransferred;
    float importanceSpreadingFactor = 0.4;

    totalRelatedness = 0.0f;
    totalTransferred = 0;

    a = getAtomSpace();
    
    MAIN_LOGGER.log(Util::Logger::FINE, "+Spreading importance for atom %s", TLB::getAtom(h)->toString().c_str());

    links = TLB::getAtom(h)->getIncomingSet()->clone();
    links = HandleEntry::filterSet(links, HEBBIAN_LINK, true);
    MAIN_LOGGER.log(Util::Logger::FINE, "  +Hebbian links found %d", links->getSize());
    
    maxTransferAmount = a->getSTI(h) * importanceSpreadingFactor;

    // sum total relatedness
    he = links;
    while (he) {
	if (((Link*)TLB::getAtom(he->handle))->isSource(h)) {
	    float val;
	    val = fabs(a->getTV(he->handle).toFloat());
	    totalRelatedness += val;
	}
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

	    if (!((Link*)TLB::getAtom(lh))->isSource(h)) {
		//MAIN_LOGGER.log(Util::Logger::FINE, "Link %s does not have this atom as a source.", TLB::getAtom(lh)->toString().c_str() );
		continue; 
	    }

	    targets = TLB::getAtom(lh)->getOutgoingSet();
	    transferWeight = linkTV.toFloat();

	    // amount to spread dependent on weight and multiplier - needs to be
	    // divided by (targets->size - 1)
	    if (importanceSpreadingMultiplier == 0.0f) {
		transferAmount = transferWeight * (maxTransferAmount / totalRelatedness);
	    } else
		transferAmount = transferWeight * importanceSpreadingMultiplier;

	    // Allow at least one hebbian link to spread importance, even if it's
	    // less than the amount the weight would indicate
	    if (transferAmount > maxTransferAmount)
		transferAmount = maxTransferAmount;
	    
	    if (targets.size() != 2)
		transferAmount = transferAmount / (targets.size()-1.0f);
	    if (transferAmount == 0.0f) continue;

	    MAIN_LOGGER.log(Util::Logger::FINE, "  +Link %s", TLB::getAtom(lh)->toString().c_str() );
	    MAIN_LOGGER.log(Util::Logger::FINE, "    |weight %f, quanta %.2f, size %d, Transfer amount %f, maxTransfer %f", transferWeight, importanceSpreadingMultiplier, targets.size(), transferAmount, maxTransferAmount);

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
		MAIN_LOGGER.log(Util::Logger::FINE, "    |%d sti from %s to %s", (int) transferAmount, TLB::getAtom(h)->toString().c_str(), TLB::getAtom(target_h)->toString().c_str() );
		
		// stimulate link 
		// Doesn't make sense to stimulate the links just for spread.
		// Maybe stimulate links that are helpful in getting the right
		// pattern.
		//if ( agent->getUpdateLinksFlag() )
		//    a->stimulateAtom(lh,stimForSpread);
	    }


	}
    }
    else {
	MAIN_LOGGER.log(Util::Logger::FINE, "  |Total relatedness = 0, spreading nothing");
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
    if (!verboseFlag) return;

    for (i = 0; i<height; i++) {	
	for (col = 0; col < width; col++) {
	    printf("%3d", nodeStim[i*width + col]);
	}
	cout << " | ";
	for (col = 0; col < width; col++) {
	    printf("%4.0f", nodeSTI[i*width + col]);
	}
	cout << " | ";

	for (col = 0; col < width; col++) {
	    printf("%3d", pattern[i*width + col]);
	}
	cout << endl;
    }
    
    //
    // Print out links.
    links = getAtomSpace()->getAtomTable().getHandleSet(HEBBIAN_LINK, true);

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

