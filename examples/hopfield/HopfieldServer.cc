/*
 * examples/hopfield/HopfieldServer.cc
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

#include "HopfieldServer.h"

#include <sstream>
#include <iomanip>

#include <math.h>
#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/time.h>
#endif

#include <opencog/atomspace/Link.h>
#include <opencog/dynamics/attention/ImportanceUpdatingAgent.h>
#include <opencog/util/Logger.h>
#include <opencog/util/platform.h>
#include <opencog/util/mt19937ar.h>

#include "HopfieldOptions.h"
#include "StorkeyAgent.h"

using namespace opencog;
using namespace std;

// factory method
BaseServer* HopfieldServer::derivedCreateInstance()
{
    logger().debug("[HopfieldServer] createInstance");
    return new HopfieldServer();
}

std::vector<float> HopfieldServer::imprintAndTestPattern(Pattern p, int imprint, int retrieve, Pattern cue, float mutate = 0.0f)
{
    std::vector<float> result;
    std::vector<int> rPattern;

    for (int i = 0; i < imprint; i++) {

        float iResult;
        if (options->cueGenerateOnce) {
            iResult = singleImprintAndTestPattern(p, retrieve, mutate, cue);
        } else {
            cue = p.mutatePattern(mutate);
            iResult = singleImprintAndTestPattern(p, retrieve, mutate, cue);
        }

        result.push_back(iResult);
        if (!options->verboseLevel) cout << "." << flush;
        if (i < imprint - 1 && options->recordToFile) {
            options->beforeFile << ", ";
            options->afterFile << ", ";
            options->diffFile << ", ";
        }

    }

    return result;

}

float HopfieldServer::singleImprintAndTestPattern(Pattern p, int retrieve = 1, float mutate = 0.0f, Pattern c = Pattern(0, 0))
{
    float result;
    float before = 0.0;
    Pattern rPattern(width, height);

    imprintPattern(p, 1);
    logger().fine("Encoded pattern for 1 loop");

    if (! options->cueGenerateOnce) {
        c = p.mutatePattern(mutate);
        logger().fine("Mutated pattern");
    }

    if (options->recordToFile) {
        before = p.hammingSimilarity(c);
        options->beforeFile << before;
    }

    rPattern = retrievePattern(c, retrieve, options->spreadCycles);
    result = p.hammingSimilarity(rPattern);
    if (options->recordToFile) {
        options->afterFile << result;
        options->diffFile << (result - before);
    }
    // Nodes are left with STI after retrieval
    resetNodes();

    logger().fine("Retrieved pattern");

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
    time_t t = tv.tv_sec;
    tm = localtime(&t);

	patternStimulus = HDEMO_DEFAULT_PATTERN_STIM;
    width = HDEMO_DEFAULT_WIDTH;
    height = HDEMO_DEFAULT_HEIGHT;
    links = HDEMO_DEFAULT_LINKS;
    density = -1.0f;
    rng = new MT19937RandGen(tv.tv_usec);
    options = new HopfieldOptions();
    options->setServer(this);
    hebLearnAgent = NULL;
    storkeyAgent = NULL;

}

HopfieldServer::~HopfieldServer()
{
    unloadModule("libattention.so");
    //delete importUpdateAgent;
    //delete hebLearnAgent;
    delete rng;
}

void HopfieldServer::init(int width, int height, int numLinks)
{
    loadModule("libattention.so");

    //CogServer& cogserver = static_cast<CogServer&>(server());
    importUpdateAgent = static_cast<ImportanceUpdatingAgent*>(this->createAgent(ImportanceUpdatingAgent::info().id, true));
    if (options->updateMethod == HopfieldOptions::CONJUNCTION) {
        hebLearnAgent     = static_cast<HebbianLearningAgent*>(this->createAgent(HebbianLearningAgent::info().id, true));
    } else {
        storkeyAgent = new StorkeyAgent();
    }
#ifdef HAVE_GSL
    diffuseAgent      = static_cast<ImportanceDiffusionAgent*>(this->createAgent(ImportanceDiffusionAgent::info().id, true));
#else
    spreadAgent       = static_cast<ImportanceSpreadingAgent*>(this->createAgent(ImportanceSpreadingAgent::info().id, true));
#endif
    forgetAgent       = static_cast<ForgettingAgent*>(this->createAgent(ForgettingAgent::info().id, true));

    if (options->verboseLevel) {
        importUpdateAgent->getLogger()->enable();
        importUpdateAgent->getLogger()->setPrintToStdoutFlag (true);
        if (hebLearnAgent) {
            hebLearnAgent->getLogger()->enable();
            hebLearnAgent->getLogger()->setPrintToStdoutFlag (true);
        } else {
            storkeyAgent->getLogger()->enable();
            storkeyAgent->getLogger()->setPrintToStdoutFlag (true);
        }
//! @todo make all attention modules use their own logger object or upgrade
//! logging system to allow hierarchical logs.
#if 0
#ifdef HAVE_GSL
        diffuseAgent->getLogger()->enable();
        diffuseAgent->getLogger()->setPrintToStdoutFlag (true);
#else
        spreadAgent->getLogger()->enable();
        spreadAgent->getLogger()->setPrintToStdoutFlag (true);
#endif
#endif
        forgetAgent->getLogger()->enable();
        forgetAgent->getLogger()->setPrintToStdoutFlag (true);
    }
    switch (options->verboseLevel) {
    case 1:
        importUpdateAgent->getLogger()->setLevel (Logger::INFO);
        forgetAgent->getLogger()->setLevel (Logger::INFO);
        if (hebLearnAgent) hebLearnAgent->getLogger()->setLevel (Logger::INFO);
        else storkeyAgent->getLogger()->setLevel (Logger::INFO);
        break;
    case 2:
        importUpdateAgent->getLogger()->setLevel (Logger::DEBUG);
        forgetAgent->getLogger()->setLevel (Logger::DEBUG);
        if (hebLearnAgent) hebLearnAgent->getLogger()->setLevel (Logger::DEBUG);
        else storkeyAgent->getLogger()->setLevel (Logger::DEBUG);
        break;
    case 3:
        importUpdateAgent->getLogger()->setLevel (Logger::FINE);
        forgetAgent->getLogger()->setLevel (Logger::FINE);
        if (hebLearnAgent) hebLearnAgent->getLogger()->setLevel (Logger::FINE);
        else storkeyAgent->getLogger()->setLevel (Logger::FINE);
        break;
    default:
        importUpdateAgent->getLogger()->setLevel (Logger::WARN);
        forgetAgent->getLogger()->setLevel (Logger::WARN);
        if (hebLearnAgent) hebLearnAgent->getLogger()->setLevel (Logger::WARN);
        else storkeyAgent->getLogger()->setLevel (Logger::WARN);
    }

    if (hebLearnAgent)
        hebLearnAgent->convertLinks = true;
    forgetAgent->forgetPercentage = options->forgetPercent;

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
        maxLinks = n * (n + 1) / 2;
        this->links = (int) (density * maxLinks);
        logger().info("Density of %.2f gives %d links out of %d", density, this->links, maxLinks);
    }

    // Tune mind agents from command line options
    //spreadAgent->setSpreadThreshold(options->spreadThreshold);
    //spreadAgent->setImportanceSpreadingMultiplier(options->importanceSpreadingMultiplier);

    // Create nodes
    for (int i = 0; i < this->width; i++) {
        for (int j = 0; j < this->height; j++) {
            nodeName = "Hopfield_";
            nodeName += to_string(i) + "_" + to_string(j);
            Handle h = atomSpace->addNode(CONCEPT_NODE, nodeName.c_str());
            // We don't want the forgetting process to remove
            // the atoms perceiving the patterns
            atomSpace->setVLTI(h, AttentionValue::NONDISPOSABLE);
            hGrid.push_back(h);
        }
    }

    // If only 1 node, don't try and connect it
    if (hGrid.size() < 2) {
        logger().warn("Only %d node(s), <2 so this is kind of silly but I shall follow instructions.", hGrid.size());
        return;
    }

    addRandomLinks();

	// make sure nodes are slightly negative if necessary
	// otherwise Hebbian Learning doesn't detect (get target conjunction == 0)
	resetNodes();

}

void HopfieldServer::reset()
{
    AtomSpace* atomSpace = getAtomSpace();
    HandleEntry *links, *l;

    // Remove all links and replace
    links = atomSpace->getAtomTable().getHandleSet(HEBBIAN_LINK, true);
    for (l = links; l->next; l = l->next)
        atomSpace->removeAtom(l->handle);
    delete links;

    addRandomLinks();
}

void HopfieldServer::addRandomLinks()
{
    AtomSpace* atomSpace = getAtomSpace();
    HandleEntry *links;
    int amount;

    // Add links if less than desired number and to replace forgotten links
    links = atomSpace->getAtomTable().getHandleSet(HEBBIAN_LINK, true);
    amount = this->links - links->getSize();
    delete links;

    logger().fine("Adding %d random Hebbian Links.", amount);
    // Link nodes randomly with amount links
    while (amount > 0) {
        int source, target;
        HandleSeq outgoing;
        HandleEntry* he;

        source = rng->randint(hGrid.size());
        target = rng->randint(hGrid.size() - 1);
        if (target >= source) target++;

        outgoing.push_back(hGrid[source]);
        outgoing.push_back(hGrid[target]);
        he = atomSpace->getAtomTable().getHandleSet(outgoing, (Type*) NULL, (bool*) NULL, outgoing.size(), SYMMETRIC_HEBBIAN_LINK, false);
        if (he) {
            //logger().fine("Trying to add %d -> %d, but already exists %s", source, target, TLB::getAtom(he->handle)->toString().c_str());
            delete he;
        } else {
            atomSpace->addLink(SYMMETRIC_HEBBIAN_LINK, outgoing);
            amount--;
        }
    }


}

void HopfieldServer::resetNodes(bool toDefault)
{
    AtomSpace* a = getAtomSpace();
    HandleEntry *nodes, *n;

    nodes = a->getAtomTable().getHandleSet(NODE, true);

	if (toDefault) {
		for (n = nodes; n; n = n->next) {
			// Set all nodes to default STI and default LTI
			a->setSTI(n->handle, AttentionValue::DEFAULTATOMSTI);
			a->setLTI(n->handle, AttentionValue::DEFAULTATOMLTI);
		}
	} else {
		// Set nodes to negative of AF boundary - patternStimulus*wages
		AttentionValue::sti_t startSTI;
		AttentionValue::lti_t startLTI;
		startSTI = getAtomSpace()->getAttentionalFocusBoundary() -
			(patternStimulus * importUpdateAgent->getSTIAtomWage())/hGrid.size();
		startLTI = getAtomSpace()->getAttentionalFocusBoundary() -
			(patternStimulus * importUpdateAgent->getLTIAtomWage())/hGrid.size();
		for (n = nodes; n; n = n->next) {
			a->setSTI(n->handle, startSTI);
			a->setLTI(n->handle, startLTI);
		}
	}
    
    delete nodes;

    logger().debug("Nodes Reset");
}

void HopfieldServer::imprintPattern(Pattern pattern, int cycles)
{
    static bool first = true;

    logger().fine("---Imprint:Begin");
    // loop for number of imprinting cyles
    for (; cycles > 0; cycles--) {
        // for each encode pattern
        logger().fine("---Imprint:Encoding pattern");
        encodePattern(pattern, patternStimulus);
        printStatus();
        // then update with learning

        logger().fine("---Imprint:Adding random links");
        addRandomLinks();

        // ImportanceUpdating with links
        logger().fine("---Imprint:Running Importance update");
        importUpdateAgent->run(this);
        printStatus();

        logger().fine("---Imprint:Hebbian learning");
        if (hebLearnAgent)
            hebLearnAgent->run(this);
        else
            storkeyAgent->run(this);

// Unnecessary
//        logger().fine("---Imprint:Importance spreading");
//#ifdef HAVE_GSL
//        diffuseAgent->run(this);
//#else
//        spreadAgent->run(this);
//#endif

        if (first)
            first = false;
        else
            logger().fine("---Imprint:Forgetting");
        forgetAgent->run(this);

        printStatus();

        logger().fine("---Imprint:Resetting nodes");
        resetNodes();
    }
    logger().fine("---Imprint:End");

}

void HopfieldServer::encodePattern(Pattern pattern, stim_t stimulus)
{
    std::vector<Handle>::iterator it = hGrid.begin();
    std::vector<int>::iterator p_it = pattern.begin();
	int activity;
	
	activity = pattern.activity();
	// Avoid floating point exception if blank pattern
	if (activity == 0)
		activity = 1;
	stim_t perUnit = patternStimulus / activity;

    while (it != hGrid.end() && p_it != pattern.end()) {
        Handle h = (*it);
        int value = (*p_it);
        getAtomSpace()->stimulateAtom(h, perUnit * value);
        it++; p_it++;
    }

}

Pattern HopfieldServer::retrievePattern(Pattern partialPattern, int numCycles, int spreadCycles)
{
    std::string logString;

    logString += "---Retrieve:Initialising " + to_string(numCycles) + " cycle pattern retrieval process.";
    logger().info(logString.c_str());

    logger().fine("---Retrieve:Resetting nodes");
    resetNodes();

    while (numCycles > 0) {
        logger().fine("---Retrieve:Encoding pattern");
        encodePattern(partialPattern, patternStimulus);
        printStatus();
        updateAtomTableForRetrieval(spreadCycles);
        printStatus();

        numCycles--;
        logger().info("---Retreive:Cycles left %d", numCycles);
    }

    logString = "Cue pattern: \n" + patternToString(partialPattern);
    logger().info(logString.c_str());

    logger().info("---Retrieve:End");

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

    logger().info("---Retreive:Running Importance updating agent");
    importUpdateAgent->run(this);

    logger().info("---Retreive:Spreading Importance %d times", spreadCycles);
    for (int i = 0; i < spreadCycles; i++) {
        logger().fine("---Retreive:Spreading Importance - cycle %d", i);
#ifdef HAVE_GSL
        diffuseAgent->run(this);
#else
        spreadAgent->run(this);
#endif
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
//    HandleEntry *links, *current_l;

    int i;
	

    int col;
    if (!options->verboseLevel) return;

    for (i = 0; i < height; i++) {
        for (col = 0; col < width; col++) {
            printf("% 1.2f ", nodeStim[i*width + col] / (float) patternStimulus);
        }
        cout << "| ";
        for (col = 0; col < width; col++) {
            printf("% 1.2f ", nodeSTI[i*width + col] / (float) getAtomSpace()->getMaxSTI());
        }
        cout << "| ";

        for (col = 0; col < width; col++) {
            printf("% 2d", pattern[i*width + col]);
        }
        cout << endl;
    }

    // Print out links.
//  if (options->verboseLevel > 1) {
// links = getAtomSpace()->getAtomTable().getHandleSet(HEBBIAN_LINK, true);
//
// for (current_l = links; current_l; current_l = current_l->next) {
//     Handle h = current_l->handle;
//
//     cout << TLB::getAtom(h)->toString() << endl;
//
// }
//  }
    cout << "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=" << endl;
}

std::string HopfieldServer::printMatrixResult(std::vector< Pattern > patterns)
{
    int i, col;

    for (i = 0; i < height; i++) {
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

