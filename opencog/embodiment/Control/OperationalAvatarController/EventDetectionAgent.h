/*
 * opencog/embodiment/Control/PerceptionActionInterface/EventDetectionAgent.h
 *
 * Copyright (C) 2011 OpenCog Foundation
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Shujing ke (rainkekekeke@gmail.com)

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

/** EventDetectionAgent.h
 *
 *  Detect events from continueously happening actions .
 */

#ifndef EventDetectorAgent_H
#define EventDetectorAgent_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/Agent.h>
#include <map>
#include <vector>
#include <opencog/learning/statistics/DataProvider.h>
#include <boost/graph/adjacency_list.hpp>

using namespace std;
using namespace opencog::statistics;

namespace opencog { namespace oac {

#define ActionsExportToScmFileName "ActionsScmCorpus"
#define MAX_PATTERN_GRAM 8

enum EVENT_TYPE
{
    EVENT_TYPE_ACTION,
    EVENT_TYPE_STATE_CHANGE,
    EVENT_TYPE_SPATIAL_PREDICATE
};

struct Frame
{
    HandleSeq Actions; // all the actions conduct by avatars in this frame
    HandleSeq StateChanges; // all the state changes in this frame
    HandleSeq SpatialPredicates;
};


//--------------------------Pattern mining -------------------------------------
// the Skeleton of a group of links, all the nodes are variable nodes
class Pattern;

class Skeleton
{
public:
    HandleSeq rootLinks; // these handles are in iAtomSpace
    vector<Pattern*> patterns;
    bool isSameSkeletonToMe(HandleSeq _rootLinks);
    int varNumber; // the number of variables in this skeleton

    vector<Skeleton*>  parents;
    vector<Skeleton*>  children;
};

// Pattern is a kind of special Skeleton, all some of its nodes may still be variable nodes, while some are grounded specific nodes
class Pattern
{
public:
    Skeleton* skeleton; // its skeleton this pattern belongs to.
    HandleSeqSeq instanceRootLinks; // these handles are the links in the main AtomSpace, which has a same skeleton as this Skeleton defined
    map<Handle,Handle> bindings; // map<variable,bindedvalue>
    bool isSamePatternToMe();

    vector<Pattern*>  parents;
    vector<Pattern*>  children;
};


//--------------------------Pattern mining end-------------------------------------


class EventDetectionAgent : public opencog::Agent
{
public:

    EventDetectionAgent(CogServer&);

    virtual ~EventDetectionAgent();

    void destroy();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("PerceptionActionInterface::EventDetectionAgent");
        return _ci;
    }

    // Entry of the Agent, CogServer will invoke this function during its cycle
    virtual void run();

    // collect action corpora from PAI
    // the actionConcernedHandles contains all the nodes concerned an action
    void actionCorporaCollect(std::vector<Handle> actionConcernedHandles);
    void exportActionConcernedNodesToSCM();

public:
    // <unsigned long timestamp,frame of this time>
    static map<unsigned long,Frame*> frames;
    static void addAnEvent(Handle eventHandle, unsigned long timestamp, EVENT_TYPE eventType);

private:
    AtomSpace& atomSpace;

    //--------------------------Pattern mining -------------------------------------
    // An imaginary AtomSpace using for storing all the possible patterns , not the main Atomspace used by the oac.
    // because there will be thousands of patterns generated during the process, a lot of which are just frequent but not useful,
    // we don't want to put all these temporary nodes and links into the main Atomspace.
    // But since all these patterns are in the same datastructure with the Atomspace, we just new another Atomspace to store them.
    AtomSpace* iAtomSpace;

    // the unsigned int indicates the occurency times of this skeleton
    // in fact the occurency times is the number of all the instanceRootLinks of this skeleton
    // we use this number as key of multimap,for convience of sort all the skeletons, so as to easily find the top frequent skeletons
    multimap<unsigned int, Skeleton*> allSkeletons;

    // the unsigned int indicates the occurency times of this Pattern
    // in fact the occurency times is the number of all the instanceRootLinks of this Pattern
    // we use this number as key of multimap,for convience of sort all the Patterns, so as to easily find the top frequent Patterns
    multimap<unsigned int, Pattern*> allPatterns;

    // to store all the node appear in corpus, map every node as a variable "$int"
    std::map<Handle,int> globalVarToInt;
    std::map<int,Handle> globalIntToVar;

    Skeleton* rootSkeleton; // the rootNode in a H-Tries to store all the skeletons and patterns.

    //--------------------------Pattern mining end-------------------------------------

private:

    std::map<UUID,Handle> allNodesForScmActions;

    int globalVarNum;

    // insert a node into allNodesForScmActions
    void insertNodeToScmMap(Handle node);

    //--------------------------Pattern mining -------------------------------------
    void processOneInputLink(Handle rootLink);
    void _processOneInputLink(Handle rootLink,std::map<Handle,Handle> varBindings);
    void extractPatternsBySharingSameVaiables(Handle oneLink);
    void extractSkeleton(Handle oneLink);

    bool isThisSkeletonAlreadyExisted(Skeleton* s, Skeleton** theExistedSkeleton);

    //--------------------------Pattern mining end-------------------------------------


    unsigned long cycleCount;  // Indicate how many times this mind
                               // agent has been executed


};

}}


#endif // EventDetectorAgent_H
