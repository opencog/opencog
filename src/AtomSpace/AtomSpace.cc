//#pragma warning(disable : 4311)
//#pragma warning(disable : 4312)

#include "AtomSpace.h"

#include <string>
#include <iostream>
#include <stdlib.h>

#include "TLB.h"
#include "SimpleTruthValue.h"
#include "IndefiniteTruthValue.h"
#include "Node.h"
#include "Link.h"
#include "HandleEntry.h"
#include "ClassServer.h"
#include <StatisticsMonitor.h>

#include "Logger.h"

#include "StdAfx.h"

using std::string;
using std::cerr;
using std::cout;
using std::endl;
using namespace Util;

const char* AtomSpace::SPACE_MAP_NODE_NAME = "SpaceMap";

AtomSpace::~AtomSpace() {

    // check if has already been deleted. See in code where it can be delete.
    if(_handle_iterator){
        delete (_handle_iterator);
    }

//    delete (_handle_entry);
}

AtomSpace::AtomSpace() {
    _handle_iterator = NULL;
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
    emptyName = "";

#ifdef USE_ATOM_HASH_MAP
    stimulatedAtoms = new AtomHashMap();
#else
    stimulatedAtoms = new AtomMap();
#endif
    totalStimulus = 0;
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_init(&stimulatedAtomsLock, NULL);
#endif

    fundsSTI = LOBE_STARTING_STI_FUNDS;
    fundsLTI = LOBE_STARTING_LTI_FUNDS;
    recentMaxSTI = 0;
    attentionalFocusBoundary = 1;
}

const AtomTable& AtomSpace::getAtomTable() const {
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
    return atomTable;
}

const TimeServer& AtomSpace::getTimeServer() const{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return timeServer;
}


void AtomSpace::print(std::ostream& output, Type type, bool subclass) const {
    atomTable.print(output, type, subclass);
}

Handle AtomSpace::addTimeInfo(Handle h, unsigned long timestamp, const TruthValue& tv) {
    cassert(TRACE_INFO, h != UNDEFINED_HANDLE, "AtomSpace::addTimeInfo: Got an UNDEFINED_HANDLE as argument\n");
    std::string nodeName = Temporal::getTimeNodeName(timestamp);
    return addTimeInfo(h, nodeName, tv);
}

Handle AtomSpace::addTimeInfo(Handle h, const Temporal& t, const TruthValue& tv) {
    cassert(TRACE_INFO, h != UNDEFINED_HANDLE, "AtomSpace::addTimeInfo: Got an UNDEFINED_HANDLE as argument\n");
    cassert(TRACE_INFO, t != UNDEFINED_TEMPORAL, "AtomSpace::addTimeInfo: Got an UNDEFINED_TEMPORAL as argument\n");
    return addTimeInfo(h, t.getTimeNodeName(), tv);
}

Handle AtomSpace::addTimeInfo(Handle h, const std::string& timeNodeName, const TruthValue& tv) {
//    MAIN_LOGGER.log(Logger::DEBUG, "AtomSpace::addTimeInfo - temp init");
    Handle timeNode = addNode(TIME_NODE, timeNodeName.c_str());
//    MAIN_LOGGER.log(Logger::DEBUG, "AtomSpace::addTimeInfo - temp 1");
    HandleSeq atTimeLinkOutgoing;
    atTimeLinkOutgoing.push_back(timeNode);
    atTimeLinkOutgoing.push_back(h);
    Handle atTimeLink = addLink(AT_TIME_LINK, atTimeLinkOutgoing, tv);
//    MAIN_LOGGER.log(Logger::DEBUG, "AtomSpace::addTimeInfo - temp end");
    return atTimeLink;
}

bool AtomSpace::removeTimeInfo(Handle h, unsigned long timestamp, TemporalTable::TemporalRelationship criterion, bool removeDisconnectedTimeNodes, bool recursive)
{
    Temporal t(timestamp);
    return removeTimeInfo(h, t, criterion, removeDisconnectedTimeNodes, recursive);
}

bool AtomSpace::removeTimeInfo(Handle h, const Temporal& t, TemporalTable::TemporalRelationship criterion, bool removeDisconnectedTimeNodes, bool recursive) {
    //printf("AtomSpace::removeTimeInfo(%s, %s, %d, %s, %d, %d)\n", TLB::getHandle(h)->toString().c_str(), t.toString().c_str(), TemporalTable::getTemporalRelationshipStr(criterion), removeDisconnectedTimeNodes, recursive);

    list<HandleTemporalPair> existingEntries;
    timeServer.get(back_inserter(existingEntries), h, t, criterion);
    bool result = !existingEntries.empty();
    for(list<HandleTemporalPair>::const_iterator itr = existingEntries.begin();
         itr != existingEntries.end(); itr++) {
        Handle atTimeLink = getAtTimeLink(*itr);
        //printf("Got atTimeLink = %p\n", atTimeLink);
        if (atTimeLink != UNDEFINED_HANDLE) {
            Handle timeNode = getOutgoing(atTimeLink, 0);
            //printf("Got timeNode = %p\n", timeNode);
            cassert(TRACE_INFO, getArity(atTimeLink) == 2, "AtomSpace::removeTimeInfo: Got invalid arity for AtTimeLink = %d\n", getArity(atTimeLink));
            cassert(TRACE_INFO, timeNode != UNDEFINED_HANDLE && getType(timeNode) == TIME_NODE, "AtomSpace::removeTimeInfo: Got no TimeNode node at the first position of the AtTimeLink\n");
            if (removeAtom(atTimeLink, recursive)) {
                //printf("atTimeLink removed from AT successfully\n");
                if (removeDisconnectedTimeNodes && getIncoming(timeNode).empty()) {
                    //printf("Trying to remove timeNode as well\n");
                    removeAtom(timeNode);
                }
            } else {
                result = false;
            }
        } else {
            result = false;
        }
    }
    return result;
}

Handle AtomSpace::getAtTimeLink(const HandleTemporalPair& htp) const{
    Handle result = UNDEFINED_HANDLE;

    const Temporal& t = *(htp.getTemporal());
    Handle h = htp.getHandle();
    //printf("AtomSpace::getAtTimeLink: t = %s, h = %s\n", t.toString().c_str(), TLB::getAtom(h)->toString().c_str());

    Handle timeNode = getHandle(TIME_NODE, t.getTimeNodeName().c_str());
    //printf("timeNode = %p\n", timeNode);
    if (CoreUtils::compare(timeNode, UNDEFINED_HANDLE)) {
        HandleSeq atTimeLinkOutgoing(2);
        atTimeLinkOutgoing[0] = timeNode;
        atTimeLinkOutgoing[1] = h;
        HandleSeq atTimeLinks;
        getHandleSet(back_inserter(atTimeLinks), atTimeLinkOutgoing, NULL, NULL, 2, AT_TIME_LINK, false);
        if (!atTimeLinks.empty()) {
            result = atTimeLinks[0];
            if (atTimeLinks.size() > 1) {
                MAIN_LOGGER.log(Util::Logger::WARNING,
                    "AtomSpace::getAtTimeLink: More than 1 AtTimeLink(TimeNode, TimedAtom) found for HandleTemporalPair = %s \n",
                    htp.toString().c_str());
            }
        //} else {
        //    MAIN_LOGGER.log(Util::Logger::DEBUG,
        //        "AtomSpace::getAtTimeLink: No corresponding AtTimeLink(TimeNode, TimedAtom) found for HandleTemporalPair = %s \n",
        //        htp.toString().c_str());
        }
    //} else {
    //    MAIN_LOGGER.log(Util::Logger::DEBUG, "AtomSpace::getAtTimeLink: No TimeNode found for Temporal = %s (timeNodeName = %s)\n", t.toString().c_str(), t.getTimeNodeName().c_str());
    }
    return result;
}

const TruthValue& AtomSpace::getDefaultTV() {
    return TruthValue::DEFAULT_TV();
}

Type AtomSpace::getTypeV(const tree<Vertex>& _target) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return getType(v2h(*_target.begin()));
}

bool AtomSpace::isReal(Handle h) const
{
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
        return TLB::getAtom(h)->isReal();
}

Type AtomSpace::getType(Handle h) const
{
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
        return TLB::getAtom(h)->getType();
}

Type AtomSpace::getAtomType(const string& str) const {
        //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return ClassServer::getType(const_cast<char*>(str.c_str()));
}
bool AtomSpace::inheritsType(Type t1,Type t2) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    //printf("AtomSpace::inheritsType(t1=%d,t2=%d)\n", t1, t2);
    bool result = ClassServer::isAssignableFrom(t2,t1);
    //printf("AtomSpace::inheritsType result = %d\n", result);
    return result;
}
bool AtomSpace::isNode(Type t) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    // XXX TODO: it would be computationally much more efficient
    // to just do this:
    // return (NULL != dynamic_cast<Node *>(this));
    return inheritsType(t,NODE);
}
string AtomSpace::getName(Type t) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    if (ClassServer::getClassName()->find(t)==ClassServer::getClassName()->end()) {
        cassert(TRACE_INFO, false, "AtomSpace::getName(): Unknown atom type.");
    }
    return ClassServer::getTypeName(t);
}

bool AtomSpace::isNode(Handle h) const {          //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
return inheritsType(getType(h), NODE); }
bool AtomSpace::isVar(Handle h) const {          //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
return inheritsType(getType(h),VARIABLE_NODE); }
bool AtomSpace::isList(Handle h) const {          //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
return inheritsType(getType(h), LIST_LINK); }

bool AtomSpace::containsVar(Handle h) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    if (!isNode(h)) {
        for (int i=0;i<getArity(h);++i)
            if (containsVar(getOutgoing(h,i)))
                return true;
        return false;
    }
    return isVar(h);
}

Handle AtomSpace::createHandle(Type t,const string& str,bool managed) {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    Handle h=getHandle(t,str);
    return h ? h : addNode(t,str,TruthValue::NULL_TV());
}

Handle AtomSpace::createHandle(Type t,const HandleSeq& outgoing,bool managed) {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    Handle h=getHandle(t,outgoing);
    return h ? h : addLink(t,outgoing,TruthValue::NULL_TV());
}

bool AtomSpace::containsVersionedTV(Handle h, VersionHandle vh) const {
     //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    bool result = isNullVersionHandle(vh);
    if (!result) {
        const TruthValue& tv = this->getTV(h);
        result = !tv.isNullTv() && tv.getType() == COMPOSITE_TRUTH_VALUE &&
            !(((const CompositeTruthValue&) tv).getVersionedTV(vh).isNullTv());
    }
    return result;
}

Handle AtomSpace::addAtom(tree<Vertex>& a, tree<Vertex>::iterator it, const TruthValue& tvn) {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    cassert(TRACE_INFO, boost::get<Handle>(&*it), "AtomSpace::addAtom(): Vertex should be of 'Handle' type.");

    HandleSeq handles;
    Handle head_type = boost::get<Handle>(*it);

    if (isReal(head_type))
    {
        return addRealAtom(*(TLB::getAtom(head_type)), tvn);
    }

    for (tree<Vertex>::sibling_iterator i = a.begin(it); i!=a.end(it); i++)
    {
        Handle *h_ptr = boost::get<Handle>(&*i);

        if (h_ptr && isReal(*h_ptr)) {
            handles.push_back(addRealAtom(*TLB::getAtom(*h_ptr), TruthValue::NULL_TV()));
        } else {
            handles.push_back(addAtom(a, i, TruthValue::TRIVIAL_TV()));
        }
    }

    return addLink((Type)(long)head_type, handles, tvn);
}

Handle AtomSpace::addAtom(tree<Vertex>& a, const TruthValue& tvn) {
          //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
   return addAtom(a,a.begin(),tvn);
}

bool AtomSpace::removeAtom(Handle h, bool recursive) {
    HandleEntry* extractedHandles = atomTable.extract(h, recursive);
    if (extractedHandles) {
        HandleEntry* currentEntry = extractedHandles;
        while (currentEntry) {
            Handle h = currentEntry->handle;
            Type type = getType(h);
            if (type == AT_TIME_LINK) {
                cassert(TRACE_INFO, getArity(h) == 2, "AtomSpace::removeAtom: Got invalid arity for removed AtTimeLink = %d\n", getArity(h));

                Handle timeNode = getOutgoing(h, 0);
                //printf("Got timeNode = %p\n", timeNode);
                cassert(TRACE_INFO, getType(timeNode) == TIME_NODE, "AtomSpace::removeAtom: Got no TimeNode node at the first position of the AtTimeLink\n");

                Handle timedAtom = getOutgoing(h, 1);
                timeServer.remove(timedAtom, Temporal::getFromTimeNodeName(((Node*) TLB::getAtom(timeNode))->getName().c_str()));
            }
	    // Also refund sti/lti to AtomSpace funds pool
	    fundsSTI += getSTI(h);
	    fundsLTI += getLTI(h);

	    // Remove stimulus
	    removeStimulus(h);
	    
            currentEntry = currentEntry->next;

        }
        atomTable.removeExtractedHandles(extractedHandles);
        return true;
    }
    return false;
}

bool TValued(Type t)
{
    return t != LIST_LINK;
}


#ifdef USE_STD_VECTOR_FOR_OUTGOING
const HandleSeq& AtomSpace::getOutgoing(Handle h) const
#else
HandleSeq AtomSpace::getOutgoing(Handle h) const
#endif
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return TLB::getAtom(h)->getOutgoingSet();
}

Handle AtomSpace::addNode(Type t,const string& name,const TruthValue& tvn) {
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    Handle result = atomTable.getHandle(name.c_str(), t);
    if (result) {
        // Just merges the TV
        if (!tvn.isNullTv()) {
            const TruthValue& currentTV = getTV(result);
            if (currentTV.isNullTv()) {
                setTV(result, tvn);
            } else {
                TruthValue* mergedTV = currentTV.merge(tvn);
                setTV(result, *mergedTV);
                delete mergedTV;
            }
        }
    } else {
	// Remove default STI/LTI from AtomSpace Funds
	fundsSTI -= AttentionValue::DEFAULTATOMSTI;
	fundsLTI -= AttentionValue::DEFAULTATOMLTI;

	// Add Node
        result = atomTable.add(new Node(t, name, tvn));
	
    }
    return result;
}

Handle AtomSpace::addLink(Type t,const HandleSeq& outgoing,const TruthValue& tvn)
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    Handle result;
    HandleEntry* he = atomTable.getHandleSet(outgoing, (Type*) NULL, (bool*) NULL, outgoing.size(), t, false);
    if (he) {
        result = he->handle;
        // Just merges the TV
        if (!tvn.isNullTv()) {
            const TruthValue& currentTV = getTV(result);
            if (currentTV.isNullTv()) {
                setTV(result, tvn);
            } else {
                TruthValue* mergedTV = currentTV.merge(tvn);
                setTV(result, *mergedTV);
                delete mergedTV;
            }
        }
        delete he;
    } else {
	// Remove default STI/LTI from AtomSpace Funds
	fundsSTI -= AttentionValue::DEFAULTATOMSTI;
	fundsLTI -= AttentionValue::DEFAULTATOMLTI;

        Link* l = new Link(t, outgoing, tvn);
        result = atomTable.add(l);
        if (l->getType() == AT_TIME_LINK) {
            // Add corresponding TimeServer entry
            if (l->getArity() == 2) {
                Atom* timeAtom = l->getOutgoingAtom(0);
                if (timeAtom->getType() == TIME_NODE) {
                    const string& timeNodeName = ((Node*) timeAtom)->getName();

                    // USED TO SEEK MEMORY LEAK
                    //if(uniqueTimestamp.find(timeNodeName) == uniqueTimestamp.end()){
                    //    uniqueTimestamp.insert(timeNodeName);
                    //    cout << "Total unique timestamps: " << uniqueTimestamp.size() << endl;
                    //}

                    Temporal t = Temporal::getFromTimeNodeName(timeNodeName.c_str());
                    Handle h = TLB::getHandle(l->getOutgoingAtom(1));
                    timeServer.add(h, t);
                } else {
                    MAIN_LOGGER.log(Util::Logger::WARNING,
                        "AtomSpace::addLink: Invalid atom type at the first element in an AtTimeLink's outgoing: %s\n",
                        ClassServer::getTypeName(timeAtom->getType()));
                }
            } else {
                MAIN_LOGGER.log(Util::Logger::WARNING,
                    "AtomSpace::addLink: Invalid arity for an AtTimeLink: %d (expected: 2)\n",
                    l->getArity());
            }
        }
    }
    return result;
}

Handle AtomSpace::addRealAtom(const Atom& atom, const TruthValue& tvn) {
    //printf("AtomSpace::addRealAtom\n");
    const TruthValue& newTV = (tvn.isNullTv())?atom.getTruthValue():tvn;
    // Check if the given handle is of an atom that was not inserted yet.
    // If so, adds the atom. Otherwise, just sets result to the correct/valid handle.
    Handle result;
    if (isNode(atom.getType())) {
        const Node& node = (const Node&) atom;
        result = getHandle(node.getType(),node.getName());
        if (!result) {
            return addNode(node.getType(), node.getName(), newTV);
        }
    } else {
        const Link& link = (const Link&) atom;
        HandleSeq outgoing;
        for (int i = 0; i < link.getArity(); i++) {
            outgoing.push_back(TLB::getHandle(link.getOutgoingAtom(i)));
        }
        result = getHandle(link.getType(), outgoing);
        if (!result) {
            return addLink(link.getType(), outgoing, newTV);
        }
    }
    const TruthValue& currentTV = getTV(result);
    if (currentTV.isNullTv()) {
        setTV(result, newTV);
    } else {
        TruthValue* mergedTV = currentTV.merge(newTV);
        setTV(result, *mergedTV);
        delete mergedTV;
    }

    return result;
}

Handle AtomSpace::getHandle(Type t,const string& str) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    Handle h=atomTable.getHandle(str.c_str(),t);
    return h;
}

Handle AtomSpace::getHandle(Type t,const HandleSeq& outgoing) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    HandleEntry* he=atomTable.getHandleSet(outgoing,NULL,NULL,outgoing.size(),t, false);
    Handle ret = he ? he->handle : NULL;
    delete he;
    return ret;
}

const string& AtomSpace::getName(Handle h) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    //  cassert(TRACE_INFO, isNode(h));
    //bool isNodeHandle = isNode(h);
    //printf("AtomSpace::getName: isNodeHandle = %d\n", isNodeHandle);
    //printf("AtomSpace::getName: atom = %s\n", TLB::getAtom(h)->toString().c_str());
    if (isNode(h))
        return dynamic_cast<Node*>(TLB::getAtom(h))->getName();
    else
        return emptyName;
}

Handle AtomSpace::getOutgoing(Handle h,int idx) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return TLB::getAtom(h)->getOutgoingSet()[idx];
}

int AtomSpace::getArity(Handle h) const {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return TLB::getAtom(h)->getArity();
}

void AtomSpace::setName(Handle h, const string& name)
{
          //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    cassert(TRACE_INFO, isNode(h), "AtomSpace::setName(): Handle h should be of 'Node' type.");
    dynamic_cast<Node*>(TLB::getAtom(h))->setName((char*)name.c_str());
}

HandleSeq AtomSpace::getIncoming(Handle h) const
{
          //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    HandleEntry* he = TLB::getAtom(h)->getIncomingSet();
    Handle *temp; int size;
    he->toHandleVector(temp,size);
    HandleSeq ret(size);
    for (int i=0; i<size; i++)
        ret[i]=temp[i];
    return ret;
}
void AtomSpace::setTV(Handle h,const TruthValue& tv, VersionHandle vh)
{
          //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    const TruthValue& currentTv = getTV(h);
    if (!isNullVersionHandle(vh)) {
        CompositeTruthValue ctv = (currentTv.getType() == COMPOSITE_TRUTH_VALUE)?
            CompositeTruthValue((const CompositeTruthValue&) currentTv):
            CompositeTruthValue(currentTv, NULL_VERSION_HANDLE);
        ctv.setVersionedTV(tv, vh);
        TLB::getAtom(h)->setTruthValue(ctv); // always call setTruthValue to update indices
    } else {
        if (currentTv.getType() == COMPOSITE_TRUTH_VALUE &&
            tv.getType() != COMPOSITE_TRUTH_VALUE) {
            CompositeTruthValue ctv((const CompositeTruthValue&) currentTv);
            ctv.setVersionedTV(tv, vh);
            TLB::getAtom(h)->setTruthValue(ctv);
        } else {
            TLB::getAtom(h)->setTruthValue(tv);
        }
    }
}

const TruthValue& AtomSpace::getTV(Handle h, VersionHandle vh) const
{
          //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    const TruthValue& tv  = TLB::getAtom(h)->getTruthValue();
    if (isNullVersionHandle(vh)) {
        return tv;
    } else if (tv.getType() == COMPOSITE_TRUTH_VALUE) {
        return ((const CompositeTruthValue&) tv).getVersionedTV(vh);
    }
    return TruthValue::NULL_TV();
}

void AtomSpace::setMean(Handle h, float mean) throw (InvalidParamException) {
    TruthValue* newTv = getTV(h).clone();
    if (newTv->getType() == COMPOSITE_TRUTH_VALUE) {
        // Since CompositeTV has no setMean() method, we must handle it differently
        CompositeTruthValue* ctv = (CompositeTruthValue*) newTv;
        TruthValue* primaryTv = ctv->getVersionedTV(NULL_VERSION_HANDLE).clone();
        if (primaryTv->getType() == SIMPLE_TRUTH_VALUE) {
            ((SimpleTruthValue*)primaryTv)->setMean(mean);
        } else if (primaryTv->getType() == INDEFINITE_TRUTH_VALUE) {
            ((IndefiniteTruthValue*)primaryTv)->setMean(mean);
        } else {
            throw InvalidParamException(TRACE_INFO,
                 "AtomSpace - Got a primaryTV with an invalid or unknown type");
        }
        ctv->setVersionedTV(*primaryTv, NULL_VERSION_HANDLE);
        delete primaryTv;
    } else {
        if (newTv->getType() == SIMPLE_TRUTH_VALUE) {
            ((SimpleTruthValue*)newTv)->setMean(mean);
        } else if (newTv->getType() == INDEFINITE_TRUTH_VALUE) {
            ((IndefiniteTruthValue*)newTv)->setMean(mean);
        } else {
            throw InvalidParamException(TRACE_INFO,
                  "AtomSpace - Got a TV with an invalid or unknown type");
        }
    }
    setTV(h,*newTv);
    delete newTv;
}


const AttentionValue& AtomSpace::getAV(Handle h) const
{
    return TLB::getAtom(h)->getAttentionValue();
}

void AtomSpace::setAV(Handle h, const AttentionValue& av)
{
    const AttentionValue& oldAV = TLB::getAtom(h)->getAttentionValue();
    // Add the old attention values to the AtomSpace funds and
    // subtract the new attention values from the AtomSpace funds
    fundsSTI += (oldAV.getSTI() - av.getSTI());
    fundsLTI += (oldAV.getLTI() - av.getLTI());

    TLB::getAtom(h)->setAttentionValue(av); // setAttentionValue takes care of updating indices

}

void AtomSpace::setSTI(Handle h, AttentionValue::sti_t stiValue)
{
    const AttentionValue& currentAv = getAV(h);
    setAV(h, AttentionValue(stiValue, currentAv.getLTI(), currentAv.getVLTI()));

}

void AtomSpace::setLTI(Handle h, AttentionValue::lti_t ltiValue)
{
    const AttentionValue& currentAv = getAV(h);
    setAV(h, AttentionValue(currentAv.getSTI(), ltiValue, currentAv.getVLTI()));
}

void AtomSpace::setVLTI(Handle h, AttentionValue::vlti_t vltiValue)
{
    const AttentionValue& currentAv = getAV(h);
    setAV(h, AttentionValue(currentAv.getSTI(), currentAv.getLTI(), vltiValue));
}

AttentionValue::sti_t AtomSpace::getSTI(Handle h) const {
    return TLB::getAtom(h)->getAttentionValue().getSTI();
}

AttentionValue::lti_t AtomSpace::getLTI(Handle h) const {
    return TLB::getAtom(h)->getAttentionValue().getLTI();
}

AttentionValue::vlti_t AtomSpace::getVLTI(Handle h) const {
    return TLB::getAtom(h)->getAttentionValue().getVLTI();
}

float AtomSpace::getCount(Handle h) const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return TLB::getAtom(h)->getTruthValue().getCount();
}

int AtomSpace::Nodes(VersionHandle vh) const
{
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    /* This is too expensive and depending on a MindAgent that may be disabled.
     * Besides it does not have statistics by VersionHandles
     DynamicsStatisticsAgent *agent=DynamicsStatisticsAgent::getInstance();
     agent->reevaluateAllStatistics();
     return agent->getNodeCount();
     */
    // The following implementation is still expensive, but already deals with VersionHandles:
    HandleEntry* he = atomTable.getHandleSet(NODE, true, vh);
    int result = he->getSize();
    delete he;
    return result;
}

void AtomSpace::decayShortTermImportance() {
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    atomTable.decayShortTermImportance();
}

long AtomSpace::getTotalSTI() const
{
    long totalSTI = 0;
    HandleEntry* q;

    /* get atom iterator, go through each atom and calculate add
     * sti to total */

    HandleEntry* h = getAtomTable().getHandleSet(ATOM, true);
    q=h;
    while (q) {
        totalSTI += getSTI(q->handle);
        q = q->next;
    }
    delete h;
    return totalSTI;

}

long AtomSpace::getTotalLTI() const
{
    long totalLTI = 0;
    HandleEntry* q;

    /* get atom iterator, go through each atom and calculate add
     * lti to total */

    HandleEntry* h = getAtomTable().getHandleSet(ATOM, true);
    q=h;
    while (q) {
        totalLTI += getLTI(q->handle);
        q = q->next;
    }
    delete h;
    return totalLTI;

}

long AtomSpace::getSTIFunds() const
{
    return fundsSTI;   
}

long AtomSpace::getLTIFunds() const
{
    return fundsLTI;
}

int AtomSpace::Links(VersionHandle vh) const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    /* This is too expensive and depending on a MindAgent that may be disabled.
     * Besides it does not have statistics by VersionHandles
     DynamicsStatisticsAgent *agent=DynamicsStatisticsAgent::getInstance();
     agent->reevaluateAllStatistics();
     return agent->getLinkCount();
     */
    // The following implementation is still expensive, but already deals with VersionHandles:
    HandleEntry* he = atomTable.getHandleSet(LINK, true, vh);
    int result = he->getSize();
    delete he;
    return result;
}

void AtomSpace::_getNextAtomPrepare() {
         //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    _handle_iterator=atomTable.getHandleIterator(ATOM,true);
}

    Handle AtomSpace::_getNextAtom() {
        if (_handle_iterator->hasNext())
            return _handle_iterator->next();
        else {
            delete _handle_iterator;
            _handle_iterator = NULL;
            return Handle(0);
        }
    }

void AtomSpace::_getNextAtomPrepare_type(Type type) {
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

        _handle_entry=atomTable.getHandleSet(type, true);
}

Handle AtomSpace::_getNextAtom_type(Type type) {
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    if (_handle_entry==NULL)
        return Handle(0);
    Handle h=_handle_entry->handle;
    _handle_entry=_handle_entry->next;
    return h;
}

stim_t AtomSpace::stimulateAtom(Handle h, stim_t amount)
{
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_lock(&stimulatedAtomsLock);
#endif
    // Add atom to the map of atoms with stimulus
    // and add stimulus to it
    (*stimulatedAtoms)[TLB::getAtom(h)] += amount;

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_unlock(&stimulatedAtomsLock);
#endif

    // update record of total stimulus given out
    totalStimulus += amount;
    return totalStimulus;
}

void AtomSpace::removeStimulus(Handle h)
{
    stim_t amount;
    // if handle not in map then return
    if (stimulatedAtoms->find(TLB::getAtom(h)) == stimulatedAtoms->end()) 
	return;

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_lock(&stimulatedAtomsLock);
#endif
    amount = (*stimulatedAtoms)[TLB::getAtom(h)];
    stimulatedAtoms->erase(TLB::getAtom(h));
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_unlock(&stimulatedAtomsLock);
#endif

    // update record of total stimulus given out
    totalStimulus -= amount;
}

stim_t AtomSpace::stimulateAtom(HandleEntry* h, stim_t amount)
{
    HandleEntry* p;
    stim_t split;

    // how much to give each atom
    split = amount / h->getSize();

    p = h;
    while (p) {
        stimulateAtom(p->handle, split);
        p = p->next;
    }

    // return unused stimulus
    return amount - (split * h->getSize());
}

stim_t AtomSpace::resetStimulus()
{

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_lock(&stimulatedAtomsLock);
#endif
    stimulatedAtoms->clear();
    // reset stimulus counter
    totalStimulus = 0;
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_unlock(&stimulatedAtomsLock);
#endif
    return totalStimulus;
}

stim_t AtomSpace::getTotalStimulus()
{
    return totalStimulus;
}

stim_t AtomSpace::getAtomStimulus(Handle h)
{
    if (stimulatedAtoms->find(TLB::getAtom(h)) == stimulatedAtoms->end()) {
        return 0;
    } else {
        return (*stimulatedAtoms)[TLB::getAtom(h)];
    }
}

AttentionValue::sti_t AtomSpace::getAttentionalFocusBoundary()
{
    return attentionalFocusBoundary;
}

AttentionValue::sti_t AtomSpace::setAttentionalFocusBoundary(AttentionValue::sti_t s)
{
    attentionalFocusBoundary = s;
    return s;
}

AttentionValue::sti_t AtomSpace::getRecentMaxSTI()
{
    return recentMaxSTI;
}

AttentionValue::sti_t AtomSpace::setRecentMaxSTI(AttentionValue::sti_t val)
{
    recentMaxSTI = val;
    return recentMaxSTI;
}
