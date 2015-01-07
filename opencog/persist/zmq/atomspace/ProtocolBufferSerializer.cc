/*
 * opencog/atomspace/ProtocolBufferSerializer.cc
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Erwin Joosten
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

#include "ProtocolBufferSerializer.h"
#include "opencog/atomspace/Handle.h"
#include "opencog/atomspace/AttentionValue.h"
#include "opencog/atomspace/Atom.h"
#include "opencog/atomspace/Link.h"
#include "opencog/atomspace/Node.h"
#include "opencog/atomspace/TruthValue.h"
#include "opencog/atomspace/CountTruthValue.h"
#include "opencog/atomspace/NullTruthValue.h"
#include "opencog/atomspace/CompositeTruthValue.h"
#include "opencog/atomspace/IndefiniteTruthValue.h"
#include "opencog/atomspace/SimpleTruthValue.h"
#include "Trail.h"
#include "VersionHandle.h"

using namespace opencog;

//TODO move this file to the persist directory

ProtocolBufferSerializer::ProtocolBufferSerializer()
{
}

ProtocolBufferSerializer::~ProtocolBufferSerializer()
{
};


void ProtocolBufferSerializer::deserializeAtom(
        const ZMQAtomMessage& atomMessage, Atom& atom)
{
    deserializeAttentionValueHolder(atomMessage.attentionvalueholder(), atom);
    atom.handle = Handle(atomMessage.handle());

    atom.atomTable=NULL;
    if(atomMessage.incoming_size()==0)
    {
        atom.incoming=NULL;
    }
    else
    {
        atom.incoming = new HandleEntry(Handle(atomMessage.incoming(0)));
        HandleEntry *previous = atom.incoming;
        for(int i = 1; i < atomMessage.incoming_size(); i++)
        {
            HandleEntry *current = new HandleEntry(Handle(atomMessage.incoming(i)));
            previous->next = current;
            previous = current;
        }
    }

    atom.type = atomMessage.type();
    atom.flags = atomMessage.flags();

    atom.truthValue = deserialize(atomMessage.truthvalue());
}

void ProtocolBufferSerializer::serializeAtom(
        Atom& atom, ZMQAtomMessage* atomMessage)
{
    serializeAttentionValueHolder(atom, atomMessage->mutable_attentionvalueholder());

    atomMessage->set_handle(atom.handle.value());

    HandleEntry* next=atom.incoming;
    while(next)
    {
        atomMessage->add_incoming(next->handle.value());
        next = next->next;
    }

    atomMessage->set_type(atom.type);
    atomMessage->set_flags(atom.flags);

    serialize(*atom.truthValue, atomMessage->mutable_truthvalue());
}

Atom* ProtocolBufferSerializer::deserialize(const ZMQAtomMessage& atomMessage)
{
    switch(atomMessage.atomtype())
    {
    case ZMQAtomTypeNode:
    {
        Node* node = new Node();
        deserializeNode(atomMessage, *node);
        return node;
    }
    case ZMQAtomTypeLink:
    {
        Link* link = new Link();
        deserializeLink(atomMessage, *link);
        return link;
    }
    default:
        throw RuntimeException(TRACE_INFO, "Invalid ZMQ atomtype");
    }
}

void ProtocolBufferSerializer::serialize(Atom &atom, ZMQAtomMessage* atomMessage)
{
    Link* link = dynamic_cast<Link *>(&atom);
    if(link)
        serializeLink(*link, atomMessage);
    else
    {
        Node* node = dynamic_cast<Node *>(&atom);
        if(node)
            serializeNode(*node, atomMessage);
        else
            throw RuntimeException(TRACE_INFO, "Invalid atomtype");
    }
}

void ProtocolBufferSerializer::deserializeAttentionValue(
        const ZMQAttentionValueHolderMessage &attentionValueHolderMessage,
        AttentionValue& av)
{
    av.m_STI=attentionValueHolderMessage.sti();
    av.m_LTI=attentionValueHolderMessage.lti();
    av.m_VLTI=attentionValueHolderMessage.vlti();
}

void ProtocolBufferSerializer::serializeAttentionValue(
        AttentionValue& av, ZMQAttentionValueHolderMessage* attentionValueHolderMessage)
{
    attentionValueHolderMessage->set_sti(av.m_STI);
    attentionValueHolderMessage->set_lti(av.m_LTI);
    attentionValueHolderMessage->set_vlti(av.m_VLTI);
}

void ProtocolBufferSerializer::deserializeAttentionValueHolder(
        const ZMQAttentionValueHolderMessage &attentionValueHolderMessage,
        AttentionValueHolder& avh )
{
    deserializeAttentionValue(attentionValueHolderMessage, avh.attentionValue);
}

void ProtocolBufferSerializer::serializeAttentionValueHolder(
        AttentionValueHolder& avh, ZMQAttentionValueHolderMessage* attentionValueHolderMessage)
{
    serializeAttentionValue(avh.attentionValue, attentionValueHolderMessage);
}

void ProtocolBufferSerializer::deserializeCompositeTruthValue(
        const ZMQTruthValueMessage& truthValueMessage, CompositeTruthValue& tv)
{
    tv.primaryTV = deserialize(truthValueMessage.singletruthvalue(0));

    for (int i=1;i<truthValueMessage.singletruthvalue_size();i++)
    {
        const ZMQSingleTruthValueMessage& singleTruthValue = truthValueMessage.singletruthvalue(i);
        VersionHandle vh;
        deserializeVersionHandle(singleTruthValue.versionhandle(), vh);
        tv.versionedTVs[vh] = deserialize(singleTruthValue);
    }
}

void ProtocolBufferSerializer::serializeCompositeTruthValue(
        CompositeTruthValue& tv, ZMQTruthValueMessage* truthValueMessage)
{
    serialize(*(tv.primaryTV), truthValueMessage);

    for (VersionedTruthValueMap::const_iterator itr = tv.versionedTVs.begin();itr != tv.versionedTVs.end(); ++itr)
    {
        serialize(*(itr->second), truthValueMessage); //creates a new singletruthvaluemessage
        VersionHandle vh = itr->first;
        ZMQVersionHandleMessage *versionHandleMessage=truthValueMessage->mutable_singletruthvalue(truthValueMessage->singletruthvalue_size()-1)->mutable_versionhandle();
        serializeVersionHandle(vh, versionHandleMessage);
    }
}

void ProtocolBufferSerializer::deserializeCountTruthValue(
        const ZMQSingleTruthValueMessage& singleTruthValue, CountTruthValue& tv)
{
    tv.mean=singleTruthValue.mean();
    tv.confidence=singleTruthValue.confidence();
    tv.count=singleTruthValue.count();
}

void ProtocolBufferSerializer::serializeCountTruthValue(
        CountTruthValue& tv, ZMQTruthValueMessage* truthValueMessage)
{
    ZMQSingleTruthValueMessage *singleTruthValue=truthValueMessage->add_singletruthvalue();
    singleTruthValue->set_truthvaluetype(ZMQTruthValueTypeCount);
    singleTruthValue->set_mean(tv.mean);
    singleTruthValue->set_count(tv.count);
    singleTruthValue->set_confidence(tv.confidence);
}

void ProtocolBufferSerializer::deserializeIndefiniteTruthValue(
        const ZMQSingleTruthValueMessage& singleTruthValue, IndefiniteTruthValue& tv)
{
    tv.L=singleTruthValue.l();
    tv.U=singleTruthValue.u();
    tv.confidenceLevel=singleTruthValue.confidencelevel();
    tv.symmetric=singleTruthValue.symmetric();
    tv.diff=singleTruthValue.diff();
    tv.mean=singleTruthValue.mean();
    tv.confidence=singleTruthValue.confidence();
    tv.count=singleTruthValue.count();

    for(int i=0;i<singleTruthValue.firstorderdistribution_size();i++)
    {
        strength_t* s=new strength_t(singleTruthValue.firstorderdistribution(i));
        tv.firstOrderDistribution.push_back(s);
    }
}

void ProtocolBufferSerializer::serializeIndefiniteTruthValue(
        IndefiniteTruthValue& tv, ZMQTruthValueMessage* truthValueMessage)
{
    ZMQSingleTruthValueMessage *singleTruthValue=truthValueMessage->add_singletruthvalue();
    singleTruthValue->set_truthvaluetype(ZMQTruthValueTypeIndefinite);
    singleTruthValue->set_l(tv.L);
    singleTruthValue->set_u(tv.U);
    singleTruthValue->set_confidencelevel(tv.confidenceLevel);
    singleTruthValue->set_symmetric(tv.symmetric);
    singleTruthValue->set_diff(tv.diff);
    singleTruthValue->set_mean(tv.mean);
    singleTruthValue->set_count(tv.count);
    singleTruthValue->set_confidence(tv.confidence);
    for (float *f : tv.firstOrderDistribution)
    {
        singleTruthValue->add_firstorderdistribution(*f);
    }
}

void ProtocolBufferSerializer::deserializeLink(
        const ZMQAtomMessage& atomMessage, Link& link)
{
    deserializeAtom(atomMessage, link);

    for(int i=0;i<atomMessage.outgoing_size();i++)
    {
        link.outgoing.push_back(Handle(atomMessage.outgoing(i)));
    }

    if(!atomMessage.has_trail())
        link.trail=NULL;
    else
    {
        link.trail=new Trail();
        deserializeTrail(atomMessage.trail(), *(link.trail));
    }
}

void ProtocolBufferSerializer::serializeLink(
        Link& link, ZMQAtomMessage * atomMessage)
{
    serializeAtom(link, atomMessage);

    atomMessage->set_atomtype(ZMQAtomTypeLink);

    for (Handle h : link.outgoing)
        atomMessage->add_outgoing(h.value());

    if(link.trail)
        serializeTrail(*(link.trail), atomMessage->mutable_trail());
}

void ProtocolBufferSerializer::deserializeNode(
        const ZMQAtomMessage &atomMessage, Node& node)
{
    deserializeAtom(atomMessage, node);

    node.name=atomMessage.name();
}

void ProtocolBufferSerializer::serializeNode(
        Node& node, ZMQAtomMessage* atomMessage)
{
    serializeAtom(node, atomMessage);

    atomMessage->set_atomtype(ZMQAtomTypeNode);

    atomMessage->set_name(node.name);
}

void ProtocolBufferSerializer::serializeNullTruthValue(
        NullTruthValue& tv, ZMQTruthValueMessage* truthValueMessage)
{
    ZMQSingleTruthValueMessage *singleTruthValue = truthValueMessage->add_singletruthvalue();
    singleTruthValue->set_truthvaluetype(ZMQTruthValueTypeNull);
}

void ProtocolBufferSerializer::deserializeSimpleTruthValue(
        const ZMQSingleTruthValueMessage& singleTruthValue, SimpleTruthValue& tv)
{
    tv.mean=singleTruthValue.mean();
    tv.count=singleTruthValue.count();
}

void ProtocolBufferSerializer::serializeSimpleTruthValue(
        SimpleTruthValue& tv, ZMQTruthValueMessage* truthValueMessage)
{
    ZMQSingleTruthValueMessage *singleTruthValue=truthValueMessage->add_singletruthvalue();
    singleTruthValue->set_truthvaluetype(ZMQTruthValueTypeSimple);
    singleTruthValue->set_mean(tv.mean);
    singleTruthValue->set_count(tv.count);
}

void ProtocolBufferSerializer::deserializeTrail(
        const ZMQTrailMessage& trailMessage, Trail& t)
{
    t.maxSize=trailMessage.maxsize();
    if(trailMessage.trail_size()==0)
    {
        t.trail=NULL;
    }
    else
    {
        t.trail = new std::deque<Handle>(trailMessage.trail_size());
        for(int i=0;i<trailMessage.trail_size();i++)
            t.trail->push_back(Handle(trailMessage.trail(i)));
    }
}

void ProtocolBufferSerializer::serializeTrail(Trail& t, ZMQTrailMessage* trailMessage)
{
    trailMessage->set_maxsize(t.maxSize);
    for (Handle h : *(t.trail))
        trailMessage->add_trail(h.value());
}

void ProtocolBufferSerializer::serialize(TruthValue &tv, ZMQTruthValueMessage* truthValueMessage)
{
    CountTruthValue* count = dynamic_cast<CountTruthValue*>(&tv);
    if(count)
    {
        serializeCountTruthValue(*count, truthValueMessage);
        return;
    }

    IndefiniteTruthValue* indefinite = dynamic_cast<IndefiniteTruthValue*>(&tv);
    if(indefinite)
    {
        serializeIndefiniteTruthValue(*indefinite, truthValueMessage);
        return;
    }

    NullTruthValue* nulltv = dynamic_cast<NullTruthValue*>(&tv);
    if(nulltv)
    {
        serializeNullTruthValue(*nulltv, truthValueMessage);
        return;
    }

    SimpleTruthValue* simple = dynamic_cast<SimpleTruthValue*>(&tv);
    if(simple)
    {
        serializeSimpleTruthValue(*simple, truthValueMessage);
        return;
    }

    throw RuntimeException(TRACE_INFO, "Invalid truthvaluetype.");
}

TruthValue* ProtocolBufferSerializer::deserialize(
        const ZMQTruthValueMessage& truthValueMessage)
{
    if(truthValueMessage.singletruthvalue_size()==1)
        return deserialize(truthValueMessage.singletruthvalue(0));
    else
    {
        CompositeTruthValue* tv = new CompositeTruthValue();
        deserializeCompositeTruthValue(truthValueMessage, *tv);
        return tv;
    }
}

TruthValue* ProtocolBufferSerializer::deserialize(
        const ZMQSingleTruthValueMessage& singleTruthValueMessage)
{
    switch(singleTruthValueMessage.truthvaluetype())
    {
    case ZMQTruthValueTypeSimple:
    {
        SimpleTruthValue* tv = new SimpleTruthValue();
        deserializeSimpleTruthValue(singleTruthValueMessage, *tv);
        return tv;
    }
    case ZMQTruthValueTypeCount:
    {
        CountTruthValue* tv = new CountTruthValue();
        deserializeCountTruthValue(singleTruthValueMessage, *tv);
        return tv;
    }
    case ZMQTruthValueTypeNull:
    {
        NullTruthValue* tv = new NullTruthValue();
        return tv;
    }
    case ZMQTruthValueTypeIndefinite:
    {
        IndefiniteTruthValue* tv = new IndefiniteTruthValue();
        deserializeIndefiniteTruthValue(singleTruthValueMessage, *tv);
        return tv;
    }
    default:
         throw RuntimeException(TRACE_INFO, "Invalid ZMQ truthvaluetype: '%d'.",
                 singleTruthValueMessage.truthvaluetype());
    }
}

void ProtocolBufferSerializer::deserializeVersionHandle(
        const ZMQVersionHandleMessage& versionHandleMessage, VersionHandle& vh)
{
    vh.indicator=(IndicatorType)versionHandleMessage.indicator();
    vh.substantive=Handle(versionHandleMessage.substantive());
}

void ProtocolBufferSerializer::serializeVersionHandle(
        VersionHandle& vh, ZMQVersionHandleMessage * versionHandleMessage)
{
    versionHandleMessage->set_indicator(vh.indicator);
    versionHandleMessage->set_substantive(vh.substantive.value());
}

