/*
 * opencog/atomspace/ProtocolBufferSerializer.h
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

//TODO move this file to the persist directory

#ifndef _OPENCOG_PROTOCOLBUFFER_SERIALIZER_H
#define _OPENCOG_PROTOCOLBUFFER_SERIALIZER_H

#include "opencog/atomspace/types.h"
#include <string>
#include "ZMQMessages.pb.h"

using namespace std;

namespace opencog {
/** \addtogroup grp_atomspace
 *  @{
 */

class AttentionValue;
class AttentionValueHolder;
class Atom;
class Link;
class Node;
class TruthValue;
class CompositeTruthValue;
class CountTruthValue;
class IndefiniteTruthValue;
class NullTruthValue;
class SimpleTruthValue;
class Trail;
class VersionHandle;

class ProtocolBufferSerializer {
    static void deserializeAtom(const ZMQAtomMessage& atomMessage, Atom& atom);
    static void serializeAtom(Atom& atom, ZMQAtomMessage* atomMessage);

    static void deserializeAttentionValue(
            const ZMQAttentionValueHolderMessage &attentionValueHolderMessage,
            AttentionValue& av);
    static void serializeAttentionValue(
            AttentionValue& av, ZMQAttentionValueHolderMessage* attentionValueHolderMessage);
    static void deserializeAttentionValueHolder(
            const ZMQAttentionValueHolderMessage& attentionValueHolderMessage,
            AttentionValueHolder& attentionValueHolder);
    static void serializeAttentionValueHolder(
            AttentionValueHolder& attentionValueHolder,
            ZMQAttentionValueHolderMessage *attentionValueHolderMessage);

    static void deserializeLink(const ZMQAtomMessage& atomMessage, Link& link);
    static void serializeLink(Link& link, ZMQAtomMessage *atomMessage);
    static void deserializeNode(const ZMQAtomMessage& atomMessage, Node& node);
    static void serializeNode(Node& node, ZMQAtomMessage *atomMessage);

    static void deserializeCompositeTruthValue(
            const ZMQTruthValueMessage& truthValueMessage, CompositeTruthValue& tv);
    static void serializeCompositeTruthValue(
            CompositeTruthValue& tv, ZMQTruthValueMessage * truthValueMessage);
    static void deserializeCountTruthValue(
            const ZMQSingleTruthValueMessage& singleTruthValue, CountTruthValue& tv);
    static void serializeCountTruthValue(
            CountTruthValue& tv, ZMQTruthValueMessage* truthValueMessage);
    static void deserializeIndefiniteTruthValue(
            const ZMQSingleTruthValueMessage& singleTruthValue, IndefiniteTruthValue& tv);
    static void serializeIndefiniteTruthValue(
            IndefiniteTruthValue& tv, ZMQTruthValueMessage* truthValueMessage);
    static void serializeNullTruthValue(
            NullTruthValue& tv, ZMQTruthValueMessage* truthValueMessage);
    static void deserializeSimpleTruthValue(
            const ZMQSingleTruthValueMessage& singleTruthValue, SimpleTruthValue& tv);
    static void serializeSimpleTruthValue(
            SimpleTruthValue& tv, ZMQTruthValueMessage* truthValueMessage);

    static void deserializeTrail(const ZMQTrailMessage& trailMessage, Trail& trail);
    static void serializeTrail(Trail& trail, ZMQTrailMessage* trailMessage);
    static void deserializeVersionHandle(
            const ZMQVersionHandleMessage& versionHandleMessage, VersionHandle& vh);
    static void serializeVersionHandle(
            VersionHandle& vh, ZMQVersionHandleMessage* versionHandleMessage);
    static TruthValue* deserialize(
            const ZMQSingleTruthValueMessage& singleTruthValueMessage);

public: 
    ProtocolBufferSerializer();
    ~ProtocolBufferSerializer();

    static Atom* deserialize(const ZMQAtomMessage& atomMessage);
    static void serialize(Atom &atom, ZMQAtomMessage* atomMessage);

    static TruthValue* deserialize(const ZMQTruthValueMessage& truthValueMessage);
    static void serialize(TruthValue &tv, ZMQTruthValueMessage* truthValueMessage);
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_PROTOCOLBUFFER_SERIALIZER_H
