/*
 * opencog/visualizer/include/AtomSpaceInterface.h
 *
 * Copyright (C) 2012 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Erwin Joosten <eni247@gmail.com>
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

#ifndef ATOMSPACEINTERFACE_H
#define ATOMSPACEINTERFACE_H

#include <limits>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <istream>
#include <ostream>
#include <sstream>
#include <map>
#include <string>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <stdexcept>

#include "AtomTypes.h"
#include "Vertex.h"


using namespace std;
using boost::asio::ip::tcp;
using boost::lexical_cast;
using boost::bad_lexical_cast;

class AtomFilter
{
    public:
        AtomFilter();
        virtual ~AtomFilter();

        string name;
        UUID uuid;
        int type;
        bool includeSubtypes;
        int sortOrder;
    private:
};

class NodeFilter
{
    public:
        NodeFilter();
        virtual ~NodeFilter();

        int type;
        bool includeSubtypes;
        short minimumSTI;
        short minimumLTI;
        double minimumStrength;
        double minimumConfidenceValue;

        bool PassesFilter(Vertex* vertex, AtomTypes* atomTypes);
    private:
};

class LinkFilter
{
    public:
        LinkFilter();
        virtual ~LinkFilter();

        int type;
        bool includeSubtypes;
        short minimumSTI;
        short minimumLTI;
        double minimumStrength;
        double minimumConfidenceValue;

        bool PassesFilter(Vertex* vertex, AtomTypes* atomTypes);
    private:
};

class AtomSpaceInterface
{
    public:
		string server;
        AtomSpaceInterface(AtomTypes* atomTypes1);
        virtual ~AtomSpaceInterface();
        void SearchAtom(AtomFilter* atomFilter, vector<Vertex*>& foundVertices, bool &isFinished, int skip);
        void GetConnectedAtoms(Vertex* vertex, NodeFilter* nodeFilter, LinkFilter* linkFilter, vector<Vertex*>& connectedVertices);
        void UpdateAtom(UUID uuid, short lti, short sti, const string &truthValue);

    private:
        int atomCount;
        AtomTypes* atomTypes;
        static const int maxNumberOfConnectedVertices=20;
        void RetrieveVerticesFromCogServer(const string &query,vector<Vertex*>& foundVertices, bool &isFinished);
        string GetResultFromCogServer(const string &queryString);
        string PostToCogServer(const string &queryString, const string &postBody);
        void ConnectToCogServer(boost::asio::io_service &io_service,tcp::socket &socket);
        void AssertChar(char c, char d, stringstream &ss);
        void SkipWhiteSpace(stringstream &ss, char &c);
        void ParseString(stringstream &ss, string &str);
        void ParseAny(stringstream &ss, string &str);
        void ParseField(stringstream &ss, string &fieldName, string &fieldContent);
        void RemoveQuotes(string &s);
        void Trim(string &s);
        void ParseRecord(stringstream &ss,map<string,string> &mapRecord);
        void ParseList(stringstream &ss,vector<string> &vector);

};


#endif // ATOMSPACEINTERFACE_H
