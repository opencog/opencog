/*
 * opencog/visualizer/include/AtomTypes.h
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

#ifndef ATOMTYPES_H
#define ATOMTYPES_H


#include <vector>
#include <set>
#include <string>
#include <ctype.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdexcept>

#include "Vertex.h"

using namespace std;

class AtomTypes
{
    public:
		string atomTypesScriptPath;
        vector<string> atomTypeNames;
        vector<string> nodeTypeNames;
        vector<string> linkTypeNames;
        vector<string> atomTypeSymbols;

        AtomTypes();
        virtual ~AtomTypes();

        void LoadAtomTypeScript();
        int ConvertTypeNameToNumber(string typeName);
        int ConvertNodeTypeToAtomType(int nodeType);
        int ConvertLinkTypeToAtomType(int linkType);
        bool CheckAtomType(Vertex* vertex, int type, bool includeSubtypes);
        bool IsNode(string &typeName);

    private:
        vector<set<int>*> atomTypeSubTypes;
        string MakeMixedCase(string typeName);
        void AddSymbol(string typeName);
        void AddSubTypes(string parentTypes);
	    static void Trim(string& line);
	    static void Split(const string &s, char delim, vector<string> &elems);
};

#endif // ATOMTYPES_H
