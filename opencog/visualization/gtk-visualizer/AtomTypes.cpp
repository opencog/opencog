/*
 * opencog/visualizer/src/AtomTypes.cpp
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

#include "AtomTypes.h"
#include <opencog/util/macros.h>

AtomTypes::AtomTypes()
{
    char buffer[500];
    ssize_t i = readlink("/proc/self/exe", buffer, 500);
    OC_UNUSED(i);
    string path = buffer;
    size_t positionLastSlash = path.find_last_of('/');

    // XXX this is wrong, there are in fact 5 or 6 different atom types script files
    // Each one contains types for each of the different subsystems ...
    atomTypesScriptPath = path.substr(0,positionLastSlash) + "/../../../../opencog/atomspace/atom_types.script";
}

AtomTypes::~AtomTypes()
{
}

void AtomTypes::LoadAtomTypeScript()
{
    ifstream typesfile (atomTypesScriptPath.c_str());
    if (!typesfile.is_open())
    {
        throw runtime_error("File '"+atomTypesScriptPath+"' not found. Copy the atom_types.script file to the application folder or specify location on command line.");
    }

    string line;
    getline (typesfile,line);
    while ( typesfile.good() )
    {
        std::size_t comment = line.find_first_of("/");
        if (comment != string::npos)
            line = line.substr(0, comment);

		Trim(line);

        if (line.length()>0)
        {
        	string specialTypeName="";
            std::size_t quote = line.find("\"");
            if (quote != string::npos)
            {
            	int quote2 = line.find("\"", quote+1);
            	specialTypeName = line.substr(quote+1, quote2-quote-1);
            	line.erase(quote, quote2-quote+1);
            }

    		Trim(line);

    		std::size_t arrow = line.find(" <- ");
            string typeName;
            string parentTypes;
            if (arrow == string::npos)
            {
                typeName = line;
                parentTypes = "";
            }
            else
            {
                typeName = line.substr(0, arrow);
                parentTypes = line.substr(arrow+4);
            }

            string prettyTypeName;
            if(specialTypeName.length()>0)
            	prettyTypeName=specialTypeName;
            else
            	prettyTypeName = MakeMixedCase(typeName);

            atomTypeNames.push_back(prettyTypeName);
            if (typeName.find("NODE")!=string::npos)
                nodeTypeNames.push_back(prettyTypeName);
            if (typeName.find("LINK")!=string::npos)
                linkTypeNames.push_back(prettyTypeName);

            AddSymbol(prettyTypeName);

            AddSubTypes(parentTypes);
        }
        getline (typesfile,line);
    }
    typesfile.close();
}

void AtomTypes::AddSymbol(string typeName)
{
    string symbol = typeName.substr(0, 2);

    if(typeName.compare("AndLink")==0)
        symbol="∧";
    if(typeName.compare("OrLink")==0)
        symbol="∨";
    if(typeName.compare("NotLink")==0)
        symbol="¬";
    if(typeName.compare("SetLink")==0)
        symbol="{}";
    if(typeName.compare("MemberLink")==0)
        symbol="∈";
    if(typeName.compare("SubsetLink")==0)
        symbol="⊂";
    if(typeName.compare("ListLink")==0)
        symbol="()";
    if(typeName.compare("ForallLink")==0)
        symbol="∀";
    if(typeName.compare("ExistsLink")==0)
        symbol="∃";
    if(typeName.compare("VariableTypeNode")==0)
        symbol="VT";
    if(typeName.compare("ImplicationLink")==0)
        symbol="⇒";
    if(typeName.compare("EvaluationLink")==0)
        symbol="=";
    if(typeName.compare("InheritanceLink")==0)
        symbol="is";

    atomTypeSymbols.push_back(symbol);
}

void split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

void AtomTypes::AddSubTypes(string parentTypes)
{
    int typeNumber=atomTypeSubTypes.size();
    set<int>* subTypeSet=new set<int>();
    subTypeSet->insert(typeNumber);
    atomTypeSubTypes.push_back(subTypeSet);
    if (parentTypes.length()>0)
    {
        vector<string> parentTypeNames;
        split(parentTypes, ',', parentTypeNames);
        for (std::size_t p = 0; p < parentTypeNames.size(); p++)
        {
            string parentTypeName = MakeMixedCase(parentTypeNames[p]);
            int parentTypeNumber = ConvertTypeNameToNumber(parentTypeName);
            for (std::size_t t = 0; t < atomTypeSubTypes.size(); t++)
                if(atomTypeSubTypes[t]->count(parentTypeNumber)>0)
                    atomTypeSubTypes[t]->insert(typeNumber);
        }
    }
}

int AtomTypes::ConvertTypeNameToNumber(string typeName)
{
    for (std::size_t i = 0; i < atomTypeNames.size(); i++)
    {
        if (atomTypeNames[i].compare(typeName) == 0)
            return (int)i;
    }

    throw runtime_error("Invalid atomtype " + typeName + ". Make sure the atom_types.script file in the application directory is up to date.");
}

string AtomTypes::MakeMixedCase(string typeName)
{
    string s = typeName.substr(0, 1);
    for (std::size_t i = 1; i < typeName.length(); i++)
    {
        if (typeName[i] == '_')
        {
            i++;
            s += typeName[i];
        }
        else
            s += tolower(typeName[i]);
    }
    return s;
}

int AtomTypes::ConvertNodeTypeToAtomType(int nodeType)
{
    string type = nodeTypeNames[nodeType];
    for (std::size_t i = 0; i < atomTypeNames.size(); i++)
        if (atomTypeNames[i].compare(type) == 0)
            return (int)i;
    return -1;
}

int AtomTypes::ConvertLinkTypeToAtomType(int linkType)
{
    string type = linkTypeNames[linkType];
    for (std::size_t i = 0; i < atomTypeNames.size(); i++)
        if (atomTypeNames[i].compare(type) == 0)
            return (int)i;
    return -1;
}

bool AtomTypes::CheckAtomType(Vertex* vertex, int type, bool includeSubtypes)
{
    if (vertex->type < 0 || (std::size_t)vertex->type >= atomTypeNames.size())
    {
        throw runtime_error("Invalid atomtype. Make sure the atom_types.script file in the application directory is up to date.");
    }

    if (includeSubtypes)
        return atomTypeSubTypes[type]->count(vertex->type)>0;
    else
        return vertex->type == type;
}

bool AtomTypes::IsNode(string &typeName)
{
	for(std::size_t i=0;i<nodeTypeNames.size();i++)
		if(nodeTypeNames[i]==typeName)
			return true;

	return false;
}

void AtomTypes::Trim(string& line)
{
	int i;
	for (i = line.length() - 1; i >= 0; i--)
		if (line[i] != ' ' && line[i] != '\n' && line[i] != '\r')
			break;
	line.erase(i + 1);
}

void AtomTypes::Split(const string &s, char delim, vector<string> &elems)
{
    stringstream ss(s);
    string item;
    while(std::getline(ss, item, delim))
    {
        elems.push_back(item);
    }
}

