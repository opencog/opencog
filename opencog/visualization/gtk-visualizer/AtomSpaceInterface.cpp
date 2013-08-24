/*
 * opencog/visualizer/src/AtomSpaceInterface.cpp
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

#include "AtomSpaceInterface.h"

AtomSpaceInterface::AtomSpaceInterface(AtomTypes* atomTypes1)
{
	server = "localhost";
    atomCount = 1;
    atomTypes=atomTypes1;
}

AtomSpaceInterface::~AtomSpaceInterface()
{
}

void AtomSpaceInterface::SearchAtom(AtomFilter* atomFilter, vector<Vertex*>& foundVertices, bool &isFinished, int skip)
{
    string query="";
	if(atomFilter->uuid!=-numeric_limits<unsigned long>::max())
		query="list?handle=" + lexical_cast<string>(atomFilter->uuid);
	else if(atomFilter->name.size()>0)
		query="list?type=" + atomTypes->atomTypeNames[atomFilter->type] + "&name="+atomFilter->name;
	else
	{
		query="list?type=" + atomTypes->atomTypeNames[atomFilter->type];
		if(atomFilter->includeSubtypes)
			query+="&subtype=1";
		else
			query+="&subtype=0";
	}

	switch(atomFilter->sortOrder)
	{
		case 0:
			break;
		case 1:
			query+="&order=sti&ascend=0";
			break;
		case 2:
			query+="&order=lti&ascend=0";
			break;
		case 3:
			query+="&order=tv.s&ascend=0";
			break;
		case 4:
			query+="&order=tv.c&ascend=0";
			break;
		case 5:
			query+="&order=sti&ascend=1";
			break;
		case 6:
			query+="&order=lti&ascend=1";
			break;
		case 7:
			query+="&order=tv.s&ascend=1";
			break;
		case 8:
			query+="&order=tv.c&ascend=1";
			break;
	}

	query+="&max=49";

	if(skip>0)
		query+="&skip=" + lexical_cast<string>(skip);

	RetrieveVerticesFromCogServer(query, foundVertices, isFinished);
}

void AtomSpaceInterface::GetConnectedAtoms(Vertex* vertex, NodeFilter* nodeFilter, LinkFilter* linkFilter, vector<Vertex*>& connectedVertices)
{
    //example: http://localhost:17034/rest/0.2/list?handle=11616&handle=10584

	if(vertex->connectedHandles.size()==0)
		return;

	string query="list?";

	int size=vertex->connectedHandles.size();
	if(vertex->isEllipsisClicked)
	{
		if(size>500)
			size=500;
	}
	else
	{
		if(size>maxNumberOfConnectedVertices)
			size=maxNumberOfConnectedVertices;
	}

	for(int i=0; i<size;i++)
	{
		if(i>0)
			query+="&";
		query+="handle=" + vertex->connectedHandles[i];
	}
	query+="&max="+lexical_cast<string>(size);

	bool dummy;
	RetrieveVerticesFromCogServer(query, connectedVertices,dummy);

    std::size_t v = 0;
    while(v < connectedVertices.size())
    {
        Vertex* connectedVertex = connectedVertices[v];

        if (connectedVertex->isNode)
        {
            if (nodeFilter->PassesFilter(connectedVertex, atomTypes))
                v++;
            else
                connectedVertices.erase(connectedVertices.begin()+v);
        }
        else
        {
            if (linkFilter->PassesFilter(connectedVertex, atomTypes))
                v++;
            else
                connectedVertices.erase(connectedVertices.begin()+v);
        }
    }

    if(!vertex->isEllipsisClicked && vertex->connectedHandles.size()>(std::size_t)maxNumberOfConnectedVertices)
    {
    	Vertex *ellipsis = new Vertex();
    	ellipsis->isEllipsis=true;
    	ellipsis->name="and " + lexical_cast<string>(vertex->connectedHandles.size()-maxNumberOfConnectedVertices) + " more";
    	connectedVertices.push_back(ellipsis);
    }
}

void AtomSpaceInterface::RetrieveVerticesFromCogServer(string query,vector<Vertex*>& foundVertices, bool &isFinished)
{
	cout << query << endl;
	string result = GetResultFromCogServer(query);
	cout << result << endl;
	if (result.substr(0, 1) != "{")
		throw runtime_error("CogServer returned error: " + result);

	stringstream ssResult(result);
	map < string, string > mapResult;
	ParseRecord(ssResult, mapResult);

	if(mapResult["total"]=="0")
	{
		isFinished=true;
		return;
	}

	stringstream ssList(mapResult["result"]);
	vector < string > vectorResults;
	ParseList(ssList, vectorResults);
	for (std::size_t i = 0; i < vectorResults.size(); i++)
	{
		stringstream ssRecord(vectorResults[i]);
		map < string, string > mapRecord;
		ParseRecord(ssRecord, mapRecord);
		Vertex* vertex = new Vertex();
		if (mapRecord.find("name") != mapRecord.end())
		{
			RemoveQuotes (mapRecord["name"]);
			vertex->name = mapRecord["name"];
		}
		if (mapRecord.find("type") != mapRecord.end())
		{
			RemoveQuotes (mapRecord["type"]);
			vertex->type = atomTypes->ConvertTypeNameToNumber(
					mapRecord["type"]);
			vertex->isNode = atomTypes->IsNode(mapRecord["type"]);
		}
		if (mapRecord.find("sti") != mapRecord.end())
			vertex->STI = lexical_cast<short>(mapRecord["sti"]);

		if (mapRecord.find("lti") != mapRecord.end())
			vertex->LTI = lexical_cast<short>(mapRecord["lti"]);

		if (mapRecord.find("handle") != mapRecord.end())
			vertex->uuid = lexical_cast < UUID > (mapRecord["handle"]);

		if (mapRecord.find("truthvalue") != mapRecord.end())
		{
			vertex->truthValue = mapRecord["truthvalue"];
			stringstream ssTruth(mapRecord["truthvalue"]);
			map < string, string > mapTruth;
			ParseRecord(ssTruth, mapTruth);
			string strTruth = ""; //TODO composite
			if (mapTruth.find("simple") != mapTruth.end())
				strTruth = mapTruth["simple"];

			if (mapTruth.find("indefinite") != mapTruth.end())
				strTruth = mapTruth["indefinite"];

			if (mapTruth.find("count") != mapTruth.end())
				strTruth = mapTruth["count"];

			if (strTruth.size() > 0)
			{
				stringstream ssSimpleTruth(strTruth);
				map < string, string > mapSimpleTruth;
				ParseRecord(ssSimpleTruth, mapSimpleTruth);
				if (mapSimpleTruth.find("str") != mapSimpleTruth.end())
					vertex->strength = lexical_cast<double>(mapSimpleTruth["str"]);

				if (mapSimpleTruth.find("conf") != mapSimpleTruth.end())
					vertex->confidenceValue = lexical_cast<double>(mapSimpleTruth["conf"]);
			}
		}

		vertex->connectedHandles.clear();
		if (mapRecord.find("outgoing") != mapRecord.end())
		{
			stringstream ssOutgoing(mapRecord["outgoing"]);
			ParseList(ssOutgoing, vertex->connectedHandles);
		}
		if (mapRecord.find("incoming") != mapRecord.end())
		{
			stringstream ssIncoming(mapRecord["incoming"]);
			ParseList(ssIncoming, vertex->connectedHandles);
		}

		atomTypes->CheckAtomType(vertex, 1, false); //check if type is known

		foundVertices.push_back(vertex);
	}

	isFinished=(lexical_cast<int>(mapResult["skipped"])+vectorResults.size()>=(std::size_t)lexical_cast<int>(mapResult["total"]));
}

void AtomSpaceInterface::UpdateAtom(UUID uuid, short lti, short sti, string truthValue)
{
	string queryString="atom?handle=" + lexical_cast<string>(uuid);
	string postBody= "{ \"lti\":" + lexical_cast<string>(lti)+ ",\"sti\":" + lexical_cast<string>(sti) + ",\"truthvalue\":"+ truthValue + "}";
	cout << queryString << endl;
	cout << postBody << endl;
	string result = AtomSpaceInterface::PostToCogServer( queryString, postBody);
	cout << result << endl;
}

string AtomSpaceInterface::GetResultFromCogServer(string queryString)
{

	boost::asio::io_service io_service;
	tcp::socket socket(io_service);
	ConnectToCogServer(io_service, socket);

	// Form the request
	boost::asio::streambuf request;
	string path = "/rest/0.2/" + queryString;
	std::ostream request_stream(&request);
	request_stream << "GET " << path << " HTTP/1.0\r\n";
	request_stream << "Host: " << server << "\r\n";
	request_stream << "Accept: */*\r\n";
	request_stream << "Connection: close\r\n\r\n";

	// Send the request.
	boost::asio::write(socket, request);

	// Read the response status line
	boost::asio::streambuf response;
	boost::asio::read_until(socket, response, "\r\n");

	// Check that response is OK.
	istream response_stream(&response);
	string http_version;
	response_stream >> http_version;
	unsigned int status_code;
	response_stream >> status_code;
	string status_message;
	getline(response_stream, status_message);
	if (!response_stream || http_version.substr(0, 5) != "HTTP/")
	{
		throw runtime_error("CogServer returned an invalid response");
	}
	if (status_code != 200)
	{
		throw runtime_error("CogServer returned status code "+ lexical_cast<string>(status_code));
	}

	boost::system::error_code error;
	boost::asio::read(socket, response, boost::asio::transfer_all(), error);
    if (error != boost::asio::error::eof)
		throw boost::system::system_error(error);

	//Remove the response headers.
	string header;
	while (getline(response_stream, header) && header != "\r")
	{
	}

	//
	char c;
	string ret = "";
	while (!response_stream.eof())
	{
		response_stream.get(c);
		ret+= c;
	}

	return ret;
}

string AtomSpaceInterface::PostToCogServer(string queryString,string postBody)
{

	boost::asio::io_service io_service;
	tcp::socket socket(io_service);
	ConnectToCogServer(io_service,socket);

	// Form the request.
	boost::asio::streambuf request;
	string path = "/rest/0.2/" + queryString;
	std::ostream request_stream(&request);
	request_stream << "POST " << path << " HTTP/1.0\r\n";
	request_stream << "Host: " << server << "\r\n";
	request_stream << "Accept: */*\r\n";
	request_stream << "Connection: close\r\n";
	request_stream << "Content-Type: text/html; charset=UTF-8\r\n";
	request_stream << "Content-Length: "+lexical_cast<string>(postBody.size())+"\r\n\r\n";
	request_stream << postBody+ "\r\n";

	// Send the request.
	boost::asio::write(socket, request);

	// Read the response status line
	boost::asio::streambuf response;



//TODO	[1335064466] [error] [client 127.0.0.1] POST /rest/0.2/atom: Error 577: Internal Server Error

	boost::system::error_code error;
	boost::asio::read(socket, response, boost::asio::transfer_at_least(1), error);
	istream response_stream(&response);
	cout<<response_stream<<endl;
	string ret="";
/*
	boost::asio::read_until(socket, response, "\r\n");

	// Check that response is OK.
	istream response_stream(&response);
	string http_version;
	response_stream >> http_version;
	unsigned int status_code;
	response_stream >> status_code;
	string status_message;
	getline(response_stream, status_message);
	if (!response_stream || http_version.substr(0, 5) != "HTTP/")
	{
		throw runtime_error("CogServer returned an invalid response");
	}
	if (status_code != 200)
	{
		throw runtime_error("CogServer returned status code "+ lexical_cast<string>(status_code));
	}

	boost::system::error_code error;
	boost::asio::read(socket, response, boost::asio::transfer_all(), error);
    if (error != boost::asio::error::eof)
		throw boost::system::system_error(error);

	//Remove the response headers.
	string header;
	while (getline(response_stream, header) && header != "\r")
	{
	}

	char c;
	string ret = "";
	while (!response_stream.eof())
	{
		response_stream.get(c);
		ret+= c;
	}
*/
	return ret;

}

void AtomSpaceInterface::ConnectToCogServer(boost::asio::io_service &io_service, tcp::socket &socket)
{
	// Get a list of endpoints corresponding to the server name.
	tcp::resolver resolver(io_service);
	tcp::resolver::query query(server, "17034");
	tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
	tcp::resolver::iterator end;
	// Try each endpoint until we successfully establish a connection.
	boost::system::error_code errorConnecting =
			boost::asio::error::host_not_found;
	while (errorConnecting && endpoint_iterator != end)
	{
		socket.close();
		socket.connect(*endpoint_iterator++, errorConnecting);
	}
	if (errorConnecting)
		throw boost::system::system_error(errorConnecting);
}

void AtomSpaceInterface::AssertChar(char c, char d, stringstream &ss)
{
	if(c!=d)
	{
		int count=20;
		string context="";
		while(!ss.eof() && count>0)
		{
			context+=ss.get();
			count--;
		}
		throw runtime_error("JSON parse error: Unexpected character in input " + lexical_cast<string>(c) + " instead of " + lexical_cast<string>(d) + " near " + context);
	}
}

void AtomSpaceInterface::SkipWhiteSpace(stringstream &ss, char &c)
{
	do
	{
		ss.get(c);
	} while(c==' '  || c=='\r'  || c=='\n');
}

void AtomSpaceInterface::ParseString(stringstream &ss, string &str)
{
	char c;
	SkipWhiteSpace(ss,c);
	AssertChar(c,'\"',ss);
	str+=c;
	ss.get(c);
	while(c!='\"')
	{
		if(ss.eof())
			throw runtime_error("JSON parse error: missing closing \"");
		str+=c;
		ss.get(c);
	}
	AssertChar(c,'\"',ss);
	str+=c;
}

void AtomSpaceInterface::ParseAny(stringstream &ss, string &str)
{
	int bracketLevel=0;
	char c;

	c=ss.peek();
	while(bracketLevel>0 || (c!=',' && c!='}' && c!=']'))
	{
		if(c=='\"')
		{
			string s;
			ParseString(ss,s);
			str+=s;
		}
		else
		{
			ss.get(c);
			str+=c;
			if(c=='{' || c=='[' )
			{
				bracketLevel++;
			}
			else if(c=='}' || c==']' )
			{
				bracketLevel--;
			}
			else
				if(ss.eof())
					throw runtime_error("JSON parse error: missing closing } or ]");

		}
		c=ss.peek();
	}
}

void AtomSpaceInterface::ParseField(stringstream &ss, string &fieldName, string &fieldContent)
{
	ParseString(ss,fieldName);
	char c;
	ss.get(c);
	AssertChar(c,':',ss);
	ParseAny(ss,fieldContent);
}

void AtomSpaceInterface::RemoveQuotes(string &s)
{
	s.erase(0,1);
	s.erase(s.size()-1);
}

void AtomSpaceInterface::Trim(string &s)
{
	while(s.size()>0 && s.substr(0,1)==" ")
		s.erase(0,1);
	while(s.size()>0 && s.substr(s.size()-1,1)==" ")
			s.erase(s.size()-1,1);
}

void AtomSpaceInterface::ParseRecord(stringstream &ss,map<string,string> &mapRecord)
{
	char c;
	SkipWhiteSpace(ss,c);
	AssertChar(c,'{',ss);
	c=',';
	while(c==',')
	{
		if(ss.peek()=='}')
			ss.get(c);
		else
		{
			string fieldName;
			string fieldContent;
			ParseField(ss, fieldName, fieldContent);
			RemoveQuotes(fieldName);
			mapRecord[fieldName]=fieldContent;
			ss.get(c);
		}
	}
	AssertChar(c,'}',ss);
}

void AtomSpaceInterface::ParseList(stringstream &ss,vector<string> &vector)
{
	char c;
	SkipWhiteSpace(ss,c);
	AssertChar(c,'[',ss);
	c=',';
	while(c==',')
	{
		if(ss.peek()==']')
			ss.get(c);
		else
		{
			string element;
			ParseAny(ss, element);
			//Trim(element);
			vector.push_back(element);
			ss.get(c);
		}
	}
	AssertChar(c,']',ss);
}

AtomFilter::AtomFilter()
{
	name="";
    type = 1; //atom
    includeSubtypes = true;
    uuid=-numeric_limits<UUID>::max();
    sortOrder=0;
}

AtomFilter::~AtomFilter()
{
}

NodeFilter::NodeFilter()
{
    type = 2; //node
    includeSubtypes = true;
    minimumSTI = -numeric_limits<short>::max();
    minimumLTI = -numeric_limits<short>::max();
    minimumStrength = -numeric_limits<double>::max();
    minimumConfidenceValue = -numeric_limits<double>::max();
}

NodeFilter::~NodeFilter()
{
}

bool NodeFilter::PassesFilter(Vertex* vertex, AtomTypes* atomTypes)
{
    return vertex->STI>=minimumSTI
        && vertex->LTI >= minimumLTI
        && vertex->strength >= minimumStrength
        && vertex->confidenceValue >= minimumConfidenceValue
        && atomTypes->CheckAtomType(vertex, type, includeSubtypes);
}

LinkFilter::LinkFilter()
{
    type = 3; //link
    includeSubtypes = true;
    minimumSTI = -numeric_limits<short>::max();
    minimumLTI = -numeric_limits<short>::max();
    minimumStrength = -numeric_limits<double>::max();
    minimumConfidenceValue = -numeric_limits<double>::max();
}

LinkFilter::~LinkFilter()
{
}

bool LinkFilter::PassesFilter(Vertex* vertex, AtomTypes* atomTypes)
{
    return vertex->STI >= minimumSTI
        && vertex->LTI >= minimumLTI
        && vertex->strength >= minimumStrength
        && vertex->confidenceValue >= minimumConfidenceValue
        && atomTypes->CheckAtomType(vertex, type, includeSubtypes);
}

