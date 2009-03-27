/*
 * opencog/embodiment/AGISimSim/shared/include/XMLNode.cpp
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Ari A. Heljakka
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

#include "utils.h"
#include "XMLNode.h"

//------------------------------------------------------------------------------------------------------------
void CleanSpace (string& s) {
	const char *str = s.c_str();
	int i = s.size()-1;
	while (i >= 0 && (str[i] == ' ' || str[i] == '\t' || str[i] == '\n' || str[i] == '\r'))
		i--;
	if (i < (int)(s.size() - 1))
		s = s.substr(0, i+1);
}

//------------------------------------------------------------------------------------------------------------
XMLNode::XMLNode()
{}

//------------------------------------------------------------------------------------------------------------	
void XMLNode::SetName (string n) {
	tagdata.name = n;
}

//------------------------------------------------------------------------------------------------------------
XMLNode::XMLNode (const string& rawdata, XMLNode* _super) 
				 : ok(false), bytesconsumed(0), super(_super) {
	bytesconsumed = ExpandLevel(rawdata,tagdata,sub,this);
	ok = bytesconsumed > 0;
}

//------------------------------------------------------------------------------------------------------------
XMLNode::~XMLNode()
{
	for (vector<XMLNode*>::iterator x = sub.begin(); x != sub.end(); x++)
		delete (*x);
}

//------------------------------------------------------------------------------------------------------------
XMLNode* XMLNode::Super() {
	return super;
}

//------------------------------------------------------------------------------------------------------------
const vector<XMLNode*>& XMLNode::Sub() {
	return sub;
}

//------------------------------------------------------------------------------------------------------------
vector<XMLNode*> XMLNode::StealSub() {
	vector<XMLNode*>  ret = sub;
	sub.clear ();  //Only ret has now the dynamic pointers.

	return ret;
}

//------------------------------------------------------------------------------------------------------------
vector<XMLNode*>& XMLNode::UnSafeSub() {
	return sub;
}

//------------------------------------------------------------------------------------------------------------
const XMLtag& XMLNode::TagData() {
	return tagdata;
}

#define MAX_NODE_NAME_LENGTH 255
//------------------------------------------------------------------------------------------------------------
int XMLNode::ExpandLevel (const string& _source, XMLtag& _contents, vector<XMLNode*>&  _sub, XMLNode* _super) {
	char 	 *str = const_cast<char*>(_source.c_str());
	char 	 temp[MAX_NODE_NAME_LENGTH+1];
	int 	 t;
	XMLNode* newnode;
	bool 	 ends = false;
	
	try	{
		nextc(&str);
		if (*str != '<') {
			throw (string("Start tag: ") + toString(*str));
			return 0;
		}
		str++;

		if (*nextc(&str) == '/') { //Must be BEGIN tag
			throw string("End tag where BEGIN tag was expected");
			return 0;
		}

		for (nextc(&str), t = 0;*str!='/' && *str!='>' && visible(*str)  &&  t < MAX_NODE_NAME_LENGTH;str++,t++)
			temp[t] = *str;
		temp[t] = 0;

  		if (temp[t] != '/') {
			//NAME
			_contents.name = temp;
			CleanSpace(_contents.name);
	
			nextc(&str);
	
			//ARGS
			if (*str == '>')
				str++;
			else { //Something before '>'	
				char c = *nextc(&str);
				if (c != '>'/* && c!='/'*/) { //If arguments				
					bool more;
					do {
						more = ExpandArgument(&str, _contents.arguments,&ends); //*str is ALSO updated 
					} while (more);
				}
			}
		}
		else ends = true;

		if (ends) {
			if (*str == '/') {
				str++; 	//Pass /
				nextc(&str);
				str++; 	//Pass >
				nextc(&str);
			}

			goto out;
		}

		nextc(&str);
		
		//Containments
		if (*str != '<') { //Contains text
			for (;*str != '<'  &&  *str != 0;str++)
				_contents.textcontent += *str; //A char at a time.

			CleanSpace(_contents.textcontent);
		}
		else if (*(str+1) != '/') { //Contains other tags
			do {
				newnode = new XMLNode(string(str), _super);
				str += newnode->bytesconsumed;
				nextc(&str);
				if (newnode->ok)
					_sub.push_back(newnode);
				else { delete newnode; break; }
			} while (newnode->ok);
		}

		if (*nextc(&str) != '<') {
			throw string("END tag not found 1.");
			return 0;
		}
		
		str++;
		if (*nextc(&str) != '/') {
			throw string("END tag not found 2.");
			return 0;
		}
		
		str++;
		for (t = 0,nextc(&str);visible(*str) && *str!='>' &&  t < MAX_NODE_NAME_LENGTH;str++,t++)
			temp[t] = *str;
		temp[t] = 0;

		if (!nocase_equal(temp, _contents.name.c_str())) { //Open/close mismatch!
			throw string(string("BEGIN/END mismatch: ")+temp + " / " + _contents.name);
			return 0;
		}

		nextc(&str);
		str++; //str points beyond this tag now.
	}
	catch(...) { return 0; }
out:
	return str - _source.c_str();
}

//------------------------------------------------------------------------------------------------------------
char* XMLNode::nextc (char **ptr) {
	while ((**ptr) == '\r' || (**ptr) == '\n' || (**ptr) == ' ' || (**ptr) == '\t' || (**ptr) == 0)
		if ((**ptr) != 0)
			(*ptr)++;
		else throw string("XMLNode::nextc - character query unsuccessful");
	return *ptr;
}

//------------------------------------------------------------------------------------------------------------
/*	input-ptr must point to 1st char of an argument
	argument is placed in arg
	if more args follow, ptr ends up pointing to the 1st char of the next one
	otherwise ptr will point just beyond the '>'!
	return: true if more args follow*/
//------------------------------------------------------------------------------------------------------------
bool XMLNode::ExpandArgument (char** ptr, map<nocase_string,nocase_string>& arg,bool* ends) {
	char temp[256];
	int  t;
	bool moreargs = true;

	nextc(ptr);

	for (t = 0; visible(**ptr) && **ptr != '=' && **ptr != '/'; (*ptr)++,t++)
		temp[t] = **ptr;
	temp[t] = 0;

	nocase_string name(temp);
	nextc(ptr); //Now at '='
	*ends = ('/' == **ptr);
		
	if (**ptr != '=')	return false;
	(*ptr)++;
	nextc(ptr);

	if (**ptr == '\"')	(*ptr)++;

	for (t = 0; (**ptr == ' ' || visible(**ptr)) && **ptr != '\"' && **ptr != '>'; (*ptr)++,t++)
		temp[t] = **ptr;
	temp[t] = 0;

	if (**ptr == '\"')	(*ptr)++;

	char ender = *nextc(ptr);
	*ends = false;

	if (ender == '>' || ender == '/') {
		if (ender == '/') {
			(*ptr)++;
			*ends = true;
		}
		(*ptr)++;
		moreargs = false;
	}
	else moreargs = true;

	nocase_string value(temp);

	CleanSpace (name);
	CleanSpace (value);
	arg[name] = value;

	return moreargs;
}

//------------------------------------------------------------------------------------------------------------
bool XMLNode::Add (const XMLNode& node) {
	sub.push_back (new XMLNode(node));

	return true;
}

//------------------------------------------------------------------------------------------------------------
XMLNode::XMLNode (const XMLNode& rhs) {
	bytesconsumed = rhs.bytesconsumed;
	ok 			  = rhs.ok;
	super 		  = rhs.super;
	tagdata 	  = rhs.tagdata;

	for (int i = 0; i < (int)(rhs.sub.size()); i++)
		sub.push_back(rhs.sub[i]);
}

//------------------------------------------------------------------------------------------------------------
template<typename T>
XMLvalue<T>::~XMLvalue()
{ }

//------------------------------------------------------------------------------------------------------------
template<typename T>
std::string XMLvalue<T>::AsXML() const {
	return XMLembed(name, Value2String());
}

//------------------------------------------------------------------------------------------------------------
template<typename T>
XMLvalue<T>::XMLvalue (std::string _name, T _value)
					  : name(_name), value(_value)
{ }

//------------------------------------------------------------------------------------------------------------
IntXMLValue::IntXMLValue (std::string name, int _value)
						 : XMLvalue<int>(name, _value)
{ }

//------------------------------------------------------------------------------------------------------------
StringXMLValue::StringXMLValue (std::string name, std::string _value)
							   : XMLvalue<string>(name, _value)
{ }

//------------------------------------------------------------------------------------------------------------
FloatXMLValue::FloatXMLValue (std::string name, float _value)
							 : XMLvalue<float>(name, _value)
{ }

//------------------------------------------------------------------------------------------------------------
std::string IntXMLValue::Value2String () const {
	return toString(value);
}

//------------------------------------------------------------------------------------------------------------
std::string FloatXMLValue::Value2String () const {
	char temp[100];
	sprintf(temp, "%.2f", value);
	return temp;
}

//------------------------------------------------------------------------------------------------------------
std::string StringXMLValue::Value2String () const
{
	return value;
}
