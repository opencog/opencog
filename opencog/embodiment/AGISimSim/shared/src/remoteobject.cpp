/***************************************************************************
 *  RemoteObject class implementation.
 * 
 *  Project: AgiSim
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *																			
 *	19.01.06	FP	formatting  
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA. */
 
 
#include "simcommon.h"
#include "remoteobject.h"

//------------------------------------------------------------------------------------------------------------
RemoteObject::RemoteObject (bool _liberal) : liberal(_liberal)
{ }
RemoteObject::~RemoteObject()
{ }

//------------------------------------------------------------------------------------------------------------
bool RemoteObject::Set (const nocase_string property, const float value) {
	char temp[80];
	sprintf(temp, "%f", value);
	
	return Set(property, nocase_string(temp));
}

bool RemoteObject::Set (const nocase_string property, const int value) {
	return Set(property, toString(value));
}

bool RemoteObject::Set (const nocase_string property, const nocase_string value) {
	if (value.empty())  return Set(property, "0"); //Probably has been NULL-assigned, when 0 was meant.		

	if (STLhas(privateSetProperties, toupper(property))) {
		 LOG("RemoteObject", 1, property + " SET access is private.")
		return false;
	}
	
	if (liberal || STLhas (validValues[property], value)) {
		 LOG("RemoteObject", 4, property + " = " + value);
		properties[toupper(property)] = value;
		if (STLhas(listeners, property)) listeners[property]->OnUpdate(value.c_str());
		return true;
	}
	else {
		 LOG("RemoteObject", 2, property + " couldn't be set to " + value)
		return false;
	}
}

//------------------------------------------------------------------------------------------------------------
bool RemoteObject::Get(const nocase_string property, nocase_string& value) const {
	if (STLhas(privateGetProperties, toupper(property))) {
		 LOG("RemoteObject", 1, property + " GET access is private.")
		return false;
	}		
		
	map<nocase_string, nocase_string>::const_iterator  i = properties.find(toupper(property));
	if (i != properties.end()) {
		value = i->second;
		return true;
	}
	else {
		 LOG("RemoteObject", 1, property + " was not found.")
		value = "";
		return false;
	}
}

bool RemoteObject::Get (const nocase_string property, int& value) const {
	nocase_string s;
	bool 		  ret = Get(property, s);
	
	if (ret) {
		value = atoi(s.c_str());
		return true;
	}
	else {
		value = 0;
		return false;
	}
}

bool RemoteObject::Get (const nocase_string property, float& value) const {
	nocase_string s;
	bool          ret = Get(property, s);
	
	if (ret) {
		value = (float)(atof(s.c_str()));
		return true;
	}
	else {
		value = 0.0f;
		return false;
	}
}

nocase_string RemoteObject::Get(const nocase_string property) const {
	nocase_string s;
	Get (property, s);
	return s;	
}

//------------------------------------------------------------------------------------------------------------
void RemoteObject::DefineValidValues (const nocase_string property, const set<nocase_string> values){
	validValues[toupper(property)] = values;
}

//------------------------------------------------------------------------------------------------------------
void RemoteObject::AddValidValue (const nocase_string property, const nocase_string value) {
	validValues[toupper(property)].insert(value);
}

//------------------------------------------------------------------------------------------------------------
void RemoteObject::SetPropertyListener (const nocase_string property, shared_ptr<Listener> listener) {
	listeners[toupper(property)] = listener;
}

//------------------------------------------------------------------------------------------------------------ 
string RemoteObject::PrintList() const
{
	string ret;
	for (map<nocase_string, nocase_string>::const_iterator	i = properties.begin();	i!= properties.end(); i++) {
		string thispair = i->first + " = " + i->second;
		 LOG("RemoteObject", 3, thispair);
		ret +=thispair;
	}
	return ret;
}

//------------------------------------------------------------------------------------------------------------
string RemoteObject::AsXML() const {
	string ret;
	for (map<nocase_string, nocase_string>::const_iterator	i = properties.begin();	i!= properties.end(); i++) {
		string thispair = XMLembed (i->first, i->second);
		 //		LOG("RemoteObject", 4, thispair);
		ret +=thispair;
	}
	return ret;
}

//------------------------------------------------------------------------------------------------------------
void RemoteObject::MakePrivateSet(const nocase_string property) {
	privateSetProperties.insert (toupper(property));
}

//------------------------------------------------------------------------------------------------------------
void RemoteObject::MakePrivateGet (const nocase_string property)
{
	privateGetProperties.insert (toupper(property));
}
