/***************************************************************************
 *  VOS property class.
 * 
 *  Project: AgiSim
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *																			
 *	19.01.06	FP	formatting  
 ****************************************************************************/

/*	This file has been altered to suit AgiSim
	by Ari Heljakka / Novamente LLC.
	
	The original copyright and license follows:

    This file is part of the Virtual Object System of
    the Interreality project (http://interreality.org).

    Copyright (C) 2001, 2002 Peter Amstutz

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA

    Peter Amstutz <tetron@interreality.org>
    Additions by Reed Hedges <reed@zerohour.net> marked with initials "rh" and date. */

#include <iostream>
#include <fstream>
#include <stdarg.h>
#include <stdio.h>

#include <property.h>

#ifdef WIN32
#include <vsscanf.h>
#endif

#define LIST_SEPERATORS " \t\n"

using namespace std;

Property::Property()
{ };

//------------------------------------------------------------------------------------------------------------
void Property::replace (const char* fmt, size_t maxlen, ...) {
    char*    s = new char[maxlen]; // could be determined by parsing format string maybe
    va_list  ap;
    va_start (ap, maxlen);
#ifdef WIN32
    _vsnprintf(s, sizeof(s), fmt, ap);
#else
    vsnprintf(s, sizeof(s), fmt, ap);
#endif
    va_end   (ap);
    replace  (s, "?");
    delete[] s;
}

void Property::replace (const char* fmt, const char* type, size_t maxlen, ...) {
    char*    s = new char[maxlen]; // could be determined by parsing format string maybe
    va_list  args;
    va_start (args, maxlen);
#ifdef WIN32
    _vsnprintf(s, sizeof(s), fmt, args);
#else
    vsnprintf(s, sizeof(s), fmt, args);
#endif
    va_end   (args);
    replace  (s, type);
    delete[] s;
}

void Property::replace (const char* str) {
    std::string s(str);
    replace (s);
}

void Property::replace (double x) {
    replace ("%f", "float", 64, x);
}

void Property::replace (int i) {
    char s[64];
#ifdef WIN32
    _snprintf (s, sizeof(s), "%i", i);
#else
    snprintf (s, sizeof(s), "%i", i);
#endif
    replace  (s, "integer");
}

void Property::replace (bool b) {
    if(b)  replace ("true" , "boolean");
    else   replace ("false", "boolean");
}

void Property::replace (double x, double y) {
    replace ("%f %f", "list: float", 64, x, y);
}

void Property::replace (double x, double y, double z) {
    replace ("%f %f %f", "list: float", 64, x, y, z);
}


void Property::replace (double x, double y, double z, double w) {
    replace ("%f %f %f %f", "list: float", 64, x, y, z, w);
}


//------------------------------------------------------------------------------------------------------------
void Property::read (const char* fmt, ...) {
    string data;
    read (data);
    va_list  ap;
    va_start (ap, fmt);
    vsscanf  ((char*)data.c_str(), (char*)fmt, ap);
    va_end   (ap);
}

void Property::read (int& i) {
    read("%i", &i);
}

void Property::read (bool& b) {
    if(read() == "true" || read() == "yes")  b = true;
    else							         b = false;
}

void Property::read (double& x) {
    read("%lf", &x);
}

void Property::read (double& x, double& y) {
    read("%lf %lf", &x, &y);
}

void Property::read (double& x, double& y, double& z) {
    read("%lf %lf %lf", &x, &y, &z);
}

void Property::read (double& x, double& y, double& z, double& w) {
    read("%lf %lf %lf %lf", &x, &y, &z, &w);
}

void Property::read (float& x) {
    read("%f", &x);
}

void Property::read (float& x, float& y) {
    read("%f %f", &x, &y);
}

void Property::read (float& x, float& y, float& z) {
    read("%f %f %f", &x, &y, &z);
}


void Property::read (float& x, float& y, float& z, float& w) {
    read("%f %f %f %f", &x, &y, &z, &w);
}

void Property::read (vector<float>& vec) {
    string data;
    read (data);
    size_t end   = 0;
    size_t start = 0;
	
    for(size_t i = 0; end != data.npos; i++) { //!!! beim 2. Argument habe ich klammern gelöscht...
        start = data.find_first_not_of (LIST_SEPERATORS, end);
        if(start >= data.npos)
            return;
        end = data.find_first_of(LIST_SEPERATORS, start);

        if(i == vec.size())  vec.push_back(atof(data.substr(start, (end-start)).c_str()));  // i starts at 0, so == will work
        else                 vec[i] = atof(data.substr(start, (end-start)).c_str());
    }
}

void Property::replace(vector<float>& vec) {
    string data;
    char   tmp[32];
	
    for (vector<float>::const_iterator i = vec.begin(); i != vec.end(); i++) {
#ifdef WIN32
        _snprintf (tmp, sizeof(tmp), "%f ", *i);
#else
        snprintf (tmp, sizeof(tmp), "%f ", *i);
#endif
        data += tmp;
    }
    replace (data, "list: float");
}

void Property::read (vector<int>& vec) {
    string data;
    read  (data);
    size_t end   = 0;
    size_t start = 0;
	
    for(size_t i = 0; (end != data.npos); i++) {
        start = data.find_first_not_of(LIST_SEPERATORS, end);
        if(start == data.npos)
            return;
        end = data.find_first_of(LIST_SEPERATORS, start);
        if(i == vec.size())  vec.push_back(atoi(data.substr(start, (end-start)).c_str()));  // i starts at 0, so == will work         
        else                 vec[i] = atoi(data.substr(start, (end-start)).c_str());
    }
}

void Property::replace(vector<int>& vec) {
    string data;
    char   tmp[32];
    for (vector<int>::const_iterator  i = vec.begin(); i != vec.end(); i++) {
#ifdef WIN32
        _snprintf (tmp, sizeof(tmp), "%i ", *i);
#else
        snprintf (tmp, sizeof(tmp), "%i ", *i);
#endif
        data += tmp;
    }
    replace (data, "list: int");
}


/*void Property::writeToFile(const string& filename, int offset, int length)
{
    ofstream f(filename.c_str(), ios::out|ios::binary);
    if(!f.is_open() || f.bad())
        throw runtime_error(string("Error opening file \"") + filename + "\" for writing.");
    string s;
    read(s, offset, length);
    f.write(s.c_str(), s.length());
    if(f.bad())
        throw runtime_error(string("Error writing to file \"")+filename+"\"");
}

void Property::readFromFile(const string& filename, const string& type)
{
    ifstream f(filename.c_str(), ios::in|ios::binary);
    if(!f.is_open() || f.bad())
        throw runtime_error(string("Error opening file \"") + filename + "\" for reading.");
    f.seekg(0, ios::end);
    unsigned long size = f.tellg();
    f.seekg(0, ios::beg);
    char* buffer = new char[size+1];
    f.read(buffer, size);
    f.close();
    string val(buffer, size);
    if(type == "")
        replace(val, getDataType());
    else
        replace(val, type);
    delete[] buffer;
}*/

//------------------------------------------------------------------------------------------------------------
int Property::getLength() {
    boost::recursive_mutex::scoped_lock lk(data_mutex);
    return data.size();
}

void Property::read(std::string& target) {
    boost::recursive_mutex::scoped_lock  lk (data_mutex);
    target = data;
}


void Property::read(std::string& target, int start, int length)
{
    boost::recursive_mutex::scoped_lock  lk (data_mutex);

    if (length < 0) {
        if(start == 0) target = data;
        else           target = data.substr(start);
    } 
	else target = data.substr(start, length);
}


std::string Property::read()
{
    boost::recursive_mutex::scoped_lock  lk (data_mutex);
    return data;
}


std::string Property::read(int start, int length) {
    boost::recursive_mutex::scoped_lock lk(data_mutex);

    if(length < 0) {
        if(start == 0)  return data;
        else 			return data.substr(start);
    } 
	else  return data.substr(start, length);
}

//------------------------------------------------------------------------------------------------------------
void Property::write(int start, const std::string& newdata)
{
    boost::recursive_mutex::scoped_lock lk(data_mutex);

    std::string oldvalue = data;

    data.replace(start, newdata.size(), newdata);
}


void Property::replace(const std::string& newdata, const std::string& newtype) {
    boost::recursive_mutex::scoped_lock lk(data_mutex);

    data 	 = newdata;
    datatype = newtype;
}

const std::string Property::getDataType() {
    boost::recursive_mutex::scoped_lock  lk (data_mutex);
    return datatype;
}

/*const std::string Property::getVOSType()
{
    return "property:property";
}*/
