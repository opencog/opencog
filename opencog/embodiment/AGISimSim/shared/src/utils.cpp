/***************************************************************************
 *  Misc. utility functions, macros and classes.
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
#include "utils.h"
#include "simconfig.h"

#ifdef WIN32
#include <sys/timeb.h>
//------------------------------------------------------------------------------------------------------------
// Just to be compatible with linux/gcc gettimeofday function
int gettimeofday(timeval *tv, void *tz) {
   struct _timeb timebuffer;
   _ftime( &timebuffer );
   tv->tv_sec = timebuffer.time;
   tv->tv_usec = 1000*timebuffer.millitm;
   // For now, ignores timezone
   return 1;
}
#else
#include <sys/time.h>
#endif

/**
 * Time used as reference to set/get timestamps over the code
 */
static timeval referenceTime; 
static bool referenceTimeInitialized = false;


void initReferenceTime() {
    gettimeofday(&referenceTime, NULL);
    referenceTimeInitialized = true;
}

unsigned long getElapsedMillis() {
    assert(referenceTimeInitialized);
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return (currentTime.tv_sec-referenceTime.tv_sec)*1000 + (currentTime.tv_usec-referenceTime.tv_usec)/1000;
}

//------------------------------------------------------------------------------------------------------------
bool LoadTextFile (const string fname, string& dest)
{
	FILE *f = fopen(fname.c_str(), "rt");
	if (f == NULL)	{
		 LOG("CSWorld", 0, "Texture definition file not found.");
		return false;
	}
	fseek (f, 0L, SEEK_END);
	long fsize = ftell(f);
	fseek (f, 0L, SEEK_SET);
	
	char  *buf = new char[fsize+2];
#ifdef WIN32
	for(int i =0; i<fsize+2;i++){buf[i] = 0;} //for no garbage+null termination
#endif
	fread (buf, fsize, 1, f);
	fclose(f);
	dest = buf;	
	delete buf;
	
	return true;
}

//------------------------------------------------------------------------------------------------------------
string toupper(string k) {
	const char* data = k.c_str();
	char 	    buf[1000];
	
	//iterator<string,b> iii;	
	unsigned int i;
	for (i = 0; i < k.size(); i++)
		buf[i] = Isox(data[i]);
	buf[i] = 0;
		
	return buf;
}

//------------------------------------------------------------------------------------------------------------
std::string XMLembed (const std::string &elem, const std::string &pcdata) {
	return std::string ("<") + elem + "> " + pcdata + " </" + elem + ">";
};


//------------------------------------------------------------------------------------------------------------
void LocalObj3D::getPosition(double& _x, double& _y, double& _z) const {
	_x = x;
	_y = y;
	_z = z;
}

//------------------------------------------------------------------------------------------------------------
void LocalObj3D::getOrientation (double &_ox, double& _oy, double& _oz, double &_ophi) const {
	_ox=ox;
	_oy=oy;
	_oz=oz;
	_ophi=ophi;
}

//------------------------------------------------------------------------------------------------------------
void LocalObj3D::setOrientation (double _ox, double _oy, double _oz, double _ophi) {
	ox   = _ox;
	oy   = _oy;
	oz   = _oz;
	ophi = _ophi;
}

//------------------------------------------------------------------------------------------------------------
void LocalObj3D::setPosition(double _x, double _y, double _z) {
	x = _x;
	y = _y;
	z = _z;
	
	if (listener)  listener->OnUpdate(this);
}

//------------------------------------------------------------------------------------------------------------
LocalObj3D::LocalObj3D () 
					   :x(0), y(0), z(0), ox(0), oy(0), oz(0), ophi(0), listener(NULL)
{ }

//------------------------------------------------------------------------------------------------------------
void LocalObj3D::setListener (Listener* _l) {
	listener = _l;
}

//--- is it visibly printed character ------------------------------------------------------------------------
bool visible (char c) {
	return c != ' ' && c != '\r' && c != '\n' && c != '\t' && c != 0;
}

//------------------------------------------------------------------------------------------------------------
string i2str (int d) {
	char   temp[20];
	sprintf (temp,"%d",d);
	return temp;
}

//------------------------------------------------------------------------------------------------------------
bool exists(const char *fname) {
	FILE* f = fopen(fname,"rb");
	if (f == NULL)
		return false;
	fclose(f);
	return true;
}

//----Uppers characters --------------------------------------------------------------------------------------------------------
char Isox (char s) {
	if (s>96&&s<123) s-=32;
	return s;
}

//------------------------------------------------------------------------------------------------------------
nocase_string::nocase_string (const char* str) : string(str) { }
nocase_string::nocase_string (const nocase_string& str) : string(str.c_str()) { }
nocase_string::nocase_string (const string& str) : string(str) { }
nocase_string::nocase_string () { }

bool nocase_string::operator < (const char* rhs) {
	 LOG("C", 0, string(*this) + " < " + rhs);
	
	if (nocase_equal(this->c_str(),rhs)) {
		 LOG("C", 0, "no");
		return false; //equal
	}
	else {
		 LOG("C", 0, "maybe");
		return (strcmp(this->c_str(), rhs) < 0);
	}
}

//------------------------------------------------------------------------------------------------------------
bool nocase_string::operator < (const nocase_string& rhs) {
	return ((*this) < rhs.c_str());
}

//------------------------------------------------------------------------------------------------------------
bool nocase_string::operator < (const string& rhs) {
	return ((*this) < rhs.c_str());
}

//------------------------------------------------------------------------------------------------------------
bool nocase_string::operator == (const char* rhs) {
	return nocase_equal(this->c_str(),rhs);
}

//------------------------------------------------------------------------------------------------------------
bool nocase_string::operator == (const nocase_string& rhs) {
	return nocase_equal(this->c_str(),rhs.c_str());
}

//------------------------------------------------------------------------------------------------------------
bool nocase_string::operator == (const string& rhs) {
	return nocase_equal(this->c_str(),rhs.c_str());
}

//------------------------------------------------------------------------------------------------------------
bool nocase_string::operator != (const char* rhs)   { return !(*this == rhs); }
bool nocase_string::operator != (const nocase_string& rhs)  { return !(*this == rhs); }
bool nocase_string::operator != (const string& rhs) { return !(*this == rhs); }
void nocase_string::operator += (nocase_string s)   { string::operator+=(s); }
nocase_string nocase_string::operator+ (nocase_string s) { return (string)(*this) + (string)(s); }

//------------------------------------------------------------------------------------------------------------
bool nocase_equal (const char *s1, const char *s2) {
	int i;
	for (i = 0; s1[i] != 0 && s2[i] != 0;i++)
		if (Isox(s1[i]) != Isox(s2[i]))
			return false;
		
	return (s1[i] == 0 && s2[i] == 0);
}

//------------------------------------------------------------------------------------------------------------
StringTokenizer::StringTokenizer (const string &rStr, const string &rDelimiters) {
	string::size_type lastPos (rStr.find_first_not_of (rDelimiters, 0));
	string::size_type pos     (rStr.find_first_of     (rDelimiters, lastPos));
	
	while (string::npos != pos || string::npos != lastPos)	{
		push_back (rStr.substr (lastPos, pos - lastPos));
		lastPos = rStr.find_first_not_of (rDelimiters, pos);
		pos 	= rStr.find_first_of     (rDelimiters, lastPos);
	}
}

//------------------------------------------------------------------------------------------------------------
vector<string> StringTokenizer::WithoutEmpty() const {
	vector<string> ret;
	
	for (unsigned int i = 0; i < this->size(); i++)
		if (!(*this)[i].empty())
			ret.push_back((*this)[i]);

	return ret;	
}
