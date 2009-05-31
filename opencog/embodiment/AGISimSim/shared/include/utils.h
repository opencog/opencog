/*
 * opencog/embodiment/AGISimSim/shared/include/utils.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari A. Heljakka
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


#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#ifdef WIN32
#include <winsock.h>
int gettimeofday(timeval *tv, void *tz);
#endif

/**
 * Initializes the reference time that will be used for getting current elapsed times
 */
void initReferenceTime();
/**
 * Gets the elapsed time (in milliseconds) since the reference time initialized with
 * initReferenceTime() function. The initReferenceTime() function must be called before
 * this function be called by the first time.
 */
unsigned long getElapsedMillis();

#ifndef WIN32
#include <string.h>
#include <dirent.h>
#  ifndef __APPLE__
// #include <libiberty.h>
#  endif
#endif

#include <map>
#include <vector>
#include <string>
#include <list>
#include <set>


typedef unsigned char byte;
typedef unsigned short int word;
typedef unsigned long int dword;

#define STLhas(container, entity) ((container).find(entity) != (container).end())

#ifdef WIN32
#define SLASH "\\"
#else
#define SLASH "/"
#endif

/** Checks whether a file exists */
bool exists(const char *fname);

/** Checks whether the character is printable */
bool visible(char c);

/** Checks whether the strings are equal, when case is ignored. */
bool nocase_equal(const char *s1, const char *s2);

/** Converts a character to upper case (also some Scandinavian characters) */
char Isox(char s);

/**
 *  * Returns a string from the given argument by using the << operator
 *   */
template <typename T>
std::string toString(T data)
{
	std::ostringstream oss;
	std::string result;
    oss << data;
    return oss.str();
}

/** Converts a string to upper case by return value */
std::string toupper(std::string k);

/** Load the contents of a textfile \param fname to \param dest. */
bool LoadTextFile(const std::string fname, std::string& dest);

/** A function to create XML elements.
 \param elem The element's name (eg. "xml" in "<xml> text <xml>")
 \param pcdata The element data (eg. "text" in "<xml> text <xml>")
 \return The XML element.
*/
std::string XMLembed (const std::string &elem, const std::string &pcdata);

//------------------------------------------------------------------------------------------------------------
/** Tokenize a string and produce a vector list of items */
//------------------------------------------------------------------------------------------------------------
class StringTokenizer : public std::vector<std::string>
{
public:
    StringTokenizer (const std::string &rStr, const std::string &rDelimiters = " ,\n");
    std::vector<std::string> WithoutEmpty() const;
};

//------------------------------------------------------------------------------------------------------------
/** @class nocase_string
 \brief An STL string class which ignores the case. */
//------------------------------------------------------------------------------------------------------------
struct nocase_string : public std::string {
public:
    nocase_string (const char* str);
    nocase_string (const nocase_string& str);
    nocase_string (const std::string& str);
    nocase_string ();

    bool operator <  (const char* rhs);
    bool operator <  (const nocase_string& rhs);
    bool operator <  (const std::string& rhs);
    bool operator == (const char* rhs);
    bool operator == (const nocase_string& rhs);
    bool operator == (const std::string& rhs);
    bool operator != (const char* rhs);
    bool operator != (const nocase_string& rhs);
    bool operator != (const std::string& rhs);
    void operator += (nocase_string s);
    nocase_string operator+ (nocase_string s);
};

#define _Int(s) atoi((s).c_str())
#define _Float(s) atof((s).c_str())

//------------------------------------------------------------------------------------------------------------
/** @class Listener
 \brief The abstract listener interface. */
//------------------------------------------------------------------------------------------------------------
struct Listener {
    virtual void OnUpdate(const void*) = 0;
    virtual ~Listener() {}
};

//------------------------------------------------------------------------------------------------------------
/** @class Action
 \brief The function object interface. */
//------------------------------------------------------------------------------------------------------------
struct Action {
    virtual bool execute(void*) = 0;
    virtual ~Action() {}
};

//------------------------------------------------------------------------------------------------------------
/** @class LocalObj3D
 \brief The CS-independent structure for location and orientation
 data of 3D objects. */
//------------------------------------------------------------------------------------------------------------
class LocalObj3D
{
public:
    LocalObj3D();
    double x, y, z, ox, oy, oz, ophi;
    float radius;

    void getOrientation (double &_ox, double &_oy, double &_oz, double &_ophi)  const;
    void setOrientation (double _ox, double _oy, double _oz, double _ophi);
    void getPosition    (double& _x, double& _y, double& _z) const;
    void setPosition    (double _x, double _y, double _z);
    void setListener    (Listener* _l);
protected:
    Listener* listener;
};

typedef LocalObj3D Obj3D;

#endif
