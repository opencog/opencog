/*
 * src/AtomSpace/utils.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef OPENCOG_UTILS_H
#define OPENCOG_UTILS_H

#include "platform.h"
#include "exceptions.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <string>
#include <sstream>

using namespace std;

#define Abs(a) ( ((a)>0) ? (a) : (-a) )

// Specific toString method for float numbers (force 3 digits after decimal point)
string toString(double data);

/**
 * Returns a string from the given argument by using the << operator
 */
template <typename T> 
string toString(T data) {
    ostringstream oss;
    oss << data;
    return oss.str();
}

/**
 * Returns a copy of the given string surrounded by ANSI bold tags. 
 *
 * @return Copy of the given string surrounded by ANSI bold tags.
 */
string bold(const char*);

/**
 * Returns the string representation of an integer surrounded by ANSI bold
 * tags.
 *
 * @return The string representation of an integer surrounded by ANSI bold
 * tags.
 */
string bold(int i);

template <class _Key> struct hash2int { };

inline size_t __hash2int(const char* __s) {
    unsigned long __h = 0; 
    size_t size = 2 * sizeof(int);
    for (size_t i = 0; i < size; i++, __s++) {
        __h = 5*__h + *__s;
    }
    return size_t(__h);
}

template <> struct hash2int<char *> {
    size_t operator()(const char* __s) const { return __hash2int(__s); }
};

enum padAlignment { CENTER, LEFT, RIGHT };
std::string padstr(const char*, unsigned int, padAlignment) throw (InvalidParamException);

class FileList {

    private:

        std::vector<char *>fileList;

    public:

        FileList(const char* ) throw (IOException);
        FileList();
        ~FileList();

        static FileList *getAllFilesRecursively(const char* );

        unsigned int getSize();
        const char* getFile(unsigned int) throw (IndexErrorException);
};

/**
 * This method reads a line from a string and returns
 * the rest of the string.
 *
 * @param Where the next line should be read from.
 * @param A reference to a string where the read next line will be.
 * @return The rest of the string after reading the line.
 */
const char *nextLine(const char *, std::string&);

/**
 * Counts the number of bits in 1 in the given unsigned long argument.
 */
int bitcount(unsigned long n);

/**
 * Initializes the reference time that will be used for getting current elapsed times
 */
void initReferenceTime(); 
/**
 * Gets the elapsed time (in milliseconds) since the reference time initialized with 
 * initReferenceTime() function. The initReferenceTime() function must be called before 
 * this function be called by the first time.
 */
ulong getElapsedMillis();

/** STL-Listifies a TemporalEntry */

#include <list>

template<typename T, typename OutT>
void to_list(OutT outIt, T inEntry)
{
	while (inEntry)
	{
		*(outIt++) = inEntry->handleTemporalPair; //handle;
		inEntry = inEntry->next;
	}
}

/** STL-Listifies a HandleEntry */

template<typename T, typename OutT>
void h_to_list(OutT outIt, T inEntry)
{
	while (inEntry)
	{
		*(outIt++) = inEntry->handle;
		inEntry = inEntry->next;
	}
}

template<typename T>
bool empty(const T& c) { return c.empty(); }

template<typename LinkT, typename ArgT1, typename ArgT2>
LinkT make_tuple(const ArgT1& arg1, const ArgT2& arg2)
{
	return LinkT(arg1, arg2);
}

template<typename ForwardIter,
	typename OutputIter,
	typename UnaryPred>
OutputIter copy_if(ForwardIter begin, ForwardIter end, OutputIter dest, UnaryPred f)
{
  while(begin != end) {
    if(f(*begin))
      *dest++ = *begin;
    ++begin;
  }
  return dest;
}

template<typename InputT, typename evalT, typename ValT>
InputT GetBest(InputT start, InputT end, evalT op, ValT minVal)
{
	if (start == end)
		return end;
	
	ValT bestV = minVal, tempVal;
	
	InputT ret = start;
	while (++start != end)
		if ( (tempVal=op(*start)) > bestV)
		{
			bestV = tempVal;
			ret = start;
		}
		
	return ret;
}

#endif /* OPENCOG_UTILS_H */
