/*
 * src/Util/platform.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks <moshe@metacog.org>
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

#ifndef _OPENCOG_PLATFORM_H_
#define _OPENCOG_PLATFORM_H_

#ifdef WIN32

#pragma warning(disable:4290)
#define _CRT_SECURE_NO_WARNINGS 1
#include <winsock2.h>
#define snprintf _snprintf
#define strcasecmp _stricmp
#define strdup _strdup

#include <hash_set>
#include <hash_map>
#include <functional>
#include <algorithm>

/// Windows headers define ATOM for other purposes, so we temporarily undef it.
#undef ATOM
#include <windows.h>
#include <Mmsystem.h>
#define ATOM 0

#else // NOT WIN32

#include <ext/hash_set>
#include <ext/hash_map>
#include <ext/functional>

#endif // WIN32

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <string>

using namespace std;

namespace opencog
{

struct eqstr {
    bool operator()(char *s1, char *s2) const {
        return strcmp(s1, s2) == 0;
    }
};

struct eqconststr {
    bool operator()(const char *s1, const char *s2) const {
        return strcmp(s1, s2) == 0;
    }
};

struct eqint {
    bool operator()(int s1, int s2) const {
        return s1 == s2;
    }
};

#ifdef WIN32

#define M_PI 3.14159265358979323846

struct timezone {};

int                round(float x);
char*              __strtok_r(char *s1, const char *s2, char **lasts);
int                gettimeofday(struct timeval* tp, void* tzp);
void               usleep(unsigned int useconds);
int                __getpid(void);
double             rint(double nr);
int                __dup2(int, int);
unsigned long long atoll(const char *str);
unsigned int       sleep(unsigned seconds);

typedef hash<int>           int_hash;
typedef hash<unsigned int>  unsigned_int_hash;
typedef hash<long>          long_hash;
typedef hash<unsigned long> unsigned_long_hash;
typedef hash<const char*>   const_char_hash;

typedef hash_map<const char*, int, const_char_hash, eqconststr> ConstCharPointerIntHashMap;
typedef hash_map<int, const char*, int_hash, eqint> IntConstCharPointerHashMap;
typedef hash_map<const char*, void*, const_char_hash, eqconststr> ConstCharPointerVoidHashMap;
typedef hash_map<int, int> Int2IntHashMap;
typedef hash_set<int> Int2IntHashSet;

#else // !WIN32

typedef __gnu_cxx::hash<int>           int_hash;
typedef __gnu_cxx::hash<unsigned int>  unsigned_int_hash;
typedef __gnu_cxx::hash<long>          long_hash;
typedef __gnu_cxx::hash<unsigned long> unsigned_long_hash;
typedef __gnu_cxx::hash<const char*>   const_char_hash;

typedef __gnu_cxx::hash_map<const char*, int, const_char_hash, eqconststr> ConstCharPointerIntHashMap;
typedef __gnu_cxx::hash_map<int, const char*, int_hash, eqint> IntConstCharPointerHashMap;
typedef __gnu_cxx::hash_map<const char*, void*, const_char_hash, eqconststr> ConstCharPointerVoidHashMap;
typedef __gnu_cxx::hash_map<int, int> Int2IntHashMap;
typedef __gnu_cxx::hash_set<int> Int2IntHashSet;

using __gnu_cxx::hash_map;
using __gnu_cxx::hash;
using std::make_pair;

#endif // WIN32!

} // namespace opencog

#endif //_PLATFORM_H_
