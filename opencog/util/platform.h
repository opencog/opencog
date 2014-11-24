/*
 * opencog/util/platform.h
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

#ifndef _OPENCOG_PLATFORM_H
#define _OPENCOG_PLATFORM_H

#ifdef WIN32

#pragma warning(disable:4290)

#define strcasecmp _stricmp
#define snprintf _snprintf

#endif // WIN32

#include <stdio.h>
#include <string.h>
#include <string>
#include <stdint.h>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

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

#ifdef __APPLE__
char*              __strtok_r(char *s1, const char *s2, char **lasts);
#endif

#ifdef WIN32_NOT_UNIX

#define M_PI 3.14159265358979323846

struct timezone {};

int                round(float x);
char*              __strtok_r(char *s1, const char *s2, char **lasts);
int                gettimeofday(struct timeval* tp, void* tzp);
void               usleep(unsigned useconds);
int                __getpid(void);
double             rint(double nr);
int                __dup2(int, int);
unsigned long long atoll(const char *str);
unsigned int       sleep(unsigned seconds);

#endif // ~WIN32_NOT_UNIX

//! Return the total amount of heap allocated (according to sbrk, on unix).
size_t getMemUsage();

//! Return the total number of bytes of physical RAM installed.
uint64_t getTotalRAM();

//! Return the total number of free bytes avaiable in RAM (excluding OS caches)
uint64_t getFreeRAM();

//! Return the OS username
const char* getUserName();

/** @}*/
} // namespace opencog

#endif // _OPENCOG_PLATFORM_H
