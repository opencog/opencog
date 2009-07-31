/*
 * src/AtomSpace/utils.cc
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

#include <stdlib.h>
#include <stdarg.h>
#include <string>
#include <time.h>
#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/time.h>
#endif

#include "utils.h"
#include <opencog/util/platform.h>

using namespace opencog;

std::string opencog::toString(double data)
{
    char buffer[256];
    sprintf(buffer, "%f", data);
    return buffer;
}

namespace opencog {

/* MISC UTILITIES */

StringTokenizer::StringTokenizer(const std::string &rStr, const std::string &rDelimiters)
{
    std::string::size_type lastPos(rStr.find_first_not_of(rDelimiters, 0));
    std::string::size_type pos(rStr.find_first_of(rDelimiters, lastPos));
    while (std::string::npos != pos || std::string::npos != lastPos) {
        push_back(rStr.substr(lastPos, pos - lastPos));
        lastPos = rStr.find_first_not_of(rDelimiters, pos);
        pos = rStr.find_first_of(rDelimiters, lastPos);
    }
}

std::vector<std::string> StringTokenizer::WithoutEmpty() const
{
    std::vector<std::string> ret;

    for (unsigned int i = 0; i < this->size(); i++)
        if (!(*this)[i].empty())
            ret.push_back((*this)[i]);

    return ret;
}


/**
 * Time used as reference to set/get timestamps over the code
 */
static timeval referenceTime;
static bool referenceTimeInitialized = false;

void initReferenceTime()
{
    gettimeofday(&referenceTime, NULL);
    referenceTimeInitialized = true;
}

unsigned long getElapsedMillis()
{
    cassert(TRACE_INFO, referenceTimeInitialized,
            "utils - refenceTimeInitialized should have been initialized.");
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return (currentTime.tv_sec -referenceTime.tv_sec)*1000 + (currentTime.tv_usec - referenceTime.tv_usec) / 1000;
}


std::string i2str(int d)
{
    char temp[20];
    sprintf(temp, "%d", d);
    return temp;
}

bool visible(char c)
{
    return c != ' ' && c != '\r' && c != '\n' && c != '\t' && c != 0;
}

};

