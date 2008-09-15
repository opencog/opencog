/*
 * opencog/xml/StringXMLBufferReader.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#include "StringXMLBufferReader.h"

#include <opencog/util/platform.h>

using namespace opencog;

StringXMLBufferReader::StringXMLBufferReader(const char* content)
{
    this->content = content;
    currentPos = NULL;
}

StringXMLBufferReader::~StringXMLBufferReader()
{
}

void StringXMLBufferReader::open()
{
    currentPos = (char *) content;
}

void StringXMLBufferReader::close()
{
    currentPos = NULL;
}

XMLBufferReader* StringXMLBufferReader::clone()
{
    XMLBufferReader* result = new StringXMLBufferReader(content);
    return result;
}


size_t StringXMLBufferReader::read(void *ptr, size_t size, size_t nmemb)
{
    size_t result = 0;
    while (*currentPos != '\0' && result < nmemb) {
        *((char *)ptr) = *currentPos++;
        ptr = (void*)(((char *) ptr) + 1);
        result++;
    }
    return result;
}
