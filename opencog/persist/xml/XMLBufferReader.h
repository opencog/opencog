/*
 * opencog/xml/XMLBufferReader.h
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

#ifndef _OPENCOG_XML_BUFFER_READER_H
#define _OPENCOG_XML_BUFFER_READER_H

#include <stdio.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class XMLBufferReader
{
public:
    virtual ~XMLBufferReader() {}

    virtual void open() = 0;
    virtual size_t read(void *ptr, size_t size, size_t nmemb) = 0;
    virtual void close() = 0;
    virtual XMLBufferReader* clone() = 0;
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_XML_BUFFER_READER_H
