/*
 * opencog/xml/StringXMLBufferReader.h
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

#ifndef _OPENCOG_STRING_XML_BUFFER_READER_H
#define _OPENCOG_STRING_XML_BUFFER_READER_H

#include <opencog/persist/xml/XMLBufferReader.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class StringXMLBufferReader : public XMLBufferReader
{
public:
    /**
     * This constructor does NOT make a copy of content
     **/
    StringXMLBufferReader(const char* content);
    /**
     * This destructor does NOT free xml content
     **/
    virtual ~StringXMLBufferReader();
    virtual void open();
    virtual size_t read(void *ptr, size_t size, size_t nmemb);
    virtual void close();
    /**
     * Does not make a deep clone
     */
    virtual XMLBufferReader* clone();

private:
    const char* content;
    char* currentPos;

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_STRING_XML_BUFFER_READER_H
