/*
 * opencog/xml/FileXMLBufferReader.h
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

#ifndef _OPENCOG_FILE_XML_BUFFER_READER_H
#define _OPENCOG_FILE_XML_BUFFER_READER_H

#include <stdio.h>

#include <opencog/persist/xml/XMLBufferReader.h>
#include <opencog/util/exceptions.h>

#ifdef WIN32
#pragma warning(disable: 4290)
#endif

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class FileXMLBufferReader : public XMLBufferReader
{

public:
    /**
     * This constructor does make a copy of filename
     **/
    FileXMLBufferReader(const char* filename);
    /**
     * This destructor frees filename memory
     **/
    virtual ~FileXMLBufferReader();

    /**
     * Open the previously indicated filename.
     * Throw an exception if the file does not exist.
     */
    virtual void open() throw (IOException);
    virtual size_t read(void *ptr, size_t size, size_t nmemb);
    virtual void close();
    /**
     * Make a deep clone
     */
    virtual XMLBufferReader* clone();
    const char* getFilename();

private:
    char* filename;
    FILE* file;
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_FILE_XML_BUFFER_READER_H
