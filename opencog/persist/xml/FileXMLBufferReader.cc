/*
 * opencog/xml/FileXMLBufferReader.cc
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

#include "FileXMLBufferReader.h"

#include <string.h>
#include <stdlib.h>
#ifdef _MSC_VER
//for _getcwd
 #include <direct.h>
 #define getcwd _getcwd
#else
 #include <unistd.h>
#endif

#include <opencog/util/platform.h>
#include <opencog/util/macros.h>

using namespace opencog;

FileXMLBufferReader::FileXMLBufferReader(const char* filename)
{
    this->filename = strdup(filename);
    file = NULL;
}

FileXMLBufferReader::~FileXMLBufferReader()
{
    close();
    free (filename);
}

const char* FileXMLBufferReader::getFilename()
{
    return filename;
}

void FileXMLBufferReader::open() throw (IOException)
{
    file = fopen(filename, "r");
    if (file == NULL) {
        char buff[1000];
        char * p_buff = getcwd(buff, 1000);
        throw IOException(TRACE_INFO, "FileXMLBufferReader - unable to open file '%s'. cwd='%s'.", filename, p_buff);
    }
}

void FileXMLBufferReader::close()
{
    if (file != NULL) {
        fclose(file);
        file = NULL;
    }
}

XMLBufferReader* FileXMLBufferReader::clone()
{
    XMLBufferReader* result = new FileXMLBufferReader(filename);
    return result;
}

size_t FileXMLBufferReader::read(void *ptr, size_t size, size_t nmemb)
{
    return fread(ptr, size, nmemb, file);
}
