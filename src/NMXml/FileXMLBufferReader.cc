#include "FileXMLBufferReader.h"
#include <string.h>

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
        throw IOException(TRACE_INFO, "FileXMLBufferReader - unable to open file '%s'.", filename);
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


