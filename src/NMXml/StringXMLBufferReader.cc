#include <platform.h>
#include "StringXMLBufferReader.h"

StringXMLBufferReader::StringXMLBufferReader(const char* content)
{
    this->content = content;
    currentPos = NULL;
}

StringXMLBufferReader::~StringXMLBufferReader()
{
}

void StringXMLBufferReader::open() {
    currentPos = (char *) content;
}

void StringXMLBufferReader::close() {
    currentPos = NULL;
}

XMLBufferReader* StringXMLBufferReader::clone() {
    XMLBufferReader* result = new StringXMLBufferReader(content);
    return result;
}


size_t StringXMLBufferReader::read(void *ptr, size_t size, size_t nmemb) {
    size_t result = 0;
    while (*currentPos != '\0' && result < nmemb) {
        *((char *)ptr) = *currentPos++;
        ptr = (void*)(((char *) ptr)+1);
        result++;
    }
    return result;
}
