#ifndef _BUFFERREADER_H_
#define _BUFFERREADER_H_

#include <stdio.h> 

class XMLBufferReader
{
public:
	virtual ~XMLBufferReader() {}

    virtual void reset() = 0;
    virtual size_t read(void *ptr, size_t size, size_t nmemb) = 0;
    virtual void close() = 0;
    virtual XMLBufferReader* clone() = 0;
};

#endif //_BUFFERREADER_H_
