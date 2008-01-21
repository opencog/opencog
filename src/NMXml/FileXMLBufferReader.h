#ifndef _FILEXMLBUFFERREADER_H_
#define _FILEXMLBUFFERREADER_H_

#include "XMLBufferReader.h"
#include "exceptions.h"

#include <stdio.h>

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
    virtual void reset() throw (IOException);
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

#endif //_FILEXMLBUFFERREADER_H_
