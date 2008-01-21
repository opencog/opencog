#ifndef _STRINGXMLBUFFERREADER_H_
#define _STRINGXMLBUFFERREADER_H_

#include "XMLBufferReader.h"

class StringXMLBufferReader : public XMLBufferReader
{
public:
    /**
     * This constructor does make a copy of content
     **/
	StringXMLBufferReader(const char* content);
    /**
     * This destructor does not frees xml content
     **/
	virtual ~StringXMLBufferReader();
    virtual void reset();
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

#endif //_STRINGXMLBUFFERREADER_H_
