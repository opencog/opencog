/*
 * Copyright 1999-2000,2004 The Apache Software Foundation.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * $Id: Win32TransService.hpp 191054 2005-06-17 02:56:35Z jberry $
 */

#ifndef WIN32TRANSSERVICE_HPP
#define WIN32TRANSSERVICE_HPP

#include <xercesc/util/TransService.hpp>
#include <xercesc/util/RefHashTableOf.hpp>
#include <windows.h>

XERCES_CPP_NAMESPACE_BEGIN

class CPMapEntry;



//---------------------------------------------------------------------------
//
//  class Win32TransService
//
//---------------------------------------------------------------------------
class XMLUTIL_EXPORT Win32TransService : public XMLTransService
{
public :
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    Win32TransService();
    virtual ~Win32TransService();


    // -----------------------------------------------------------------------
    //  Implementation of the virtual transcoding service API
    // -----------------------------------------------------------------------
    virtual int compareIString
    (
        const   XMLCh* const    comp1
        , const XMLCh* const    comp2
    );

    virtual int compareNIString
    (
        const   XMLCh* const    comp1
        , const XMLCh* const    comp2
        , const unsigned int    maxChars
    );

    virtual const XMLCh* getId() const;

    virtual bool isSpace(const XMLCh toCheck) const;

    virtual XMLLCPTranscoder* makeNewLCPTranscoder();

    virtual bool supportsSrcOfs() const;

    virtual void upperCase(XMLCh* const toUpperCase) const;
    virtual void lowerCase(XMLCh* const toLowerCase) const;


protected :
    // -----------------------------------------------------------------------
    //  Protected virtual methods, implemented in Win32TransService2.cpp
    // -----------------------------------------------------------------------
    virtual XMLTranscoder* makeNewXMLTranscoder
    (
        const   XMLCh* const            encodingName
        ,       XMLTransService::Codes& resValue
        , const unsigned int            blockSize
        ,       MemoryManager* const    manager
    );


private :
    // -----------------------------------------------------------------------
    //  Unimplemented constructors and operators
    // -----------------------------------------------------------------------
    Win32TransService(const Win32TransService&);
    Win32TransService& operator=(const Win32TransService&);

    //      This is a hash table of entries which map encoding names to their
    //      Windows specific code pages. The code page allows us to create
    //      transcoders for those encodings. The encoding names come from XML
    //      files.
    //
    //      This map is shared unsynchronized among all threads of the process,
    //      which is cool since it will be read only once its initialized.



    static bool isAlias(const   HKEY            encodingKey
                    ,       char* const     aliasBuf = 0
                    , const unsigned int    nameBufSz = 0);


    RefHashTableOf<CPMapEntry>    *fCPMap;
};







//---------------------------------------------------------------------------
//
//  class Win32Transcoder
//
//---------------------------------------------------------------------------

class XMLUTIL_EXPORT Win32Transcoder : public XMLTranscoder
{
public :
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    Win32Transcoder
    (
        const   XMLCh* const    encodingName
        , const unsigned int    winCP
        , const unsigned int    ieCP
        , const unsigned int    blockSize
        , MemoryManager* const  manager = XMLPlatformUtils::fgMemoryManager);
    ~Win32Transcoder();


    // -----------------------------------------------------------------------
    //  Implementation of the virtual transcoder interface
    // -----------------------------------------------------------------------
    virtual unsigned int transcodeFrom
    (
        const   XMLByte* const          srcData
        , const unsigned int            srcCount
        ,       XMLCh* const            toFill
        , const unsigned int            maxChars
        ,       unsigned int&           bytesEaten
        ,       unsigned char* const    charSizes
    );

    virtual unsigned int transcodeTo
    (
        const   XMLCh* const    srcData
        , const unsigned int    srcCount
        ,       XMLByte* const  toFill
        , const unsigned int    maxBytes
        ,       unsigned int&   charsEaten
        , const UnRepOpts       options
    );

    virtual bool canTranscodeTo
    (
        const   unsigned int    toCheck
    )   const;


private :
    // -----------------------------------------------------------------------
    //  Unimplemented constructors and operators
    // -----------------------------------------------------------------------
    Win32Transcoder(const Win32Transcoder&);
    Win32Transcoder& operator=(const Win32Transcoder&);


    // -----------------------------------------------------------------------
    //  Private data members
    //
    //  fIECP
    //      This is the internet explorer code page for this encoding.
    //
    //  fWinCP
    //      This is the windows code page for this encoding.
    // -----------------------------------------------------------------------
    unsigned int    fIECP;
    unsigned int    fWinCP;
};





//---------------------------------------------------------------------------
//
//  class Win32LCPTranscoder
//
//---------------------------------------------------------------------------
class XMLUTIL_EXPORT Win32LCPTranscoder : public XMLLCPTranscoder
{
public :
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    Win32LCPTranscoder();
    ~Win32LCPTranscoder();


    // -----------------------------------------------------------------------
    //  Implementation of the virtual transcoder interface
    // -----------------------------------------------------------------------
    virtual unsigned int calcRequiredSize(const char* const srcText
        , MemoryManager* const manager = XMLPlatformUtils::fgMemoryManager);

    virtual unsigned int calcRequiredSize(const XMLCh* const srcText
        , MemoryManager* const manager = XMLPlatformUtils::fgMemoryManager);

    virtual char* transcode(const XMLCh* const toTranscode);
    virtual char* transcode(const XMLCh* const toTranscode,
                            MemoryManager* const manager);

    virtual XMLCh* transcode(const char* const toTranscode);
    virtual XMLCh* transcode(const char* const toTranscode,
                             MemoryManager* const manager);

    virtual bool transcode
    (
        const   char* const     toTranscode
        ,       XMLCh* const    toFill
        , const unsigned int    maxChars
        , MemoryManager* const  manager = XMLPlatformUtils::fgMemoryManager
    );

    virtual bool transcode
    (
        const   XMLCh* const    toTranscode
        ,       char* const     toFill
        , const unsigned int    maxChars
        , MemoryManager* const  manager = XMLPlatformUtils::fgMemoryManager
    );


private :
    // -----------------------------------------------------------------------
    //  Unimplemented constructors and operators
    // -----------------------------------------------------------------------
    Win32LCPTranscoder(const Win32LCPTranscoder&);
    Win32LCPTranscoder& operator=(const Win32LCPTranscoder&);
};

XERCES_CPP_NAMESPACE_END

#endif
