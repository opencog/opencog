/*
 * Copyright 2002,2004 The Apache Software Foundation.
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
 * $Id: FileHandleImpl.hpp 176026 2004-09-08 13:57:07Z peiyongz $
 */

#ifndef FILEHANDLEIMPL_HPP
#define FILEHANDLEIMPL_HPP
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMemory.hpp>

XERCES_CPP_NAMESPACE_BEGIN

class FileHandleImpl : public XMemory
{
  private:
  FILE*      Handle;       // handle from fopen
  XMLByte*   stgBufferPtr; // address of staging buffer
  int        nextByte;     // NAB in staging buffer
  int        openType;     // 0=write, 1=read
  int        lrecl;        // LRECL if openType is write
  bool       recordType;   // true if "type=record"
  MemoryManager* const fMemoryManager;

  public:
      FileHandleImpl(FILE* open_handle, int o_type, bool r_type, int fileLrecl=0, MemoryManager* const manager=XMLPlatformUtils::fgMemoryManager);
 ~FileHandleImpl();
  void  setHandle(FILE* newHandlePtr) { Handle = newHandlePtr; }
  void* getHandle() { return Handle; }
  XMLByte* getStgBufferPtr() { return stgBufferPtr; }
  int   getNextByte() { return nextByte; }
  void  setNextByte(int newNextByte)  { nextByte = newNextByte; }
  int   getOpenType() { return openType; }
  bool  isRecordType() { return recordType; }
  void  setRecordType(bool newType) { recordType = newType; }
  int   getLrecl() { return lrecl; }
  void  setLrecl(int newLrecl)  { lrecl = newLrecl; }
};

// Constants for the openType member
#define _FHI_WRITE 0
#define _FHI_READ 1
// Constant for the typeRecord member
#define _FHI_NOT_TYPE_RECORD 0
#define _FHI_TYPE_RECORD 1

XERCES_CPP_NAMESPACE_END

#endif
