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
 * $Id: DOMPrintErrorHandler.cpp 176026 2004-09-08 13:57:07Z peiyongz $
 * $Log$
 * Revision 1.7  2004/09/08 13:55:31  peiyongz
 * Apache License Version 2.0
 *
 * Revision 1.6  2003/05/30 09:36:35  gareth
 * Use new macros for iostream.h and std:: issues.
 *
 * Revision 1.5  2003/02/05 18:53:22  tng
 * [Bug 11915] Utility for freeing memory.
 *
 * Revision 1.4  2002/12/10 15:36:36  tng
 * DOMPrint minor update: print error message to XERCES_STD_QUALIFIER cerr.
 *
 * Revision 1.3  2002/06/13 14:55:01  peiyongz
 * Fix to UNIX compilation failure
 *
 * Revision 1.2  2002/06/11 19:46:28  peiyongz
 * Display error message received from the serializer.
 *
 * Revision 1.1  2002/05/29 21:19:50  peiyongz
 * DOM3 DOMWriter/DOMWriterFilter
 *
 *
 */

#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOMError.hpp>
#if defined(XERCES_NEW_IOSTREAMS)
#include <iostream>
#else
#include <iostream.h>
#endif

#include "DOMPrintErrorHandler.h"

bool DOMPrintErrorHandler::handleError(const DOMError &domError)
{
    // Display whatever error message passed from the serializer
    if (domError.getSeverity() == DOMError::DOM_SEVERITY_WARNING)
        XERCES_STD_QUALIFIER cerr << "\nWarning Message: ";
    else if (domError.getSeverity() == DOMError::DOM_SEVERITY_ERROR)
        XERCES_STD_QUALIFIER cerr << "\nError Message: ";
    else
        XERCES_STD_QUALIFIER cerr << "\nFatal Message: ";

    char *msg = XMLString::transcode(domError.getMessage());
    XERCES_STD_QUALIFIER cerr<< msg <<XERCES_STD_QUALIFIER endl;
    XMLString::release(&msg);

    // Instructs the serializer to continue serialization if possible.
    return true;
}

