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
 * $Id: DOMPrintErrorHandler.hpp 176026 2004-09-08 13:57:07Z peiyongz $
 * $Log$
 * Revision 1.3  2004/09/08 13:55:31  peiyongz
 * Apache License Version 2.0
 *
 * Revision 1.2  2002/11/05 21:46:19  tng
 * Explicit code using namespace in application.
 *
 * Revision 1.1  2002/05/29 21:19:50  peiyongz
 * DOM3 DOMWriter/DOMWriterFilter
 *
 *
 */


#ifndef DOM_PRINT_ERROR_HANDLER_HPP
#define DOM_PRINT_ERROR_HANDLER_HPP

#include <xercesc/dom/DOMErrorHandler.hpp>

XERCES_CPP_NAMESPACE_USE

class DOMPrintErrorHandler : public DOMErrorHandler
{
public:

    DOMPrintErrorHandler(){};
    ~DOMPrintErrorHandler(){};

    /** @name The error handler interface */
    bool handleError(const DOMError& domError);
    void resetErrors(){};

private :
    /* Unimplemented constructors and operators */
    DOMPrintErrorHandler(const DOMErrorHandler&);
    void operator=(const DOMErrorHandler&);

};

#endif
