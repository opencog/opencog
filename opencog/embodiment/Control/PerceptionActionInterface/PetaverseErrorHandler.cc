/*
 * opencog/embodiment/Control/OperationalPetController/PetaverseErrorHandler.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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
#include "PetaverseErrorHandler.h"
#include "util/Logger.h"

using namespace PerceptionActionInterface;

void PetaverseErrorHandler::warning(const XERCES_CPP_NAMESPACE::SAXParseException& exc) {
    char* errorMsg = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getMessage());
    char* publicIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getPublicId());
    char* systemIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getSystemId());
    logger().log(opencog::Logger::WARN, "PetaverseErrorHandling - Got a warning while parsing XML: %s\n"
                    "Line: %d\n"
                    "Column: %d\n"
                    //"PublicId: %s\n"
                    "SystemId: %s\n"
		    , errorMsg
                    , exc.getLineNumber()
                    , exc.getColumnNumber()
                    //, publicIdStr
                    , systemIdStr 
		    ); 
    XERCES_CPP_NAMESPACE::XMLString::release(&errorMsg);
    XERCES_CPP_NAMESPACE::XMLString::release(&publicIdStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&systemIdStr);
}

void PetaverseErrorHandler::error(const XERCES_CPP_NAMESPACE::SAXParseException& exc) {
    char* errorMsg = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getMessage());
    char* publicIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getPublicId());
    char* systemIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getSystemId());
    logger().log(opencog::Logger::ERROR, "PetaverseErrorHandling - Got an error while parsing XML: %s\n"
                    "Line: %d\n"
                    "Column: %d\n"
                    //"PublicId: %s\n"
                    "SystemId: %s\n"
		    , errorMsg
                    , exc.getLineNumber()
                    , exc.getColumnNumber()
                    //, publicIdStr
                    , systemIdStr 
		    ); 
    XERCES_CPP_NAMESPACE::XMLString::release(&errorMsg);
    XERCES_CPP_NAMESPACE::XMLString::release(&publicIdStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&systemIdStr);
    throw exc;
}

void PetaverseErrorHandler::fatalError(const XERCES_CPP_NAMESPACE::SAXParseException& exc) {
    char* errorMsg = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getMessage());
    char* publicIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getPublicId());
    char* systemIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getSystemId());
    logger().log(opencog::Logger::ERROR, "PetaverseErrorHandling - Got a fatal error while parsing XML: %s\n"
                    "Line: %d\n"
                    "Column: %d\n"
                    //"PublicId: %s\n"
                    "SystemId: %s\n"
		    , errorMsg
                    , exc.getLineNumber()
                    , exc.getColumnNumber()
                    //, publicIdStr
                    , systemIdStr 
		    ); 
    XERCES_CPP_NAMESPACE::XMLString::release(&errorMsg);
    XERCES_CPP_NAMESPACE::XMLString::release(&publicIdStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&systemIdStr);
    throw exc;
}

void PetaverseErrorHandler::resetErrors() {
    logger().log(opencog::Logger::INFO, "PetaverseErrorHandling - resetErrors() called\n"); 
}

