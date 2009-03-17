#include "PetaverseErrorHandler.h"
#include "util/Logger.h"

using namespace PerceptionActionInterface;

void PetaverseErrorHandler::warning(const XERCES_CPP_NAMESPACE::SAXParseException& exc) {
    char* errorMsg = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getMessage());
    char* publicIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getPublicId());
    char* systemIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getSystemId());
    logger().log(opencog::Logger::WARNING, "PetaverseErrorHandling - Got a warning while parsing XML: %s\n"
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

