#include "XmlErrorHandler.h"
#include "util/Logger.h"

using namespace VirtualWorldData;
using namespace opencog;

void XmlErrorHandler::warning(const XERCES_CPP_NAMESPACE::SAXParseException& exc) {
    char* errorMsg = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getMessage());
    char* publicIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getPublicId());
    char* systemIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getSystemId());
    logger().log(opencog::Logger::WARN, "XmlErrorHandler - Got a warning while parsing XML: %s\n"
                    "Line: %d\n"
                    "Column: %d\n"
                    "SystemId: %s\n"
		            , errorMsg
                    , exc.getLineNumber()
                    , exc.getColumnNumber()
                    , systemIdStr 
		    ); 
    XERCES_CPP_NAMESPACE::XMLString::release(&errorMsg);
    XERCES_CPP_NAMESPACE::XMLString::release(&publicIdStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&systemIdStr);
}

void XmlErrorHandler::error(const XERCES_CPP_NAMESPACE::SAXParseException& exc) {
    char* errorMsg = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getMessage());
    char* publicIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getPublicId());
    char* systemIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getSystemId());
    logger().log(opencog::Logger::ERROR, "XmlErrorHandler - Got an error while parsing XML: %s\n"
                    "Line: %d\n"
                    "Column: %d\n"
                    "SystemId: %s\n"
		            , errorMsg
                    , exc.getLineNumber()
                    , exc.getColumnNumber()
                    , systemIdStr 
		    ); 
    XERCES_CPP_NAMESPACE::XMLString::release(&errorMsg);
    XERCES_CPP_NAMESPACE::XMLString::release(&publicIdStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&systemIdStr);
    throw exc;
}

void XmlErrorHandler::fatalError(const XERCES_CPP_NAMESPACE::SAXParseException& exc) {
    char* errorMsg = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getMessage());
    char* publicIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getPublicId());
    char* systemIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(exc.getSystemId());
    logger().log(opencog::Logger::ERROR, "XmlErrorHandler - Got a fatal error while parsing XML: %s\n"
                    "Line: %d\n"
                    "Column: %d\n"
                    "SystemId: %s\n"
		            , errorMsg
                    , exc.getLineNumber()
                    , exc.getColumnNumber()
                    , systemIdStr 
		    ); 
    XERCES_CPP_NAMESPACE::XMLString::release(&errorMsg);
    XERCES_CPP_NAMESPACE::XMLString::release(&publicIdStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&systemIdStr);
    throw exc;
}

void XmlErrorHandler::resetErrors() {
    logger().log(opencog::Logger::INFO, "XmlErrorHandler - resetErrors() called\n"); 
}

