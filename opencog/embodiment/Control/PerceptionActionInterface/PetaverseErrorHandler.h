#ifndef PETAVERSES_ERROR_HANDLER_H_
#define PETAVERSES_ERROR_HANDLER_H_
/**
 * This is an extension of ErrorHandler to log the parse errors according with Petaverse log policy.
 */

#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include "PAIUtils.h"

namespace PerceptionActionInterface {

class PetaverseErrorHandler : public XERCES_CPP_NAMESPACE::ErrorHandler  {
    void warning(const XERCES_CPP_NAMESPACE::SAXParseException& exc);
    void error(const XERCES_CPP_NAMESPACE::SAXParseException& exc);
    void fatalError(const XERCES_CPP_NAMESPACE::SAXParseException& exc);
    void resetErrors();

public: 
    PetaverseErrorHandler() { PAIUtils::initializeXMLPlatform(); }

};

}

#endif // PETAVERSES_ERROR_HANDLER_H_
