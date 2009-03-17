#include "PetaverseDOMParser.h"
#include <LADSUtil/Logger.h>
#include <xercesc/dom/DOMException.hpp>

using namespace PerceptionActionInterface;

void PetaverseDOMParser::error (const unsigned int    errCode,
                                const XMLCh *const   errDomain,
                                const ErrTypes  type,
                                const XMLCh *const   errorText,
                                const XMLCh *const   systemId,
                                const XMLCh *const   publicId,
                                const XMLSSize_t  lineNum,
                                const XMLSSize_t  colNum) {
    XERCES_CPP_NAMESPACE::XercesDOMParser::error(errCode, errDomain, type, errorText, systemId, publicId, lineNum, colNum);

    char* errDomainStr = XERCES_CPP_NAMESPACE::XMLString::transcode(errDomain);
    char* errorTextStr  = XERCES_CPP_NAMESPACE::XMLString::transcode(errorText);
    char* systemIdStr  = XERCES_CPP_NAMESPACE::XMLString::transcode(systemId);
    char* publicIdStr  = XERCES_CPP_NAMESPACE::XMLString::transcode(publicId);
    MAIN_LOGGER.log(LADSUtil::Logger::ERROR, "PetaverseDOMParser - XML Parser error:"
		                         "\n  code = %u"
		                         "\n  domain = %s"
		                         "\n  type = %d"
		                         "\n  text = %s"
		                         "\n  systemId = %s"
		                         "\n  publicId = %s"
		                         "\n  lineNum = %d"
		                         "\n  colNum = %d", 
                                         errDomainStr,
                                         type,
                                         errorTextStr,
                                         systemIdStr,
                                         publicIdStr,
                                         lineNum,
                                         colNum); 
    XERCES_CPP_NAMESPACE::XMLString::release(&errDomainStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&errorTextStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&systemIdStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&publicIdStr);
    throw new XERCES_CPP_NAMESPACE::DOMException(errCode, errorText);
}

