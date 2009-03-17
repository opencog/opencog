#ifndef PETAVERSES_DOM_PARSERT_H_
#define PETAVERSES_DOM_PARSERT_H_
/**
 * This is an extension of XercesDOMParser to log the parse errors according with Petaverse log policy.
 */

#include <xercesc/parsers/XercesDOMParser.hpp>
#include "PAIUtils.h"

namespace PerceptionActionInterface {

class PetaverseDOMParser : public XERCES_CPP_NAMESPACE::XercesDOMParser  {

    public: 
        PetaverseDOMParser() { PAIUtils::initializeXMLPlatform(); }
        virtual ~PetaverseDOMParser() {};

        void error(const unsigned int    errCode,
           const XMLCh *const   errDomain,
           const ErrTypes  type,
           const XMLCh *const   errorText,
           const XMLCh *const   systemId,
           const XMLCh *const   publicId,
           const XMLSSize_t  lineNum,
           const XMLSSize_t  colNum);

};

}

#endif // PETAVERSES_DOM_PARSERT_H_
