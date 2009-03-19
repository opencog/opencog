#ifndef XML_LOADER_H
#define XML_LOADER_H

#include "XmlErrorHandler.h"
#include "VirtualWorldState.h"

#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>


namespace VirtualWorldData {

class XmlLoader {

    private: 
        
        XERCES_CPP_NAMESPACE::XercesDOMParser * parser;
    	VirtualWorldData::XmlErrorHandler errorHandler;

        // Initializes the xerces-c XML Platform
        void initializeXMLPlatform();
    
        // Teminate the xerces-c XML Platform
        void terminateXMLPlatform();

        // Process the World State XML document
        void processWorldStateDocument(XERCES_CPP_NAMESPACE::DOMDocument * doc,  
                                       VirtualWorldState & worldState);
        
        //
        void processEntityInfo(XERCES_CPP_NAMESPACE::DOMElement * element, 
                               VirtualWorldState & worldState);

        //
        void processEntityElement(XERCES_CPP_NAMESPACE::DOMElement * element, 
                                  VirtualWorldData::VirtualEntity & entity);

        //
        void processAgentElement(XERCES_CPP_NAMESPACE::DOMElement * element, 
                                 VirtualWorldData::VirtualAgent & agent);

        //
        void processIndefiniteObjectInfo(XERCES_CPP_NAMESPACE::DOMElement * element,
                                         VirtualWorldState & worldState );
        
        //
        void processWorldStateInfo(XERCES_CPP_NAMESPACE::DOMElement * element, 
                                   VirtualWorldState & worldState );

    public: 
        
        XmlLoader();
        ~XmlLoader();

        bool fromFile(const std::string & filename, VirtualWorldState & worldState);

};
} // namespace

#endif
