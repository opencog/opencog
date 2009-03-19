#ifndef XML_ERROR_HANDLER_H_
#define XML_ERROR_HANDLER_H_

#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>

namespace VirtualWorldData {

class XmlErrorHandler : public XERCES_CPP_NAMESPACE::ErrorHandler  {
    private:
        void resetErrors();

        void warning(const XERCES_CPP_NAMESPACE::SAXParseException& exc);

        void error(const XERCES_CPP_NAMESPACE::SAXParseException& exc);
        void fatalError(const XERCES_CPP_NAMESPACE::SAXParseException& exc);

    public: 
        XmlErrorHandler() { }

}; // class
}  // namespace

#endif // XML_ERROR_HANDLER_H_
