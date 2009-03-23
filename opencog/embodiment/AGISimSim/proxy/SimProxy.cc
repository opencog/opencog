#include <memory>
#include <iostream>

#include <Sockets/SocketHandler.h>

#include "SimProxy.h"
#include "SimClientSocket.h"

#include "AsynchronousMessageReceiver.h"

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLUni.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/util/XMLString.hpp>

#include <xercesc/framework/MemBufInputSource.hpp>
#include <xercesc/framework/MemBufFormatTarget.hpp>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMWriter.hpp>

#include <xercesc/parsers/XercesDOMParser.hpp>

#include "DOMTreeErrorReporter.h"
#include "DOMPrintErrorHandler.h"

#include "util/foreach.h"
#include "util/platform.h"
#include "util/Logger.h"

#define WAIT_RESPONSE_TIMEOUT 10
#define SEC_DELAY 0
#define USEC_DELAY 5000

#define MAX_TAG_LENGTH 256

// Define this to generate predicates related to the start of AgiSim action events.
#define CONVERT_ACTION_STARTED_EVENTS

#define BROADCAST_MESSAGE_GOES_TO_PENDING_ACTIONS

using namespace std;
using namespace opencog;

bool SimProxy::using_single_component_mode = true;
unsigned long SimProxy::numberOfInstances = 0;
XERCES_CPP_NAMESPACE::DOMImplementation* SimProxy::domImplementation = NULL;

// ---------------------------------------------------------------------------
//  This is a simple class that lets us do easy (though not terribly efficient)
//  trancoding of char* data to XMLCh data.
// ---------------------------------------------------------------------------
class XStr
{
public :
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    XStr(const char* const toTranscode)
    {
        // Call the private transcoding method
        fUnicodeForm = XMLString::transcode(toTranscode);
    }

    ~XStr()
    {
        XMLString::release(&fUnicodeForm);
    }


    // -----------------------------------------------------------------------
    //  Getter methods
    // -----------------------------------------------------------------------
    const XMLCh* unicodeForm() const
    {
        return fUnicodeForm;
    }

private :
    // -----------------------------------------------------------------------
    //  Private data members
    //
    //  fUnicodeForm
    //      This is the Unicode XMLCh format of the string.
    // -----------------------------------------------------------------------
    XMLCh*   fUnicodeForm;
};

#define X(str) XStr(str).unicodeForm()

SimProxy::SimProxy(const std::string& _host, unsigned short  _port, AsynchronousPerceptionAndStatusHandler* handler, bool _echoing) {
    host = _host;
    port = _port;
    perceptionAndStatusHandler = handler;
    echoing = _echoing;
    eventMap = new EventId2AtomRepMap();
    numberOfPendingActions = 0;

    sh = new SocketHandler();
    cc = new SimClientSocket(*sh, this, echoing);
    InitXMLPlatform();
    nextActionTicket = 1; // starts from 1 since 0 is reserved for indicating imediate success of the operation
    checkingAsynchronousMessages = false;
    numberOfInstances++;
    printf("\nSimProxy constructor: number of instances = %lu\n", numberOfInstances);
}

SimProxy::~SimProxy() {
    if (checkingAsynchronousMessages) {
        checkingAsynchronousMessages = false;
        void* threadReturn;
        pthread_join(checkAsynchronousMessagesThread, &threadReturn);
    }
    if (cc->IsConnected()) {
        cc->SetCloseAndDelete();
    }

    delete cc;
	delete sh;

    numberOfInstances--;
    printf("\nSimProxy destructor: number of instances = %lu\n", numberOfInstances);
    TerminateXMLPlatform();
}

void* SimProxy::ThreadCheckAsynchronousMessages(void* args) {
    logger().log(opencog::Logger::DEBUG, "SimProxy::ThreadCheckAsynchronousMessages called");
    SimProxy* proxy = (SimProxy*) args;
    while(true) {
        //sleep(1); // 1s
        usleep(100000); // 100ms
	if (!proxy->checkingAsynchronousMessages) {
            pthread_exit(NULL);
        }
        proxy->checkAsynchronousMessages();
    }
    return NULL;
}

bool SimProxy::eqstr::operator()(char *s1, char *s2) const {
    return strcmp(s1, s2) == 0;
}

void SimProxy::InitXMLPlatform() {
    if (numberOfInstances > 0) return;

    printf("\nInitializing Xerces-c XML Platform\n");
    // Initialize the XML4C2 system
    try
    {
        XMLPlatformUtils::Initialize();
    }
    catch(const XMLException &toCatch)
    {
        XERCES_STD_QUALIFIER cerr << "Error during Xerces-c Initialization.\n"
             << "  Exception message:"
             << StrX(toCatch.getMessage()) << XERCES_STD_QUALIFIER endl;
    }
    domImplementation =  DOMImplementationRegistry::getDOMImplementation(X("Core"));
    if (domImplementation == NULL) {
        XERCES_STD_QUALIFIER cerr << "Requested DOM implementation is not supported!" << XERCES_STD_QUALIFIER endl;
        exit(-1);
    }
}

void SimProxy::TerminateXMLPlatform() {
    if (!numberOfInstances) {
        printf("\nTerminating Xerces-c XML Platform\n");
        XMLPlatformUtils::Terminate();
    }
}

bool SimProxy::connect() {
    bool result = false;
    if (cc->Open(host,port)) {
        sh->Add(cc);
        sh->Select(1,0);
        // Check connection (It took a while to connect)
        while(!cc->IsConnected()) {
            sh->Select(0,0);
            if (cc->ConnectionFailed()) {
                break;
            }
        }
        if (cc->IsConnected()) {
            //printf("Connection to the AGISIM server (%s, %d) opened\n", host.c_str(), port);

            // Create a thread to check asynchronous messages periodically
            pthread_create(&checkAsynchronousMessagesThread, NULL, SimProxy::ThreadCheckAsynchronousMessages, (void *) this);
            checkingAsynchronousMessages = true;

            result = true;
        //} else {
          //    printf("Could not open connection to the AGISIM server (%s, %d)\n", host.c_str(), port);
        }
    } else {
           //printf("Could not open connection to the AGISIM server (%s, %d)\n", host.c_str(), port);
    }
    return result;
}

bool SimProxy::IsConnected() {
    return cc->IsConnected();
}

bool SimProxy::hasPendingActions() {
    //if (numberOfPendingActions > 0) printf("SimProxy::hasPendingActions(): numberOfPendingActions = %d\n", numberOfPendingActions);
    return (numberOfPendingActions > 0);
}

void SimProxy::checkAsynchronousMessages() {
    //logger().log(opencog::Logger::FINE, "SimProxy::checkAsynchronousMessages()");
    if (IsConnected()) {
        while(sh->Select(SEC_DELAY, USEC_DELAY)>0){}; // to get any old/spontaneous responses
    }
}

std::string SimProxy::getAgentName() {
    return agentName;
}



bool SimProxy::timeout(timeval beginTime, timeval currentTime, long waitTimeout) {
    long timeoutSec = beginTime.tv_sec + waitTimeout;
    if (currentTime.tv_sec < timeoutSec) {
       return false;
    } else if (currentTime.tv_sec == timeoutSec) {
       return (currentTime.tv_usec > beginTime.tv_usec);
    } else {
       return true;
    }
}

/**
 * METHODS TO SEND COMMANDS TO AND RECEIVE RESPONSES FROM AGISIM SERVER
 */

// Internal methods

XERCES_CPP_NAMESPACE::DOMDocument* SimProxy::parseXML(const std::string &xmlStr) {
//    printf("SimProxy::parseXML()\n");
    //  Create MemBufferInputSource from the buffer containing the XML
    //  statements.
    //  NOTE: We are using strlen() here, since we know that the chars in
    //  our hard coded buffer are single byte chars!!! The parameter wants
    //  the number of BYTES, not chars, so when you create a memory buffer
    //  give it the byte size (which just happens to be the same here.)
    MemBufInputSource* memBufIS = new MemBufInputSource(
        (const XMLByte*)xmlStr.c_str(),
        xmlStr.size(),
        "incomingXml");

    //  Create our parser, then attach an error handler to the parser.
    //  The parser will call back to methods of the ErrorHandler if it
    //  discovers errors during the course of parsing the XML document.
    XercesDOMParser *parser = new XercesDOMParser();

    //  Optional parameters
    //parser->setValidationScheme(gValScheme);
    //parser->setDoNamespaces(gDoNamespaces);
    //parser->setDoSchema(gDoSchema);
    //parser->setValidationSchemaFullChecking(gSchemaFullChecking);
    //parser->setCreateEntityReferenceNodes(gDoCreate);

    // Parser and parser error handler
    DOMTreeErrorReporter *errReporter = new DOMTreeErrorReporter();
    parser->setErrorHandler(errReporter);

    //  Parse the XML buffer, catching any XML exceptions that
    //  might propogate out of it.
    bool errorsOccured = false;
    try {
        parser->parse(*memBufIS);
    } catch (const OutOfMemoryException&) {
        XERCES_STD_QUALIFIER cerr << "OutOfMemoryException" << XERCES_STD_QUALIFIER endl;
        errorsOccured = true;
    } catch (const XMLException& e) {
        XERCES_STD_QUALIFIER cerr << "An error occurred during parsing\n   Message: "
             << StrX(e.getMessage()) << XERCES_STD_QUALIFIER endl;
        errorsOccured = true;
    } catch (const DOMException& e) {
        const unsigned int maxChars = 2047;
        XMLCh errText[maxChars + 1];
        XERCES_STD_QUALIFIER cerr << "\nDOM Error during parsing message\n"
             << "DOMException code is:  " << e.code << XERCES_STD_QUALIFIER endl;
        if (DOMImplementation::loadDOMExceptionMsg(e.code, errText, maxChars))
             XERCES_STD_QUALIFIER cerr << "Message is: " << StrX(errText) << XERCES_STD_QUALIFIER endl;
        errorsOccured = true;
    } catch (...) {
        XERCES_STD_QUALIFIER cerr << "An error occurred during parsing\n " << XERCES_STD_QUALIFIER endl;
        errorsOccured = true;
    }

    // Get result
    XERCES_CPP_NAMESPACE::DOMDocument *result = NULL;
    // Check if any errors ocurred
    if (!errorsOccured && !errReporter->getSawErrors()) {
        result = parser->adoptDocument();
    } else {
        parser->adoptDocument()->release();
    }

    // Free memory
    delete memBufIS;
    delete parser;
    delete errReporter;
    return result;
}

std::string SimProxy::convert(const XMLCh * xString) {
    if (xString) {
        // Convert string.
        char * s =  XERCES_CPP_NAMESPACE::XMLString::transcode(xString);
        std::string retval(s);
        // Cleanup
        XERCES_CPP_NAMESPACE::XMLString::release(&s);
        return retval;
    } else {
        return std::string("");
    }
}

// Splits the values in the text with the following format: "(0.04,0.16,0.20)"
void SimProxy::splitValues(char*text, std::vector<char*>& values) {
    char* buf;
    char *value = __strtok_r(text," (,", &buf);
    while(value != NULL) {
        values.push_back(value);
        value = __strtok_r(NULL," (,)", &buf);
    }
}

std::string SimProxy::getString(XERCES_CPP_NAMESPACE::DOMDocument* doc) {
    std::string result;
    if (!doc) return result;
    try
    {
        // get a serializer, an instance of DOMWriter
        XMLCh tempStr[100];
        XMLString::transcode("LS", tempStr, 99);
        DOMImplementation *impl          = DOMImplementationRegistry::getDOMImplementation(tempStr);
        DOMWriter         *theSerializer = ((DOMImplementationLS*)impl)->createDOMWriter();

        // construct the MemBufFormatTarget
        XMLFormatTarget *myFormatTarget = new MemBufFormatTarget();

        // set the encoding to be Big5
        XMLCh* gOutputEncoding = XMLString::transcode("UTF-8");
        theSerializer->setEncoding(gOutputEncoding);
        XMLString::release(&gOutputEncoding);

        // plug in user's own error handler
        DOMErrorHandler *myErrorHandler = new DOMPrintErrorHandler();
        theSerializer->setErrorHandler(myErrorHandler);

        // set feature if the serializer supports the feature/mode

        //if (theSerializer->canSetFeature(XMLUni::fgDOMWRTSplitCdataSections, gSplitCdataSections))
        //    theSerializer->setFeature(XMLUni::fgDOMWRTSplitCdataSections, gSplitCdataSections);

        //if (theSerializer->canSetFeature(XMLUni::fgDOMWRTDiscardDefaultContent, gDiscardDefaultContent))
        //    theSerializer->setFeature(XMLUni::fgDOMWRTDiscardDefaultContent, gDiscardDefaultContent);

        if (theSerializer->canSetFeature(XMLUni::fgDOMWRTFormatPrettyPrint, true))
            theSerializer->setFeature(XMLUni::fgDOMWRTFormatPrettyPrint, true);


        //if (theSerializer->canSetFeature(XMLUni::fgDOMWRTBOM, gWriteBOM))
        //    theSerializer->setFeature(XMLUni::fgDOMWRTBOM, gWriteBOM);


        // Do the serialization to a string object

        // serialize a DOMNode to an internal memory buffer
        theSerializer->writeNode(myFormatTarget, *doc);
        // get the string which is encoded in UTF-8 from the MemBufFormatTarget
        result = ((char*) ((MemBufFormatTarget*)myFormatTarget)->getRawBuffer());

        // result = convert(theSerializer->writeToString(*doc));

        //printf("Parsed XML:\n%s\n", result.c_str());

        //XMLString::release(&gOutputEncoding);
        theSerializer->release();
        // Delete objects that are NOT owned by the serializer.
        delete myErrorHandler;
        delete myFormatTarget;

    }
     catch (const OutOfMemoryException&)
    {
        XERCES_STD_QUALIFIER cerr << "OutOfMemoryException" << XERCES_STD_QUALIFIER endl;
        //retval = 5;
    }
    catch (XMLException& e)
    {
        XERCES_STD_QUALIFIER cerr << "An error occurred during creation of output transcoder. Msg is:"
            << XERCES_STD_QUALIFIER endl
            << StrX(e.getMessage()) << XERCES_STD_QUALIFIER endl;
       // retval = 4;
    }
    return result;
}

void SimProxy::addAtomType(std::vector<char *>&atomTypes, const char* newAtomType) {
    //return;
    for (unsigned int i = 0; i < atomTypes.size(); i++) {
         if (strcmp(atomTypes[i],newAtomType) == 0) {
             return;
         }
    }
    atomTypes.push_back(strdup(newAtomType));
}

#if 0
void SimProxy::createBasicSensationEvalLinkElem(XERCES_CPP_NAMESPACE::DOMDocument* novamenteDoc,
                                                DOMElement* parentElem,
                                                DOMElement* agisimSensationNodeElem,
                                                DOMElement* attrElem,
                                                const char* predicateNodeName,
                                                std::vector<char*>& atomTypes,
                                                NodesMap* declaredNodes,
                                                bool isNumber,
                                                const char* timestamp) {

    std::vector<char*> values(1, XMLString::transcode(attrElem->getTextContent()));
    XMLString::trim(values[0]);
    createSensationEvalLinkElem(novamenteDoc, parentElem, agisimSensationNodeElem, values, predicateNodeName, atomTypes, declaredNodes, isNumber, timestamp);
    XMLString::release(&values[0]);
}

void SimProxy::createMultiArgSensationEvalLinkElem(XERCES_CPP_NAMESPACE::DOMDocument* novamenteDoc,
                                                   DOMElement* parentElem,
                                                   DOMElement* agisimSensationNodeElem,
                                                   DOMElement* attrElem,
                                                   const char* predicateNodeName,
                                                   std::vector<char*>& atomTypes,
                                                   NodesMap* declaredNodes,
                                                   bool isNumber,
                                                   const char* timestamp) {
    // Get the multiple values of the attribute.
    std::vector<char *> values;
    DOMNode* valueElem;
    for(valueElem = attrElem->getFirstChild(); valueElem != NULL; valueElem=valueElem->getNextSibling()) {
        if (valueElem->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
        }
        char* value = XMLString::transcode(valueElem->getTextContent());
        XMLString::trim(value);
        values.push_back(value);
    }
    createSensationEvalLinkElem(novamenteDoc, parentElem, agisimSensationNodeElem, values, predicateNodeName, atomTypes, declaredNodes, isNumber, timestamp);
    for (int i = 0; i < values.size(); i++) {
        XMLString::release(&values[i]);
    }
}
#endif

void SimProxy::processSelfDataElements(DOMElement* sensationElem) {
    //printf("SimProxy::processSelfDataElements()\n");
    DOMNode* childElem;
    for(childElem = sensationElem->getFirstChild(); childElem != NULL; childElem=childElem->getNextSibling()) {
        if (childElem->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
        }
        char *childElemName = XMLString::transcode(childElem->getNodeName());
        if (strcmp(childElemName, AGISIM_ENERGY_TAG) == 0) {
        } else if (strcmp(childElemName, AGISIM_POSITION_TAG) == 0) {
        } else if (strcmp(childElemName, AGISIM_CUSTOM_TAG) == 0) {
            //DOMElement* agisimPerceptNodeElem = createNodeElem(novamenteDoc, AGISIM_PERCEPT_NODE_CLASS, "", atomTypes, declaredNodes, timestamp);
            DOMNode* attrElem;
            bool holdingObjectEvent = false;
            bool isBroadcastMessage = false;
            DOMElement* messageAttrElem = NULL;
            for(attrElem = childElem->getFirstChild(); attrElem != NULL; attrElem=attrElem->getNextSibling()) {
                if (attrElem->getNodeType() != DOMNode::ELEMENT_NODE) {
                    continue;
                }
                char *attrElemName = XMLString::transcode(attrElem->getNodeName());
                if (strcmp(AGISIM_QUALITY_TAG, attrElemName) == 0) {
                    char* strQualityValue = XMLString::transcode(attrElem->getTextContent()); XMLString::trim(strQualityValue);
                    int qualityValue = atoi(strQualityValue);
                    switch (qualityValue) {
#ifdef CONVERT_ACTION_STARTED_EVENTS
                        // Check if this is an ACTION STARTED EVENT
                        case CUSTOM_SENSATION_MOVE_ANIMATION_STARTED:
                        case CUSTOM_SENSATION_TURN_ANIMATION_STARTED:
                        case CUSTOM_SENSATION_GENERIC_ANIMATION_STARTED:
                        case CUSTOM_SENSATION_SEMIPOSE_ANIMATION_STARTED:
                            {
                                //printf("SimProxy received ACTION STARTED EVENT: %s\n", strQualityValue);
                                logger().log(opencog::Logger::DEBUG, "SimProxy received ACTION STARTED EVENT: %s\n", strQualityValue);
                                unsigned long actionTicket = getActionInProgress(qualityValue-1);
                                if (actionTicket) {
                                    //printf("Dequeued action with ticket = %ld\n", actionTicket);
                                    logger().log(opencog::Logger::DEBUG, "Dequeued action with ticket = %ld\n", actionTicket);
                                    // TODO: If start event must be considered, the 2nd argument bellow must be an enum, not a boolean
                                    //perceptionAndStatusHandler->actionStatus(actionTicket, ???);
                                } else {
                                    //printf("WARN: Found no action in progress for this action type!\n");
                                    logger().log(opencog::Logger::WARN, "SimProxy: Found no action in progress for this action type!\n");
                                }
                            }
                            break;
#endif
                        // Check if this is an ACTION ENDED EVENT
                        case CUSTOM_SENSATION_MOVE_ANIMATION_DONE:
                        case CUSTOM_SENSATION_TURN_ANIMATION_DONE:
                        case CUSTOM_SENSATION_GENERIC_ANIMATION_DONE:
                        case CUSTOM_SENSATION_SEMIPOSE_ANIMATION_DONE:
                            {
                                //printf("SimProxy received ACTION END EVENT: %s\n", strQualityValue);
                                logger().log(opencog::Logger::DEBUG, "SimProxy received ACTION END EVENT: %s\n", strQualityValue);
                                unsigned long actionTicket = dequeueActionInProgress(qualityValue-2);
                                if (actionTicket) {
                                    //printf("Dequeued action with ticket = %ld\n", actionTicket);
                                    logger().log(opencog::Logger::DEBUG, "Dequeued action with ticket = %ld\n", actionTicket);
                                    perceptionAndStatusHandler->actionStatus(actionTicket, true);
                                } else {
                                    //printf("WARN: Found no action in progress for this action type!\n");
                                    logger().log(opencog::Logger::WARN, "SimProxy: Found no action in progress for this action type!\n");
                                }
                            }
                            break;
                        // Check if this is an ACTION FAILED EVENT
                        case CUSTOM_SENSATION_MOVE_ANIMATION_FAILED:
                        case CUSTOM_SENSATION_TURN_ANIMATION_FAILED:
                        case CUSTOM_SENSATION_GENERIC_ANIMATION_FAILED:
                        case CUSTOM_SENSATION_SEMIPOSE_ANIMATION_FAILED:
                            {
                                //printf("SimProxy received ACTION FAILED EVENT: %s\n", strQualityValue);
                                logger().log(opencog::Logger::DEBUG, "SimProxy received ACTION FAILED EVENT: %s\n", strQualityValue);
                                unsigned long actionTicket = dequeueActionInProgress(qualityValue-3);
                                if (actionTicket) {
                                    //printf("Dequeued action with ticket = %ld\n", actionTicket);
                                    logger().log(opencog::Logger::DEBUG, "Dequeued action with ticket = %ld\n", actionTicket);
                                    perceptionAndStatusHandler->actionStatus(actionTicket, false);
                                } else {
                                    //printf("WARN: Found no action in progress for this action type!\n");
                                    logger().log(opencog::Logger::WARN, "SimProxy: Found no action in progress for this action type!\n");
                                }
                            }
                            break;
                        case CUSTOM_SENSATION_HOLDING_OBJECT:
                            if (messageAttrElem) {
                                //printf("SimProxy received HOLDING OBJECT EVENT: object name = '%s'\n", XMLString::transcode(messageAttrElem->getTextContent()));
                                logger().log(opencog::Logger::DEBUG, "SimProxy received HOLDING OBJECT EVENT: object name = '%s'\n", XMLString::transcode(messageAttrElem->getTextContent()));
                                //createHoldingObjectElements(novamenteDoc, parentElem, messageAttrElem, atomTypes, declaredNodes, inc_stamp);
                            } else {
                                holdingObjectEvent = true;
                            }
                            break;
                        case CUSTOM_SENSATION_BROADCAST_MESSAGE:
                            if (messageAttrElem) {
                                char* fullMessage = XMLString::transcode(messageAttrElem->getTextContent());
                                //printf("SimProxy received CUSTOM_SENSATION_BROADCAST_MESSAGE: fullMessage = '%s'\n", fullMessage);
								if (logger().getLevel() >= opencog::Logger::FINE)
								    logger().log(opencog::Logger::FINE, "SimProxy received CUSTOM_SENSATION_BROADCAST_MESSAGE: fullMessage = '%s'\n", fullMessage);
                                XMLString::release(&fullMessage);
                                //createSayingMessageElements(novamenteDoc, parentElem, messageAttrElem, atomTypes, declaredNodes, timestamp);
                            } else {
                                isBroadcastMessage = true;
                            }
                        default:
                            // TODO: Check if this is the right handling for each remaining type/quality of CustomSensation
                            // (CUSTOM_SENSATION_ENERGY_GAIN, CUSTOM_SENSATION_ENERGY_LOSS, CUSTOM_SENSATION_OBJECT_TOO_FAR_TO_EAT, etc)
                            //createBasicSensationEvalLinkElem(novamenteDoc, parentElem, agisimPerceptNodeElem, (DOMElement*) attrElem, AGISIM_QUALITY_PREDICATE_NAME, atomTypes, declaredNodes, true, timestamp);
                            break;
                    }
                    XMLString::release(&strQualityValue);
                } else if (strcmp(AGISIM_INTENSITY_TAG, attrElemName) == 0) {
                    //createBasicSensationEvalLinkElem(novamenteDoc, parentElem, agisimPerceptNodeElem, (DOMElement*) attrElem, AGISIM_INTENSITY_PREDICATE_NAME, atomTypes, declaredNodes, true, timestamp);
                } else if (strcmp(AGISIM_MSG_TAG, attrElemName) == 0) {
                    messageAttrElem = (DOMElement*) attrElem;
                    //createBasicSensationEvalLinkElem(novamenteDoc, parentElem, agisimPerceptNodeElem, messageAttrElem, AGISIM_MESSAGE_PREDICATE_NAME, atomTypes, declaredNodes, false, timestamp);
                    if (holdingObjectEvent) {
                        char* objNameStr = XMLString::transcode(attrElem->getTextContent());
                        //printf("SimProxy received HOLDING OBJECT EVENT: object name = '%s'\n", objNameStr);
                        logger().log(opencog::Logger::DEBUG, "SimProxy received HOLDING OBJECT EVENT: object name = '%s'\n", objNameStr);
                        //createHoldingObjectElements(novamenteDoc, parentElem, messageAttrElem, atomTypes, declaredNodes, timestamp);
                        XMLString::release(&objNameStr);
                    }
                    if (isBroadcastMessage) {
                        char* fullMessage = XMLString::transcode(attrElem->getTextContent());
                        //printf("SimProxy received CUSTOM_SENSATION_BROADCAST_MESSAGE: fullMessage = '%s'\n", fullMessage);
						if (logger().getLevel() >= opencog::Logger::FINE)
                    	    logger().log(opencog::Logger::FINE, "SimProxy received CUSTOM_SENSATION_BROADCAST_MESSAGE: fullMessage = '%s'\n", fullMessage);
                        XMLString::release(&fullMessage);
                        //createSayingMessageElements(novamenteDoc, parentElem, messageAttrElem, atomTypes, declaredNodes, timestamp);
                    }
                }
                XMLString::release(&attrElemName);
            }
        }
        XMLString::release(&childElemName);
    }
}

void SimProxy::processMapInfoElements(DOMElement* sensationElem) {
    //printf("SimProxy::processMapInfoElements()\n");

    std::vector<ObjMapInfo> objects;

    XMLCh tag[MAX_TAG_LENGTH+1];

    XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_TAG, tag, MAX_TAG_LENGTH);
    XERCES_CPP_NAMESPACE::DOMNodeList * objList = sensationElem->getElementsByTagName(tag);
    for(unsigned int i = 0; i < objList->getLength(); i++){
        ObjMapInfo objMapInfo;

        XERCES_CPP_NAMESPACE::DOMElement* objElement = (XERCES_CPP_NAMESPACE::DOMElement*) objList->item(i);
        //printf("SimProxy: got info for an object:\n");

        XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_NAME_TAG, tag, MAX_TAG_LENGTH);
        XERCES_CPP_NAMESPACE::DOMElement* nameElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
        char* name = XERCES_CPP_NAMESPACE::XMLString::transcode(nameElement->getTextContent());
        XERCES_CPP_NAMESPACE::XMLString::trim(name);
        //printf("==========================================\nSimProxy: name: %s\n", name);

        XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_TYPE_TAG, tag, MAX_TAG_LENGTH);
        XERCES_CPP_NAMESPACE::DOMElement* typeElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
        char* type = XERCES_CPP_NAMESPACE::XMLString::transcode(typeElement->getTextContent());
        XERCES_CPP_NAMESPACE::XMLString::trim(type);
        //printf("==========================================\nSimProxy: type: %s\n", type);

        XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_REMOVE_TAG, tag, MAX_TAG_LENGTH);
        XERCES_CPP_NAMESPACE::DOMElement* removeElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
        char* remove = XERCES_CPP_NAMESPACE::XMLString::transcode(removeElement->getTextContent());
        XERCES_CPP_NAMESPACE::XMLString::trim(remove);
        //printf("==========================================\nSimProxy: remove: '%s'\n", remove);

	objMapInfo.name = name;
	objMapInfo.type = type;
        objMapInfo.removed = !strcmp(remove,"true");

        if (!objMapInfo.removed) {
            XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_POSITION_TAG, tag, MAX_TAG_LENGTH);
            XERCES_CPP_NAMESPACE::DOMElement* posElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
            char* pos = XERCES_CPP_NAMESPACE::XMLString::transcode(posElement->getTextContent());
            XERCES_CPP_NAMESPACE::XMLString::trim(pos);
            //printf("SimProxy: pos: %s\n", pos);
            double posX,posY,posZ;
            sscanf(pos, "%lf,%lf,%lf", &posX,&posY,&posZ);
            //printf("SimProxy: posX: %lf, posY: %lf, posZ: %lf\n", posX, posY, posZ);

            XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_ROTATION_TAG, tag, MAX_TAG_LENGTH);
            XERCES_CPP_NAMESPACE::DOMElement* rotElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
            char* rot = XERCES_CPP_NAMESPACE::XMLString::transcode(rotElement->getTextContent());
            XERCES_CPP_NAMESPACE::XMLString::trim(rot);
            //printf("SimProxy: rot: %s\n", rot);
            double rotX,rotY,rotZ;
            sscanf(rot, "%lf,%lf,%lf", &rotX,&rotY,&rotZ);
            //printf("SimProxy: rotX: %lf, rotY: %lf, rotZ: %lf\n", rotX, rotY, rotZ);

            XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_SIZE_TAG, tag, MAX_TAG_LENGTH);
            XERCES_CPP_NAMESPACE::DOMElement* sizeElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
            char* size = XERCES_CPP_NAMESPACE::XMLString::transcode(sizeElement->getTextContent());
            XERCES_CPP_NAMESPACE::XMLString::trim(size);
            //printf("SimProxy: size: %s\n", size);
            double length,width,height;
            sscanf(size, "%lf,%lf,%lf", &length,&height,&width);
            //printf("SimProxy: length: %lf, width: %lf, height: %lf\n", length, width, height);

            XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_EDIBLE_TAG, tag, MAX_TAG_LENGTH);
            XERCES_CPP_NAMESPACE::DOMElement* edibleElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
            char* edible = XERCES_CPP_NAMESPACE::XMLString::transcode(edibleElement->getTextContent());
            XERCES_CPP_NAMESPACE::XMLString::trim(edible);
            //printf("SimProxy: edible: %s\n", edible);
            bool isEdible = !strcmp(edible, "true");

            XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_DRINKABLE_TAG, tag, MAX_TAG_LENGTH);
            XERCES_CPP_NAMESPACE::DOMElement* drinkableElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
            char* drinkable = XERCES_CPP_NAMESPACE::XMLString::transcode(drinkableElement->getTextContent());
            XERCES_CPP_NAMESPACE::XMLString::trim(drinkable);
            //printf("SimProxy: drinkable: %s\n", drinkable);
            bool isDrinkable = !strcmp(drinkable, "true");

            XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_PETHOME_TAG, tag, MAX_TAG_LENGTH);
            XERCES_CPP_NAMESPACE::DOMElement* petHomeElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
            char* petHome = XERCES_CPP_NAMESPACE::XMLString::transcode(petHomeElement->getTextContent());
            XERCES_CPP_NAMESPACE::XMLString::trim(petHome);
            //printf("SimProxy: petHome: %s\n", drinkable);
            bool isPetHome = !strcmp(petHome, "true");

            XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_FOODBOWL_TAG, tag, MAX_TAG_LENGTH);
            XERCES_CPP_NAMESPACE::DOMElement* foodBowlElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
            char* foodBowl = XERCES_CPP_NAMESPACE::XMLString::transcode(foodBowlElement->getTextContent());
            XERCES_CPP_NAMESPACE::XMLString::trim(foodBowl);
            //printf("SimProxy: drinkable: %s\n", drinkable);
            bool isFoodBowl = !strcmp(foodBowl, "true");

            XERCES_CPP_NAMESPACE::XMLString::transcode(AGISIM_OBJECT_WATERBOWL_TAG, tag, MAX_TAG_LENGTH);
            XERCES_CPP_NAMESPACE::DOMElement* waterBowlElement = (XERCES_CPP_NAMESPACE::DOMElement*) objElement->getElementsByTagName(tag)->item(0);
            char* waterBowl = XERCES_CPP_NAMESPACE::XMLString::transcode(waterBowlElement->getTextContent());
            XERCES_CPP_NAMESPACE::XMLString::trim(waterBowl);
            //printf("SimProxy: drinkable: %s\n", drinkable);
            bool isWaterBowl = !strcmp(waterBowl, "true");

            objMapInfo.posX = posX;
            objMapInfo.posY = posY;
            objMapInfo.posZ = posZ;
            objMapInfo.rotX = rotX;
            objMapInfo.rotY = rotY;
            objMapInfo.rotZ = rotZ;
            objMapInfo.length = length;
            objMapInfo.width = width;
            objMapInfo.height = height;
            objMapInfo.edible = isEdible;
            objMapInfo.drinkable = isDrinkable;
            objMapInfo.petHome = isPetHome;
            objMapInfo.foodBowl = isFoodBowl;
            objMapInfo.waterBowl = isWaterBowl;

            XERCES_CPP_NAMESPACE::XMLString::release(&pos);
            XERCES_CPP_NAMESPACE::XMLString::release(&rot);
            XERCES_CPP_NAMESPACE::XMLString::release(&size);
            XERCES_CPP_NAMESPACE::XMLString::release(&edible);
            XERCES_CPP_NAMESPACE::XMLString::release(&drinkable);
            XERCES_CPP_NAMESPACE::XMLString::release(&petHome);
            XERCES_CPP_NAMESPACE::XMLString::release(&foodBowl);
            XERCES_CPP_NAMESPACE::XMLString::release(&waterBowl);
        }

        XERCES_CPP_NAMESPACE::XMLString::release(&name);
        XERCES_CPP_NAMESPACE::XMLString::release(&type);
        XERCES_CPP_NAMESPACE::XMLString::release(&remove);

	objects.push_back(objMapInfo);
    }
    perceptionAndStatusHandler->mapInfo(objects);
}

#if 0
void SimProxy::createObjectPerceptElements(XERCES_CPP_NAMESPACE::DOMDocument *novamenteDoc,
                                           DOMElement* parentElem,
                                           DOMElement* sensationElem,
                                           std::vector<char*>& atomTypes,
                                           NodesMap* declaredNodes,
                                           const char* timestamp) {
    DOMNode* objectNode;
if (using_single_component_mode) {
    for(objectNode = sensationElem->getFirstChild(); objectNode != NULL; objectNode=objectNode->getNextSibling()) {
        if (objectNode->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
        }
        DOMElement* objectElem = (DOMElement*) objectNode;
        // GET OBJECT NAME
        DOMNodeList* nodeList = objectElem->getElementsByTagName(X(AGISIM_NAME_TAG));
        char* nodeName = XMLString::transcode(nodeList->item(0)->getTextContent());
        XMLString::trim(nodeName);
        // Remove "/null" part
        char* p = nodeName; while (*p != '\0') { if (*p == '/') *p = '\0'; else p++; };
        DOMElement* agisimObjectPerceptNodeElem = createNodeElem(novamenteDoc, AGISIM_OBJECT_PERCEPT_NODE_CLASS, nodeName, atomTypes, declaredNodes, timestamp);
        XMLString::release(&nodeName);
        DOMNode* attrElem;
        for(attrElem = objectNode->getFirstChild(); attrElem != NULL; attrElem=attrElem->getNextSibling()) {
            if (attrElem->getNodeType() != DOMNode::ELEMENT_NODE) {
                continue;
            }
            char *attrElemName = XMLString::transcode(attrElem->getNodeName());
            if (strcmp(AGISIM_BRIGHTNESS_TAG, attrElemName) == 0) {
                createBasicSensationEvalLinkElem(novamenteDoc, parentElem, agisimObjectPerceptNodeElem, (DOMElement*) attrElem, AGISIM_BRIGHTNESS_PREDICATE_NAME, atomTypes, declaredNodes, true, timestamp);
            } else if (strcmp(AGISIM_FOV_POS_TAG, attrElemName) == 0) {
                createMultiArgSensationEvalLinkElem(novamenteDoc, parentElem, agisimObjectPerceptNodeElem, (DOMElement*) attrElem, AGISIM_POSITION_PREDICATE_NAME, atomTypes, declaredNodes, true, timestamp);
            }
            XMLString::release(&attrElemName);
        }
    }
} else {
    for(objectNode = sensationElem->getFirstChild(); objectNode != NULL; objectNode=objectNode->getNextSibling()) {
        if (objectNode->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
        }
        DOMElement* objectElem = (DOMElement*) objectNode;
        // GET OBJECT NAME
        DOMNodeList* nodeList = objectElem->getElementsByTagName(X(AGISIM_NAME_TAG));
        char* nodeName = XMLString::transcode(nodeList->item(0)->getTextContent());
        XMLString::trim(nodeName);
        // Creates an evaluationLink for Brightness predicate only if Node was not mentioned before (since the brighness of a same object does not change)
        auto_ptr<char> key(new char[1024]);
        sprintf(key.get(), "%s_%s", AGISIM_OBJECT_PERCEPT_NODE_CLASS, nodeName);
        NodesMapIterator ti =  declaredNodes->find(key.get());
        bool createBrightness = (ti == declaredNodes->end());
        // Create a new element for this object
        DOMElement* agisimObjectPerceptNodeElem = createNodeElem(novamenteDoc, AGISIM_OBJECT_PERCEPT_NODE_CLASS, nodeName, atomTypes, declaredNodes, timestamp);
        if (createBrightness) {
            // GET OBJECT BRIGHTNESS
            nodeList = objectElem->getElementsByTagName(X(AGISIM_BRIGHTNESS_TAG));
            createBasicSensationEvalLinkElem(novamenteDoc, parentElem, agisimObjectPerceptNodeElem,
                (DOMElement*) nodeList->item(0), AGISIM_BRIGHTNESS_PREDICATE_NAME, atomTypes, declaredNodes, true, timestamp);
        }
        free(nodeName);
        // GET POLYGON OBJECT NAME
        nodeList = objectElem->getElementsByTagName(X(AGISIM_POLYGON_NAME_TAG));
        nodeName = XMLString::transcode(nodeList->item(0)->getTextContent());
        XMLString::trim(nodeName);
        // Create an Element for this polygon
        DOMElement* agisimPolygonPerceptNodeElem = createNodeElem(novamenteDoc, AGISIM_POLYGON_PERCEPT_NODE_CLASS, nodeName, atomTypes, declaredNodes, timestamp);
        // Create a member link between Polygon and Object
        DOMElement* memberLinkElem = createLinkElem(novamenteDoc, MEMBER_LINK_CLASS, atomTypes, timestamp);
        memberLinkElem->appendChild(agisimPolygonPerceptNodeElem);
        memberLinkElem->appendChild(createBrightness?agisimObjectPerceptNodeElem->cloneNode(false):agisimObjectPerceptNodeElem);
        parentElem->appendChild(memberLinkElem);
        // Get Polygon Node declaration element
        sprintf(key.get(), "%s_%s", AGISIM_POLYGON_PERCEPT_NODE_CLASS, nodeName);
        XMLString::release(&nodeName);
        DOMElement* agisimPolygonNodeDeclarationElem = declaredNodes->find(key.get())->second;
        // GET POLYGON CORNERS
        nodeList = objectElem->getElementsByTagName(X(AGISIM_POLYGON_CORNER_TAG));
        for (unsigned int i = 0; i < nodeList->getLength(); i++) {
            // Create the corner element in the polygonNode element.
            DOMElement* cornerElem = novamenteDoc->createElement(X(POLYGON_CORNER_TOKEN));
            char* text = XMLString::transcode(((DOMElement*) nodeList->item(i))->getTextContent());
            std::vector<char*> values;
            splitValues(text, values);
            if (values.size() != 3) {
                printf("ERROR: Got wrong number of coordinates for polygon corner: %d\n", values.size());
            } else {
                cornerElem->setAttribute(X(POLYGON_CORNER_X_TOKEN), XMLString::transcode(values[0]));
                cornerElem->setAttribute(X(POLYGON_CORNER_Y_TOKEN), XMLString::transcode(values[1]));
                cornerElem->setAttribute(X(POLYGON_CORNER_Z_TOKEN), XMLString::transcode(values[2]));
                agisimPolygonNodeDeclarationElem->appendChild(cornerElem);
            }
            XMLString::release(&text);
        }
    }
}
}

void SimProxy::createHoldingObjectElements(XERCES_CPP_NAMESPACE::DOMDocument* novamenteDoc,
                                      DOMElement* parentElem,
                                      DOMElement* objectNameElem,
                                      std::vector<char*>& atomTypes,
                                      NodesMap* declaredNodes,
                                      const char* timestamp) {

    DOMElement* evalLinkElem = createLinkElem(novamenteDoc, EVALUATION_LINK_CLASS, atomTypes, timestamp);
    // PredicateNode element
    DOMElement* predicateNodeElem = createNodeElem(novamenteDoc, PREDICATE_NODE_CLASS, HOLDING_OBJECT_PREDICATE_NAME, atomTypes, declaredNodes, timestamp);
    evalLinkElem->appendChild(predicateNodeElem);
    // PredicateNode always use ListLink for its arguments.
    DOMElement* listLinkElem = createLinkElem(novamenteDoc, LIST_LINK_CLASS, atomTypes, timestamp);
    evalLinkElem->appendChild(listLinkElem);
    char* objectName = XMLString::transcode(objectNameElem->getTextContent());
    XMLString::trim(objectName);
    DOMElement* wordNodeElem = createNodeElem(novamenteDoc, WORD_NODE_CLASS, objectName, atomTypes, declaredNodes, timestamp);

    listLinkElem->appendChild(wordNodeElem);
    // Finally, appends the eval link to the doc.
    parentElem->appendChild(evalLinkElem);
}

void SimProxy::createSayingMessageElements(XERCES_CPP_NAMESPACE::DOMDocument* novamenteDoc,
                                      DOMElement* parentElem,
                                      DOMElement* objectNameElem,
                                      std::vector<char*>& atomTypes,
                                      NodesMap* declaredNodes,
                                      const char* timestamp) {

    // Parses the full message and gets the demon/agent name and the said message.
    char* fullMessage = XMLString::transcode(objectNameElem->getTextContent());
    XMLString::trim(fullMessage);
    char* msg = strchr(fullMessage, ':');
    if (msg == NULL) return;
    char* demonName = fullMessage;
    *msg = '\0';
    msg++;

#ifdef BROADCAST_MESSAGE_GOES_TO_PENDING_ACTIONS
    if (!strcmp(demonName, agentName.c_str())) {
        XERCES_CPP_NAMESPACE::DOMDocument* actionDoc = parseXML(dequeueActionInProgress(CUSTOM_SENSATION_BROADCAST_MESSAGE));
        if (actionDoc) {
            actionDoc->release();
        } else {
            printf("WARNING: Found no SAY action in progress for agent %s! Message = '%s'\n", demonName, msg);
        }
    }
#endif

    DOMElement* evalLinkElem = createLinkElem(novamenteDoc, EVALUATION_LINK_CLASS, atomTypes, timestamp);
    DOMElement* predicateNodeElem = createNodeElem(novamenteDoc, PREDICATE_NODE_CLASS, ACTION_DONE_PREDICATE_NAME, atomTypes, declaredNodes, timestamp);
    evalLinkElem->appendChild(predicateNodeElem);
    DOMElement* listLinkElem = createLinkElem(novamenteDoc, LIST_LINK_CLASS, atomTypes, timestamp);
    evalLinkElem->appendChild(listLinkElem);

    DOMElement* execLinkElem = createLinkElem(novamenteDoc, EXECUTION_LINK_CLASS, atomTypes, timestamp);
    listLinkElem->appendChild(execLinkElem);

    DOMElement* schemaNodeElem = createNodeElem(novamenteDoc, GROUNDED_SCHEMA_NODE_CLASS, SAY_SCHEMA_NAME, atomTypes, declaredNodes, timestamp);
    execLinkElem->appendChild(schemaNodeElem);
    DOMElement* execListLinkElem = createLinkElem(novamenteDoc, LIST_LINK_CLASS, atomTypes, timestamp);
    execLinkElem->appendChild(execListLinkElem);
    DOMElement* wordNodeElem = createNodeElem(novamenteDoc, WORD_NODE_CLASS, demonName, atomTypes, declaredNodes, timestamp);
    execListLinkElem->appendChild(wordNodeElem);
    DOMElement* sentenceNodeElem = createNodeElem(novamenteDoc, SENTENCE_NODE_CLASS, msg, atomTypes, declaredNodes, timestamp);
    execListLinkElem->appendChild(sentenceNodeElem);

    parentElem->appendChild(evalLinkElem);
}
#endif


unsigned long SimProxy::enqueueActionInProgress(int actionType) {
    //printf("SimProxy::enqueueActionInProgress()\n");
    logger().log(opencog::Logger::DEBUG, "SimProxy::enqueueActionInProgress()\n");

    // Check if this action type is already associated with any in progress action
    EventId2AtomRepMap::iterator itr = eventMap->find(actionType);
    std::deque<unsigned long> actionList;
    if (itr != eventMap->end()) {
        actionList = (*itr).second;
    }
    // Gets the current ticket and sets the next
    unsigned long result = nextActionTicket;
    nextActionTicket++;
    if (nextActionTicket == ULONG_MAX - 1) nextActionTicket = 1; // circular selection, excluding 0 and ULONG_MAX, ULONG_MAX - 1
    // Stores it as the last sent command of the corresponding type
    actionList.push_back(result);
    (*eventMap)[actionType] = actionList;
    //printf("SimProxy::enqueueActionInProgress() OK!\n");
    //printf("Action list length = %d\n", actionList.size());
    //printf("Number of enqueued actions = %d\n", ++numberOfPendingActions);
    logger().log(opencog::Logger::DEBUG, "Number of enqueued actions = %d\n", ++numberOfPendingActions);
    return result;
}

unsigned long SimProxy::dequeueActionInProgress(int actionType) {
    //printf("SimProxy::dequeueActionInProgress()\n");
    unsigned long actionTicket = 0;
    EventId2AtomRepMap::iterator itr = eventMap->find(actionType);
    if (itr != eventMap->end()) {
        //printf("Found an associated list to this action type\n");
    std::deque<unsigned long> actionList = (*itr).second;
        actionTicket = actionList.front();
        actionList.pop_front();
        if (actionList.empty()) {
            actionList.clear();
            eventMap->erase(actionType);
        } else {
            (*eventMap)[actionType] = actionList;
        }
        //printf("Action list length = %d\n", actionList.size());
        //printf("Number of enqueued actions = %d\n", --numberOfPendingActions);
        logger().log(opencog::Logger::DEBUG, "Number of enqueued actions = %d\n", --numberOfPendingActions);
    } else {
        //printf("WARN: There is no associated list to this action type\n");
        logger().log(opencog::Logger::WARN, "SimProxy: There is no associated list to this action type\n");
    }
    for (itr = eventMap->begin(); itr != eventMap->end(); itr++) {
        std::cout << (*itr).first << " => " << (*itr).second.size() << " entries."  << std::endl;
    }
    return actionTicket;
}

unsigned long SimProxy::getActionInProgress(int actionType) {
    //printf("SimProxy::getActionInProgress()\n");
    unsigned long actionTicket = 0;
    EventId2AtomRepMap::iterator itr = eventMap->find(actionType);
    if (itr != eventMap->end()) {
        //printf("Found an associated list to this action type\n");
        std::deque<unsigned long> actionList = (*itr).second;
        actionTicket = actionList.front();
    } else {
        //printf("WARNING: There is no associated list to this action type\n");
        logger().log(opencog::Logger::WARN, "SimProxy: There is no associated list to this action type\n");
    }
    return actionTicket;
}

#if 0
XERCES_CPP_NAMESPACE::DOMDocument* SimProxy::createActionDocument(const char* actionSchemaName,
                                                     const char* firstArgStr,
                                                     const char* firstArgAtomClass,
                                                     const char* secondArgStr,
                                                     const char* secondArgAtomClass) {
    // Creates the new DOM document
    XERCES_CPP_NAMESPACE::DOMDocument* actionDoc = domImplementation->createDocument(NULL, X(LIST_TOKEN), NULL);

    // Temporary data structures to store the used atom types and nodes.
    std::vector<char*> atomTypes;
    NodesMap* declaredNodes = new NodesMap();
    char timestamp[128];
    sprintf(timestamp, "%lu", getElapsedMillis());

    DOMElement*  descElem = actionDoc->createElement(X(TAG_DESCRIPTION_TOKEN));
    actionDoc->getDocumentElement()->appendChild(descElem);

    DOMElement* evalLinkElem = createLinkElem(actionDoc, EVALUATION_LINK_CLASS, atomTypes, timestamp);
    DOMElement* predicateNodeElem = createNodeElem(actionDoc, PREDICATE_NODE_CLASS, ACTION_TRIED_PREDICATE_NAME, atomTypes, declaredNodes, timestamp);
    evalLinkElem->appendChild(predicateNodeElem);
    DOMElement* evalListLinkElem = createLinkElem(actionDoc, LIST_LINK_CLASS, atomTypes, timestamp);
    evalLinkElem->appendChild(evalListLinkElem);
    DOMElement* execLinkElem = createLinkElem(actionDoc, EXECUTION_LINK_CLASS, atomTypes, timestamp);
    evalListLinkElem->appendChild(execLinkElem);
    DOMElement* schemaNodeElem = createNodeElem(actionDoc, GROUNDED_SCHEMA_NODE_CLASS, actionSchemaName, atomTypes, declaredNodes, timestamp);
    execLinkElem->appendChild(schemaNodeElem);
    DOMElement* execListLinkElem = createLinkElem(actionDoc, LIST_LINK_CLASS, atomTypes, timestamp);
    execLinkElem->appendChild(execListLinkElem);
    if (firstArgStr != NULL) {
        DOMElement* firstArgNodeElem = createNodeElem(actionDoc, firstArgAtomClass, firstArgStr, atomTypes, declaredNodes, timestamp);
        execListLinkElem->appendChild(firstArgNodeElem);
    }
    if (secondArgStr != NULL) {
        DOMElement* secondArgNodeElem = createNodeElem(actionDoc, secondArgAtomClass, secondArgStr, atomTypes, declaredNodes, timestamp);
        execListLinkElem->appendChild(secondArgNodeElem);
    }
    actionDoc->getDocumentElement()->appendChild(evalLinkElem);

    addTypeDescriptionsAndFreeAuxiliarData(actionDoc, descElem, atomTypes, declaredNodes);

    return actionDoc;
}

void SimProxy::addTypeDescriptionsAndFreeAuxiliarData(XERCES_CPP_NAMESPACE::DOMDocument* novamenteDoc,
                                                      DOMElement* descElem,
                                                      std::vector<char*>& atomTypes,
                                                      NodesMap* declaredNodes) {
        for(int i = 0; i < atomTypes.size(); i++) {
            DOMElement* tagElem = novamenteDoc->createElement(X(TAG_TOKEN));
            tagElem->setAttribute(X(NAME_TOKEN), X(atomTypes[i]));
            tagElem->setAttribute(X(VALUE_TOKEN), X(atomTypes[i]));
            descElem->appendChild(tagElem);
            free(atomTypes[i]);
        }
        atomTypes.clear();

        NodesMapIterator current = declaredNodes->begin();
        while (current != declaredNodes->end()) {
            char* key = current->first;
            declaredNodes->erase(current);
            free(key);
            current = declaredNodes->begin();
        }
        delete declaredNodes;
}
#endif

void SimProxy::processPerceptionAndStatus(XERCES_CPP_NAMESPACE::DOMDocument* agiSimMessageDoc) {
    //printf("SimProxy::processPerceptionAndStatus\n");
    int errorCode = 0;
    try {
        // Check if root element is ok
        DOMElement* sensationElem = agiSimMessageDoc->getDocumentElement();
        char* sensationElemName = XMLString::transcode(sensationElem->getNodeName());
        if (strcmp(AGISIM_SENSATION_ROOT_TAG, sensationElemName) != 0) {
            XMLString::release(&sensationElemName);
            return;
        }
        XMLString::release(&sensationElemName);

        DOMNode* child;
        for (child = sensationElem->getFirstChild(); child != NULL; child=child->getNextSibling()) {
           if (child->getNodeType() != DOMNode::ELEMENT_NODE) {
               continue;
           }
           char *childNodeName = XMLString::transcode(child->getNodeName());
           if (strcmp(AGISIM_SMELL_DATA_TAG,childNodeName) == 0) {
               //printf("GOT smell data\n");
               //createBasicSensationElements(novamenteDoc, outterAndLinkElem, (DOMElement*) child, AGISIM_SMELL_NODE_CLASS, atomTypes, declaredNodes, timestamp);
           } else if (strcmp(AGISIM_TASTE_DATA_TAG,childNodeName) == 0) {
               //printf("GOT taste data\n");
               //createBasicSensationElements(novamenteDoc, outterAndLinkElem, (DOMElement*) child, AGISIM_TASTE_NODE_CLASS, atomTypes, declaredNodes, timestamp);
           } else if (strcmp(AGISIM_AUDIO_DATA_TAG,childNodeName) == 0) {
               //printf("GOT sound data\n");
               //createBasicSensationElements(novamenteDoc, outterAndLinkElem, (DOMElement*) child, AGISIM_SOUND_NODE_CLASS, atomTypes, declaredNodes, timestamp);
           } else if (strcmp(AGISIM_SELF_DATA_TAG,childNodeName) == 0) {
               //printf("GOT self data\n");
               processSelfDataElements((DOMElement*) child);
           } else if (strcmp(AGISIM_OBJECT_VISUAL_TAG,childNodeName) == 0) {
               //printf("GOT object visual data\n");

               //createObjectPerceptElements(novamenteDoc, outterAndLinkElem, (DOMElement*) child, atomTypes, declaredNodes, timestamp);
           } else if (strcmp(AGISIM_VISUAL_TAG,childNodeName) == 0) {
               //printf("GOT pixel visual data\n");
               //createPixelPerceptElements(novamenteDoc, outterAndLinkElem, (DOMElement*) child, atomTypes, declaredNodes, timestamp);
           } else if (strcmp(AGISIM_MAP_INFO_TAG,childNodeName) == 0) {
               //printf("GOT map info data\n");
               logger().log(opencog::Logger::DEBUG, "GOT map info data\n");
               processMapInfoElements((DOMElement*) child);
           } else {
               //printf("WARN: Unknown sensation data tag: %s\n", childNodeName);
               logger().log(opencog::Logger::WARN, "SimProxy: Unknown sensation data tag: %s\n", childNodeName);
           }
           XMLString::release(&childNodeName);
        }

    } catch (const OutOfMemoryException&) {
           XERCES_STD_QUALIFIER cerr << "OutOfMemoryException" << XERCES_STD_QUALIFIER endl;
       errorCode = 5;
    } catch (const DOMException& e) {
       XERCES_STD_QUALIFIER cerr << "DOMException code is:  " << e.code << XERCES_STD_QUALIFIER endl;
       errorCode = 2;
    } catch (...) {
       XERCES_STD_QUALIFIER cerr << "An error occurred creating the document" << XERCES_STD_QUALIFIER endl;
       errorCode = 3;
    }
}

bool SimProxy::isAdminResponse(XERCES_CPP_NAMESPACE::DOMDocument* agiSimMessageDoc) {
    // Check if root element is ok
    char* rootElemName = XMLString::transcode(agiSimMessageDoc->getDocumentElement()->getNodeName());
    if (strcmp(AGISIM_ADMIN_ROOT_TAG, rootElemName) != 0) {
        XMLString::release(&rootElemName);
        return false;
    }
    XMLString::release(&rootElemName);
    return true;
}

bool SimProxy::processAdminResponse(XERCES_CPP_NAMESPACE::DOMDocument* agiSimMessageDoc, std::string& message) {
    DOMNodeList* msgElems = agiSimMessageDoc->getDocumentElement()->getElementsByTagName(X(AGISIM_ADMIN_MSG_TAG));
    if (msgElems->getLength() > 0) {
        char* msg = XMLString::transcode(msgElems->item(0)->getTextContent());
        message += msg;
        XMLString::release(&msg);
        return true;
    } else {
        DOMNodeList* errorElems = agiSimMessageDoc->getDocumentElement()->getElementsByTagName(X(AGISIM_ADMIN_ERROR_TAG));
        if (errorElems->getLength() > 0) {
            char* error = XMLString::transcode(errorElems->item(0)->getTextContent());
            message += error;
            XMLString::release(&error);
            return false;
        }
    }
    message += "Could not find '";
    message += AGISIM_ADMIN_MSG_TAG;
    message += "' or '";
    message += AGISIM_ADMIN_ERROR_TAG;
    message += "' tag.\n";
    return false;
}

std::string SimProxy::processAgiSimMessage(const std::string& agiSimMessage, bool isAdminCommand) {
    std::string result;
    //printf("SimProxy::processAgiSimMessage():\n%s\n", agiSimMessage.c_str());
	if (logger().getLevel() >= opencog::Logger::FINE)
	    logger().log(opencog::Logger::FINE, "SimProxy::processAgiSimMessage():\n%s\n", agiSimMessage.c_str());
    XERCES_CPP_NAMESPACE::DOMDocument* agiSimMessageDoc = parseXML(agiSimMessage);
    if (agiSimMessageDoc != NULL) {
        if (isAdminResponse(agiSimMessageDoc)) {
            //printf("admin response!\n");
            logger().log(opencog::Logger::DEBUG, "admin response!\n");
            std::string message;
            if (!processAdminResponse(agiSimMessageDoc, message)) {
                perceptionAndStatusHandler->errorNotification(message);
            }
            result = message;
        } else {
            //printf("non-admin response!\n");
            logger().log(opencog::Logger::DEBUG, "non-admin response!\n");
            processPerceptionAndStatus(agiSimMessageDoc);
        }
        agiSimMessageDoc->release();
    } else {
        std::string errorMessage;
        errorMessage += "Invalid xml response: ";
        errorMessage += agiSimMessage;
        perceptionAndStatusHandler->errorNotification(errorMessage);
        result = errorMessage;
    }
    return result;
}

std::string SimProxy::send(const std::string &str, bool isAdminCommand, bool waitResponse) {
    //printf("SimProxy::send(\"%s\", %d, %d)\n", str.c_str(), isAdminCommand, waitResponse);
    //printf("AgentName = %s\n", agentName.c_str());
	if (logger().getLevel() >= opencog::Logger::FINE){
	    logger().log(opencog::Logger::FINE, "SimProxy::send(\"%s\", %d, %d)\n", str.c_str(), isAdminCommand, waitResponse);
    	logger().log(opencog::Logger::DEBUG, "AgentName = %s\n", agentName.c_str());
	}
	std::string result;

    timeval beginTime;
    gettimeofday(&beginTime, NULL);

    if (waitResponse && str.size() > 0) {
        // to get any old/spontaneous responses
        checkAsynchronousMessages();
        //sh->Select(SEC_DELAY, USEC_DELAY);
    }

    timeval curTime;
    //gettimeofday(&curTime, NULL);
    //printf("SPENT TIME TO RECEIVE SPONTANEOUS MESSAGE = %ld\n", ((curTime.tv_sec-beginTime.tv_sec)*1000)+((curTime.tv_usec-beginTime.tv_usec)/1000));
    if (echoing) {
        //printf("Agisim Client (@%ld.%ld ms) << '%s'\n", beginTime.tv_usec/1000, beginTime.tv_usec%1000, str.c_str());
        logger().log(opencog::Logger::DEBUG, "SimProxy - Send - Agisim Client (@%ld.%ld ms) << '%s'\n", beginTime.tv_usec/1000, beginTime.tv_usec%1000, str.c_str());
    }
    if (cc->IsConnected()) {
        if (str.size() > 0) {
            cc->Send(str);
        }
        if (waitResponse) {
            //printf("Waiting response...\n");
            logger().log(opencog::Logger::DEBUG, "Waiting response...\n");
            std::string agiSimMessage;
            cc->markWaitingForResponse();
            sh->Select(SEC_DELAY, USEC_DELAY);
            while (cc->isWaitingForResponse() && !cc->ConnectionFailed()) {
                timeval theTime;
                gettimeofday(&theTime, NULL);
                if (echoing) {
    //                printf("Polling @%ld.%ld  ms\n", theTime.tv_sec*1000 + theTime.tv_usec/1000, theTime.tv_usec%1000);
                }
                if (timeout(beginTime,theTime,WAIT_RESPONSE_TIMEOUT)) {
                    break;
                }
                sh->Select(SEC_DELAY, USEC_DELAY);
                //if (cc->ConnectionFailed()) {
                //       break;
                //}
            } // while
            if (!cc->isWaitingForResponse()) {
                agiSimMessage = cc->getResponse();
            } else {
                // Update current state of SimClientSocket properly
                cc->cancelWaitingForResponse();
            }
            gettimeofday(&curTime, NULL);
            //printf("SPENT TIME TO RECEIVE RAW MESSAGE = %ld\n", ((curTime.tv_sec-beginTime.tv_sec)*1000)+((curTime.tv_usec-beginTime.tv_usec)/1000));
            if (agiSimMessage.size() > 0) {
                result = processAgiSimMessage(agiSimMessage, isAdminCommand);
            } else {
                result = "Timeout waiting for AGIsim response\n";
            }
            gettimeofday(&curTime, NULL);
            //printf("SPENT TIME TO RECEIVE RESPONSE AND CONVERT IT TO NM-XML = %ld\n", ((curTime.tv_sec-beginTime.tv_sec)*1000)+((curTime.tv_usec-beginTime.tv_usec)/1000));
        } else {
            result = "ok";
        }
    } else {
        result = "SimProxy is not connected to the server\n";
    }
    //printf("SimProxy::send() returning %s\n", result.c_str());
    logger().log(opencog::Logger::DEBUG, "SimProxy::send() returning %s\n", result.c_str());
    return result;
}

bool SimProxy::sendActionCommand(const char* cmdName, float value) {
    std::string cmd_str;
    cmd_str += AGISIM_ACTION;
    cmd_str += " ";
    cmd_str += cmdName;
    cmd_str += " ";
    char floatStr[20];
    sprintf(floatStr,"%f\n",value);
    cmd_str += floatStr;
    return !strcmp(send(cmd_str, false, false).c_str(), "ok");
}

bool SimProxy::sendActionCommand(const char* cmdName, float value1, float value2) {
    std::string cmd_str;
    cmd_str += AGISIM_ACTION;
    cmd_str += " ";
    cmd_str += cmdName;
    cmd_str += " ";
    char floatStr[20];
    sprintf(floatStr,"%f ",value1);
    cmd_str += floatStr;
    sprintf(floatStr,"%f\n",value2);
    cmd_str += floatStr;
    return !strcmp(send(cmd_str, false, false).c_str(), "ok");
}

bool SimProxy::sendActionCommand(const char* cmdName, float value1, float value2, float value3) {
    std::string cmd_str;
    cmd_str += AGISIM_ACTION;
    cmd_str += " ";
    cmd_str += cmdName;
    cmd_str += " ";
    char floatStr[20];
    sprintf(floatStr,"%f ",value1);
    cmd_str += floatStr;
    sprintf(floatStr,"%f ",value2);
    cmd_str += floatStr;
    sprintf(floatStr,"%f\n",value3);
    cmd_str += floatStr;
    return !strcmp(send(cmd_str, false, false).c_str(), "ok");
}

bool SimProxy::sendActionCommand(const char* cmdName) {
    std::string cmd_str;
    cmd_str += AGISIM_ACTION;
    cmd_str += " ";
    cmd_str += cmdName;
    cmd_str += "\n";
    return !strcmp(send(cmd_str, false, false).c_str(), "ok");
}

bool SimProxy::sendActionCommand(const char* cmdName, const char* str) {
    std::string cmd_str;
    cmd_str += AGISIM_ACTION;
    cmd_str += " ";
    cmd_str += cmdName;
    cmd_str += " ";
    cmd_str += str;
    cmd_str += "\n";
    return !strcmp(send(cmd_str, false, false).c_str(), "ok");
}

bool SimProxy::sendActionCommand(const char* cmdName, const char* str, float value) {
    std::string cmd_str;
    cmd_str += AGISIM_ACTION;
    cmd_str += " ";
    cmd_str += cmdName;
    cmd_str += " ";
    cmd_str += str;
    char floatStr[20];
    sprintf(floatStr," %f ", value);
    cmd_str += floatStr;
    cmd_str += "\n";
    return !strcmp(send(cmd_str, false, false).c_str(), "ok");
}

bool SimProxy::sendActionCommand(const char* cmdName, const char* str, float value1, float value2) {
    std::string cmd_str;
    cmd_str += AGISIM_ACTION;
    cmd_str += " ";
    cmd_str += cmdName;
    cmd_str += " ";
    cmd_str += str;
    char floatStr[20];
    sprintf(floatStr," %f", value1);
    cmd_str += floatStr;
    sprintf(floatStr," %f", value2);
    cmd_str += floatStr;
    cmd_str += "\n";
    return !strcmp(send(cmd_str, false, false).c_str(), "ok");
}

// body movements

unsigned long SimProxy::turnLeft(float value) {
    if (sendActionCommand(AGISIM_TURN_LEFT, value)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_TURN_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::turnRight(float value) {
    if (sendActionCommand(AGISIM_TURN_RIGHT, value)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_TURN_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::moveForward(float value) {
    if (sendActionCommand(AGISIM_MOVE_FORWARD, value)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_MOVE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::moveGoto(std::string objName) {
    const char* argStr = objName.c_str();
    if (sendActionCommand(AGISIM_GOTO, argStr)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_MOVE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::moveBackward(float value) {
    if (sendActionCommand(AGISIM_MOVE_BACKWARD, value)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_MOVE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::strafeLeft(float value) {
    if (sendActionCommand(AGISIM_STRAFE_LEFT, value)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_MOVE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::strafeRight(float value) {
    if (sendActionCommand(AGISIM_STRAFE_RIGHT, value)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_MOVE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::walkTowards(std::string objName) {
    const char* argStr = objName.c_str();
    if (sendActionCommand(AGISIM_WALK_TOWARDS, argStr)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_MOVE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::walkTowards(float x, float z, float max_distance) {
    if (max_distance >= 0) {
        if (sendActionCommand(AGISIM_WALK_TOWARDS, x, z, max_distance)) {
            return enqueueActionInProgress(CUSTOM_SENSATION_MOVE_ANIMATION);
        }
    } else {
        if (sendActionCommand(AGISIM_WALK_TOWARDS, x, z)) {
            return enqueueActionInProgress(CUSTOM_SENSATION_MOVE_ANIMATION);
        }
    }
    return ULONG_MAX;
}

unsigned long SimProxy::nudgeTo(std::string objName, float x, float z) {
    const char* argStr = objName.c_str();
    if (sendActionCommand(AGISIM_NUDGE_TO, argStr, x, z)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_MOVE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::turnTo(std::string objName) {
    const char* argStr = objName.c_str();
    if (sendActionCommand(AGISIM_TURN_TO, argStr)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_TURN_ANIMATION);
    }
    return ULONG_MAX;
}


// eye movements

bool SimProxy::eyeUp(float value) {
  return sendActionCommand(AGISIM_EYE_UP, value);
}

bool SimProxy::eyeDown(float value) {
  return sendActionCommand(AGISIM_EYE_DOWN, value);
}

bool SimProxy::eyeLeft(float value) {
  return sendActionCommand(AGISIM_EYE_LEFT, value);
}

bool SimProxy::eyeRight(float value) {
  return sendActionCommand(AGISIM_EYE_RIGHT, value);
}

// Misc

unsigned long SimProxy::lift(std::string objName) {
    const char* argStr = objName.c_str();
    if (sendActionCommand(AGISIM_LIFT, argStr)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_SEMIPOSE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::drop() {
    if (sendActionCommand(AGISIM_DROP)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_SEMIPOSE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::throws(float distance) {
    if (sendActionCommand(AGISIM_THROW, distance)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_SEMIPOSE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::throwsAt(float x, float z) {
    if (sendActionCommand(AGISIM_THROW_AT, x, z)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_SEMIPOSE_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::kickLow() {
    if (sendActionCommand(AGISIM_KICK_LOW)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_GENERIC_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::kickHigh() {
    if (sendActionCommand(AGISIM_KICK_HIGH)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_GENERIC_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::smile() {
    if (sendActionCommand(AGISIM_SMILE)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_GENERIC_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::frown() {
    if (sendActionCommand(AGISIM_FROWN)) {
        return enqueueActionInProgress(CUSTOM_SENSATION_GENERIC_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::eat(std::string objName, float quantity) {
    if (quantity > 0) {
        if (sendActionCommand(AGISIM_EAT, objName.c_str(), quantity)) {
            return enqueueActionInProgress(CUSTOM_SENSATION_GENERIC_ANIMATION);
	}
    } else if (sendActionCommand(AGISIM_EAT, objName.c_str())) {
        return enqueueActionInProgress(CUSTOM_SENSATION_GENERIC_ANIMATION);
    }
    return ULONG_MAX;
}

unsigned long SimProxy::drink(std::string objName, float quantity) {
    if (quantity > 0) {
        if (sendActionCommand(AGISIM_DRINK, objName.c_str(), quantity)) {
            return enqueueActionInProgress(CUSTOM_SENSATION_GENERIC_ANIMATION);
        }
    } else if (sendActionCommand(AGISIM_DRINK, objName.c_str())) {
        return enqueueActionInProgress(CUSTOM_SENSATION_GENERIC_ANIMATION);
    }
    return ULONG_MAX;
}

// Communications

bool SimProxy::noiseMake() {
    return sendActionCommand(AGISIM_NOISE_MAKE);
}

// message command

unsigned long SimProxy::message(const char* msg) {

    logger().log(opencog::Logger::FINE, "SimProxy::message - init\n");
    std::string cmd_str;
    cmd_str += AGISIM_MESSAGE;
    cmd_str += " ";
    cmd_str += msg;
    cmd_str += "\n";
    //printf("SimProxy::message(%s) => %s\n", msg, cmd_str.c_str());
	if (logger().getLevel() >= opencog::Logger::FINE){
	    logger().log(opencog::Logger::FINE, "SimProxy::message(%s) => %s\n", msg, cmd_str.c_str());
	}
	std::string result = send(cmd_str, false, false);
    //printf("SimProxy::message() result = %s\n", result.c_str());
    if (!strcmp(result.c_str(), "ok")) {
#ifdef BROADCAST_MESSAGE_GOES_TO_PENDING_ACTIONS
        return enqueueActionInProgress(CUSTOM_SENSATION_BROADCAST_MESSAGE);
#endif
    }
    return ULONG_MAX;
}

// goto command

unsigned long SimProxy::gotoTarget(const char* target) {
  std::string cmd_str;
  cmd_str += "test 2 "; // TODO: Use the right command. For now, using this hack.
  cmd_str += target; // TODO: Use integer coordinates. For now, just use the string comming from target arg.
  cmd_str += "\n";
  //printf("SimProxy::gotoTarget => %s\n", cmd_str.c_str());
  send(cmd_str, false);
  return moveForward(10.0f);
}

bool SimProxy::getCurrentSense() {
  std::string cmd_str;
  cmd_str += AGISIM_SENSE;
  cmd_str += "\n";
  return !strcmp(send(cmd_str, false, false).c_str(), "ok");
}


// ADMIN COMMANDS

std::string SimProxy::newAgent(float x, float y, float z,  const char* objectType, const char*  meshType, float radius, const char* agentBaseName) {
    std::string cmd_str;
    cmd_str += AGISIM_NEW;
    char floatStr[20];
    sprintf(floatStr," %f ",x);
    cmd_str += floatStr;
    sprintf(floatStr,"%f ",y);
    cmd_str += floatStr;
    sprintf(floatStr,"%f ",z);
    cmd_str += floatStr;
    cmd_str += " ";
    cmd_str += objectType;
    cmd_str += " ";
    cmd_str += meshType;
    cmd_str += " ";
    cmd_str += agentBaseName;
    cmd_str += " agent "; // special init_string parameter
#if 1
    sprintf(floatStr,"%f ",radius);
    cmd_str += floatStr;
#else // this is for AGISim only (not AGISimSIm)
    cmd_str += AGISIM_AGENT;
#endif
    cmd_str += "\n";
    std::string answer = send(cmd_str, false);
    //printf("SimProxy::newAgent() got answer: '%s'\n", answer.c_str());
    logger().log(opencog::Logger::DEBUG, "SimProxy::newAgent() got answer: '%s'\n", answer.c_str());
    // Gets the new agent's name. For doing this, answer must be in this format:
    //<agisim> <msg> answer </msg> </agisim>
    // where answer is the new agent's name
    //<agisim> <error> answer </error> </agisim>
    // where answer is the error message
    if (strstr(answer.c_str(), agentBaseName) == NULL) {
        std::cerr << "Error creating new agent in AgiSim: " << answer << std::endl;
        //assert(false);
        answer = "";
    } else {
        // Set attribute agentName
        char buf[256];
        const char* p = answer.c_str();
        int i = 0;
        while (*p != '\0' && i < 255) {
            if (*p != ' ') buf[i++] = *p;
            p++;
        }
        buf[i] = '\0';
        agentName = buf;
        std::cout << "SimProxy::newAgent(): GOT AGENT NAME = '" << agentName << "'" << std::endl;
        answer = agentName;
    }
    return answer;
}

std::string SimProxy::resetCurrentWorld() {
  std::string cmd_str;
  cmd_str += AGISIM_RESET;
  cmd_str += " soft\n";
  return send(cmd_str, true, true);
}

std::string SimProxy::disableAgiSimClock() {
  std::string cmd_str;
  cmd_str += AGISIM_CONFIG;
  cmd_str += " ";
  cmd_str += AGISIM_FRAME_UPDATE_DELAY;
  cmd_str += "=2000000000\n";
  return send(cmd_str, true, true);
}


// Special commands

std::string SimProxy::getPos(std::string objName) {
  std::string cmd_str;
  cmd_str += "getpos ";
  cmd_str += objName;
  cmd_str += "\n";
  //printf("SimProxy::getPos => %s\n", cmd_str.c_str());
  return send(cmd_str, false);
}

std::string SimProxy::setPos(std::string objName, float x, float y, float z) {
  std::string cmd_str;
  cmd_str += "setpos ";
  cmd_str += objName;
  cmd_str += " ";
  char floatStr[20];
  sprintf(floatStr,"%f ",x);
  cmd_str += floatStr;
  sprintf(floatStr,"%f ",y);
  cmd_str += floatStr;
  sprintf(floatStr,"%f\n",z);
  cmd_str += floatStr;
  //printf("SimProxy::setPos => %s\n", cmd_str.c_str());
  return send(cmd_str, false);
}

std::string SimProxy::getRot(std::string objName) {
  std::string cmd_str;
  cmd_str += "getrot ";
  cmd_str += objName;
  cmd_str += "\n";
  //printf("SimProxy::getRot => %s\n", cmd_str.c_str());
  return send(cmd_str, false);
}

std::string SimProxy::setRot(std::string objName, float x, float y, float z) {
  std::string cmd_str;
  cmd_str += "setrot ";
  cmd_str += objName;
  cmd_str += " ";
  char floatStr[20];
  sprintf(floatStr,"%f ",x);
  cmd_str += floatStr;
  sprintf(floatStr,"%f ",y);
  cmd_str += floatStr;
  sprintf(floatStr,"%f\n",z);
  cmd_str += floatStr;
  //printf("SimProxy::setRot => %s\n", cmd_str.c_str());
  return send(cmd_str, false);
}


// IMPLEMENTATION OF ABSTRACT METHODS

void SimProxy::receiveAsynchronousMessage(const std::string& agiSimMessage) {

	if (logger().getLevel() >= opencog::Logger::FINE)
	    logger().log(opencog::Logger::FINE, "SimProxy::receiveAsynchronousMessage: %s", agiSimMessage.c_str());
    processAgiSimMessage(agiSimMessage, false);
}


// IMPLEMENTATION OF METHODS OF AUXILIAR CLASSES

// Mock for perception and status handler

DefaultPerceptionAndStatusHandler::~DefaultPerceptionAndStatusHandler() {
}

void DefaultPerceptionAndStatusHandler::mapInfo(std::vector<ObjMapInfo>& objects) {

	if (logger().getLevel() >= opencog::Logger::FINE) {
            foreach(ObjMapInfo obj, objects) {
	        logger().log(opencog::Logger::FINE, "=========================================\n");
	        logger().log(opencog::Logger::FINE, "object: %s, type: %s%s\n", obj.name.c_str(), obj.type.c_str(), obj.removed?" => REMOVED!":"");
	        logger().log(opencog::Logger::FINE, "Position: posX = %lf, posY = %lf, posZ = %lf\n", obj.posX, obj.posY, obj.posZ);
	        logger().log(opencog::Logger::FINE, "Position: rotX = %lf, rotY = %lf, rotZ = %lf\n", obj.rotX, obj.rotY, obj.rotZ);
	        logger().log(opencog::Logger::FINE, "Size: length = %lf, width = %lf, height = %lf\n", obj.length, obj.width, obj.height);
	        logger().log(opencog::Logger::FINE, "Edible: %d, Drinkable: %d\n\n", obj.edible, obj.drinkable);
	        logger().log(opencog::Logger::FINE, "PetHome: %d, FoodBowl: %d, WaterBowl: %d\n\n", obj.petHome, obj.foodBowl, obj.waterBowl);
	    }
	}
}

void DefaultPerceptionAndStatusHandler::actionStatus(unsigned long actionTicket, bool success) {
    logger().log(opencog::Logger::DEBUG, "actionStatus: actionTicket = %ld, success = %d\n", actionTicket, success);
}

void DefaultPerceptionAndStatusHandler::errorNotification(const std::string& errorMsg) {
    logger().log(opencog::Logger::DEBUG, "ERROR: errorMsg = %s\n", errorMsg.c_str());
}

