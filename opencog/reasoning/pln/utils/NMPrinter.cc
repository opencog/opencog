#include "NMPrinter.h"

#include <ClassServer.h>
#include <AtomSpace.h>
#include <CogServer.h>

FILE* NMPrinter::logFile = NULL;
int NMPrinter::numberOfPrintersUsingLogFile = 0;

#define CLOSE_LOG_FILE_AUTOMATICALLY

NMPrinter::NMPrinter(unsigned long _printOptions, int _tvPrecision, int _indentationTabSize, int _defaultLogLevel, int _initialIndentation)
{
    //printf("NMPrinter::NMPrinter()\n");
    printOptions = _printOptions;
    tvPrecision = _tvPrecision;
    indentationTabSize = _indentationTabSize;
    defaultLogLevel = _defaultLogLevel;
    initialIndentation = _initialIndentation;
    printToFile = printOptions & NMP_PRINT_TO_FILE;
    if (printToFile && numberOfPrintersUsingLogFile < 1) {
        // If it was opened before (i.e., logFile != NULL), open for appending only
        // Otherwise, creates a new file or truncate its size to zero length.
#ifndef CLOSE_LOG_FILE_AUTOMATICALLY
        if (logFile == NULL)
#endif        
        logFile = fopen(NM_PRINTER_LOG_FILE_NAME, (logFile?"at":"wt"));
        if (logFile == NULL) {
            printf("ERROR: Could not open log file '%s' for writing/appending\n", NM_PRINTER_LOG_FILE_NAME);
            printToFile = false;
        }
    }
#ifdef CLOSE_LOG_FILE_AUTOMATICALLY
    if (printToFile) {
        //printf("Increasing numberOfPrintersUsingLogFile from %d", numberOfPrintersUsingLogFile);
        numberOfPrintersUsingLogFile++;
        //printf(" to %d\n", numberOfPrintersUsingLogFile);
    }
#endif    
}

NMPrinter::~NMPrinter()
{
//    printf("NMPrinter::~NMPrinter()\n");
#ifdef CLOSE_LOG_FILE_AUTOMATICALLY
    if (printToFile && numberOfPrintersUsingLogFile > 0) {

        //printf("Decreasing numberOfPrintersUsingLogFile from %d", numberOfPrintersUsingLogFile);

        numberOfPrintersUsingLogFile--;
        //printf(" to %d\n", numberOfPrintersUsingLogFile);
        if (numberOfPrintersUsingLogFile ==0) {
            fclose(logFile);
        }
    }
#endif        
}

void NMPrinter::closeLogFile() {
    //printf("NMPrinter::closeLogFile()\n");
#ifdef CLOSE_LOG_FILE_AUTOMATICALLY    
    if (numberOfPrintersUsingLogFile && logFile != NULL) {
        fclose(logFile);
    }
#else        
    if (logFile != NULL) {
        fclose(logFile);
        logFile = NULL;
    }
#endif        
}

void NMPrinter::operator()(NMPrintable arg, int logLevel) const {
    print(arg, logLevel);
}


bool NMPrinter::logLevelOk(int logLevel) const {
    bool result;
    if (logLevel == NM_PRINTER_KEEP_DEFAULT_LOG_LEVEL) {
        logLevel = defaultLogLevel;
    }
    result = (logLevel <= currentDebugLevel); 
//    if (!result)  printf("NMPrinter::print(): logLevel not enough: %d, currentDebugLevel: %d\n", logLevel, currentDebugLevel);
    return result;
}

void NMPrinter::print(NMPrintable p, int logLevel) const {
    //printf("NMPrinter::print()\n");
    if (logLevelOk(logLevel)) {
        toStream(cout, p, initialIndentation);
    }
}

string NMPrinter::toString(NMPrintable p, int logLevel) {
    //printf("NMPrinter::toString()\n");
    stringstream ss;
    if (logLevelOk(logLevel)) {
        toStream(ss, p, initialIndentation);
    }
    return ss.str();
}

void NMPrinter::toStream(ostream& out, const NMPrintable p, int indentationLevel) const{
    //printf("NMPrinter::toStream()\n");
    if (isHandle(p)) {
        //printf("PRINTING HANDLE:\n");
        Handle h = boost::get<Handle>(p);
        printHandle(out, h, indentationLevel);
        if (printOptions & NMP_BRACKETED) {
            out << endl;
            if (printToFile) fprintf(logFile, "\n");
        }
    } else if (isVtree(p)) {
        //printf("PRINTING VTREE:\n");
        vtree vt = boost::get<vtree>(p);
        printVTree(out, vt.begin(), indentationLevel);
        if (printOptions & NMP_BRACKETED) {
            out << endl;
            if (printToFile) fprintf(logFile, "\n");
        }
    } else if (isVtreeIterator(p)) {
        //printf("PRINTING VTREE from Iterator:\n");
        printVTree(out, boost::get<vtree::iterator_base>(p), indentationLevel);
        if (printOptions & NMP_BRACKETED) {
            out << endl;
            if (printToFile) fprintf(logFile, "\n");
        }
    } else {
        printf("NMPrinter::toStream(): Got an invalid NMPrintable!\n");
    }
}

void NMPrinter::printSpaces(ostream& out, int indentationLevel) const{
    //printf("NMPrinter::printSpaces()\n");
    for (int i=0; i < indentationLevel; i++) {
        for (int j=0; j < indentationTabSize; j++) {
            out << " ";
            if (printToFile) fprintf(logFile, " ");
        }
    }
}

std::string NMPrinter::getFloatStr(float value) const {
    char str[256];
    switch(tvPrecision) {
        case 1:
            sprintf(str, "%.1f", value);
            break;
        case 2:
            sprintf(str, "%.2f", value);
            break;
        case 3:
            sprintf(str, "%.3f", value);
            break;
        case 4:
            sprintf(str, "%.4f", value);
            break;
        case 5:
            sprintf(str, "%.5f", value);
            break;
        case 6:
            sprintf(str, "%.6f", value);
            break;
        default:
            sprintf(str, "%f", value);
            break;
    }
    return str;
}

std::string NMPrinter::getFloatStrWithoutTrailingZeros(float value) const
{
  std::string result = getFloatStr(value);
  char* str = strdup(result.c_str());

  while (strlen(str)>2 && str[strlen(str)-1] == '0')
    str[strlen(str)-1] = NULL;
  if (strlen(str) == 2) // 1,
    str[1] = NULL;

  result = str;
  free(str);

  return result;
}

void NMPrinter::printHandle(ostream& out, Handle h, int indentationLevel) const{
    //printf("NMPrinter::printHandle()\n");
//    printf("NMPrinter::printHandle(%p): printOptions = %X\n", h, printOptions);
    AtomSpace *nm = CogServer::getAtomSpace();
    
    bool isNode = nm->isNode(h); 
    Type type = nm->getType(h);
    if ((printOptions & NMP_NO_UNARY_LIST_LINKS) && (type == LIST_LINK) && (nm->getArity(h) == 1)) {
        Handle newH = nm->getOutgoing(h, 0);
        printHandle(out, newH, indentationLevel);
    } else {
        if (!(printOptions & NMP_BRACKETED)) printSpaces(out, indentationLevel);
        if (isNode) {
            char* nodeName = strdup(nm->getName(h).c_str());
            if (printOptions & NMP_NODE_NAME) { 
                out << "\"" << nodeName << "\"";
                if (printToFile) fprintf(logFile, "\"%s\"", nodeName);
            }
            free(nodeName);
            if ((printOptions & NMP_NODE_NAME) && (printOptions & NMP_NODE_TYPE_NAME)) { 
                out << ":"; 
                if (printToFile) fprintf(logFile, ":");
            }
            if (printOptions & NMP_NODE_TYPE_NAME) {
                const char* typeStr = (*ClassServer::getClassName())[type].c_str();
                out << typeStr;
                if (printToFile) fprintf(logFile, "%s", typeStr);
            }
        } else {
            if (printOptions & NMP_TYPE_NAME) {
                const char* typeStr = (*ClassServer::getClassName())[type].c_str();
                out << typeStr;
                if (printToFile) fprintf(logFile, "%s", typeStr);
            }
        }        
        if (printOptions & NMP_TYPE_ID) {
            out << "[typeId=" << type << "]"; 
            if (printToFile) fprintf(logFile, "[typeId=%d]", type);
        }
        if (printOptions & NMP_ATOM_ADDRESS) {
            out << "[address=" << ((void*) h) << "]"; 
            if (printToFile) fprintf(logFile, "[address=%p]", (void*) h);
        }
        if (printOptions & NMP_TRUTH_VALUE) {
            const TruthValue& tv = nm->getTV(h);
            if (tv.isNullTv()) {
                if (!(printOptions & NMP_NO_TV_WITH_NO_CONFIDENCE)) {
                    out << " <NULL TV>";
                    if (printToFile) fprintf(logFile, " <NULL TV>");
                }
            } else {
                //printf("tv->getConfidence() = %14.12f\n", tv->getConfidence());
//                if (!(printOptions & NMP_NO_TV_WITH_NO_CONFIDENCE) || tv->getConfidence() > 0.0000001) {
                if (!(printOptions & NMP_NO_TV_WITH_NO_CONFIDENCE) || tv.getConfidence() > 0) {
                    std::string str = getFloatStrWithoutTrailingZeros(tv.getMean());
                    out << " <" + str + ",";
                    if (printToFile) fprintf(logFile, " <%s,", str.c_str());
                    str = getFloatStrWithoutTrailingZeros(tv.getConfidence());
                    out << str + ">"; 
                    if (printToFile) fprintf(logFile, "%s>", str.c_str());
                }
            }
        }
        if (printOptions & NMP_HANDLE) {
            char str[100];
            sprintf(str, "%lu", (long) h);
            out << " [" << str << "]"; 
            if (printToFile) fprintf(logFile, " [%s]", str);
        }
        if (!(printOptions & NMP_BRACKETED)) {
            out << endl;
            if (printToFile) fprintf(logFile, "\n");
        }
        if (!isNode) {
            if (printOptions & NMP_BRACKETED) {
                out << "(";
                if (printToFile) fprintf(logFile, "(");
            }
            int arity = nm->getArity(h);
            for (int i = 0; i < arity; i++) {
                if (i > 0 && (printOptions & NMP_BRACKETED)) {
                    out << ",";
                    if (printToFile) fprintf(logFile, ",");
                }
                Handle newH = nm->getOutgoing(h, i);
                printHandle(out, newH, indentationLevel+1);
            }
            if (printOptions & NMP_BRACKETED) {
                out << ")";
                if (printToFile) fprintf(logFile, ")");
            }
        }
    }
}

void NMPrinter::printVTree(ostream& out, vtree::iterator top, int indentationLevel) const{
    //printf("NMPrinter::printVTree()\n");

    Handle h = boost::get<Handle>(*top);
    Handle undefined = TLB::UndefinedHandle();
    if (CoreUtils::handleCompare(&h,&undefined) == 0)
    { 
        out << "null" << endl;
        if (printToFile) fprintf(logFile, "null\n");
        return;
    }

    if (CogServer::getAtomSpace()->isReal(h))
    {
        printHandle(out, h, indentationLevel);
    } else {
        Type type = (Type)((int)h);

        if ((printOptions & NMP_NO_UNARY_LIST_LINKS) && (type == LIST_LINK) && (top.number_of_children() == 1)) {
            vtree::sibling_iterator c=top.begin();
            printVTree(out, c, indentationLevel);
        } else {
            if (!(printOptions & NMP_BRACKETED)) printSpaces(out, indentationLevel);
    
            if (printOptions & NMP_TYPE_NAME) {
                const char* typeStr = (*ClassServer::getClassName())[type].c_str();
                out << typeStr;
                if (printToFile) fprintf(logFile, "%s", typeStr);
            }
            if (printOptions & NMP_TYPE_ID) {
                out << "[typeId=" << type << "]"; 
                if (printToFile) fprintf(logFile, "[typeId=%d]", type);
            }
            if (printOptions & NMP_ARITY) {
                int arity = top.number_of_children();
                out << "{" << arity << "}"; 
                if (printToFile) fprintf(logFile, "{%d}", arity);
            }
            if (!(printOptions & NMP_BRACKETED)) {
                out << endl;
                if (printToFile) fprintf(logFile, "\n");
            }
    
            if (printOptions & NMP_BRACKETED) {
                out << "(";
                if (printToFile) fprintf(logFile, "(");
            }
            for (vtree::sibling_iterator c=top.begin(); c!=top.end();c++) {
                if (c!=top.begin() && (printOptions & NMP_BRACKETED)) {
                    out << ",";
                    if (printToFile) fprintf(logFile, ",");
                }
                printVTree(out, c, indentationLevel+1);
            }
            if (printOptions & NMP_BRACKETED) {
                out << ")";
                if (printToFile) fprintf(logFile, ")");
            }
        }
    }

}

bool NMPrinter::areFromSameType(NMPrintable p1, NMPrintable p2) const{
    bool result = boost::apply_visitor(are_from_same_type_visitor(), p1, p2);
    return result;
}

NMPrintable HANDLE_NM_PRINTABLE_EXAMPLE = (Handle) NULL;
NMPrintable VTREE_NM_PRINTABLE_EXAMPLE = mva((Handle) NULL);
NMPrintable VTREE_ITERATOR_NM_PRINTABLE_EXAMPLE = vtree::iterator_base();

bool NMPrinter::isHandle(NMPrintable p) const {
    bool result = areFromSameType(p,HANDLE_NM_PRINTABLE_EXAMPLE);
    return result;
}

bool NMPrinter::isVtree(NMPrintable p) const {
    bool result = areFromSameType(p,VTREE_NM_PRINTABLE_EXAMPLE);
    return result;
}

bool NMPrinter::isVtreeIterator(NMPrintable p) const {
    bool result = areFromSameType(p,VTREE_ITERATOR_NM_PRINTABLE_EXAMPLE);
    return result;
}

