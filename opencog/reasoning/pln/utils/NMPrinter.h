/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#ifndef _NMPRINTER_H_
#define _NMPRINTER_H_

#include <sstream>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>

#include <boost/variant.hpp>

#include <opencog/atomspace/types.h>
#include "../PLNUtils.h"

//#include "PLNatom.h"

#define NM_PRINTER_DEFAULT_LOG_LEVEL 0
#define NM_PRINTER_DEFAULT_TRUTH_VALUE_PRECISION 4
#define NM_PRINTER_KEEP_DEFAULT_LOG_LEVEL -100
#define NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE 3
#define NM_PRINTER_DEFAULT_INITIAL_INDENTATION 0

#define NMP_TYPE_NAME                  (1UL << 0)
#define NMP_TYPE_ID                    (1UL << 1) 
#define NMP_NODE_NAME                  (1UL << 2) 
#define NMP_NODE_TYPE_NAME             (1UL << 3) 
#define NMP_TRUTH_VALUE                (1UL << 4) 
#define NMP_HANDLE                     (1UL << 5) 
#define NMP_ATOM_ADDRESS               (1UL << 6) 
#define NMP_ARITY                      (1UL << 7) 
#define NMP_BRACKETED                  (1UL << 8) 
#define NMP_PRINT_TO_FILE              (1UL << 9) 
#define NMP_NO_UNARY_LIST_LINKS        (1UL << 10) 
#define NMP_NO_TV_WITH_NO_CONFIDENCE   (1UL << 11) 

#define NMP_DEFAULT         (NMP_TYPE_NAME | NMP_NODE_TYPE_NAME | NMP_NODE_NAME | NMP_TRUTH_VALUE) 
#define NMP_ABBREVIATED     (NMP_TYPE_NAME | NMP_NODE_NAME | NMP_BRACKETED) 
#define NMP_ALL             (NMP_TYPE_NAME | NMP_TYPE_ID | NMP_NODE_TYPE_NAME | NMP_NODE_NAME | NMP_TRUTH_VALUE | NMP_HANDLE | NMP_ARITY | NMP_PRINT_TO_FILE) 

#define NM_PRINTER_LOG_FILE_NAME "nm_printer.log"

using namespace opencog;
using namespace opencog::pln;

typedef boost::variant < pHandle, vtree, vtree::iterator_base > NMPrintable;

/**
 * This class provides methods for printing a data structure that represents an atom or 
 * converting it into a string. See the NMPrinterUTest.cxxtest unit test file under 
 * novamente/test/common/core directory to have examples of how to use this class.
 */
class NMPrinter
{
public:

    /**
     * Creates a new NMPrinter with print options according with the given arguments and/or
     * default options (since all arguments are optional).
     * 
     * @param printOptions  A bit-mask argument for selecting the ON/OFF options of a printer.
     *    The available options are: 
     *      - NMP_TYPE_NAME                : prints the name of the type of an atom (for Links only)
     *      - NMP_TYPE_ID                  : prints the id of the type of an atom (for any type of atom)
     *      - NMP_NODE_NAME                : prints the name of an atom, if it exists (i.e., for Nodes only)
     *      - NMP_NODE_TYPE_NAME           : prints the name of the type of a Node
     *      - NMP_TRUTH_VALUE              : prints the truth value of an atom
     *      - NMP_HANDLE                   : prints the handle of an atom (as an integer value)
     *      - NMP_ATOM_ADDRESS             : prints the address of an atom object
     *      - NMP_ARITY                    : prints the arity of an atom (for Links only)
     *      - NMP_BRACKETED                : uses brackets instead of indentation for printing the tree.
     *      - NMP_PRINT_TO_FILE            : prints also to a log file (see NMP_PRINT_TO_FILE)
     *      - NMP_NO_UNARY_LIST_LINKS      : do not print any unary ListLink
     *      - NMP_NO_TV_WITH_NO_CONFIDENCE : do not print any TruthValue with confidence equalt to zero
     *      
     *    There are also some additional constants that combines a subset of these options, for convinience:
     *      - NMP_DEFAULT       :  NMP_TYPE_NAME | NMP_NODE_TYPE_NAME | NMP_NODE_NAME | NMP_TRUTH_VALUE
     *      - NMP_ABBREVIATED   :  NMP_TYPE_NAME | NMP_NODE_NAME | NMP_BRACKETED 
     *      - NMP_ALL           :  NMP_TYPE_NAME | NMP_TYPE_ID | NMP_NODE_TYPE_NAME | NMP_NODE_NAME | NMP_TRUTH_VALUE | NMP_HANDLE | NMP_ARITY | NMP_PRINT_TO_FILE
     * 
     * 
     *    So, some examples of how to construct a NMPrinter object are: 
     *      - NMPrinter() => Uses the default configuration. It has the same effect as NMPrinter(NMP_DEFAULT)
     *      - NMPrinter(NMP_ALL) => Uses all options defined by NMP_ALL as descrived above.
     *      - NMPrinter(NMP_ALL & ~NMP_PRINT_TO_FILE) => Uses all options defined by NMP_ALL, except for the NMP_PRINT_TO_FILE option.
     * 
     * @param tvPrecision   The precision used to print truth value float attributes. Default is NM_PRINTER_DEFAULT_TRUTH_VALUE_PRECISION
     * @param indentationTabSize    The size of an indentation tab. Default is NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE 
     * @param defaultLogLevel   The default log level to be used when calling print(...) or toString(...) methods without specify the log level argument.
     * @param initialIdentation The initial number of "tabs" to be used by print(...) and toString(...) methods.
     * 
     */
    NMPrinter(unsigned long _printOptions = NMP_DEFAULT, 
             int _tvPrecision = NM_PRINTER_DEFAULT_TRUTH_VALUE_PRECISION, 
             int _indentationTabSize = NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE, 
             int _defaultLogLevel = NM_PRINTER_DEFAULT_LOG_LEVEL, 
             int _initialIndentation = NM_PRINTER_DEFAULT_INITIAL_INDENTATION);

    virtual ~NMPrinter();
    
    /**
     * Prints a NMPPrintable object (Handle, vtree, etc) according with the NMPrinter options defined 
     * in its constructor. Optionally, a specific log level (for only this call) may be specified in the second argument. 
     */
    void print(NMPrintable printable, int logLevel = NM_PRINTER_KEEP_DEFAULT_LOG_LEVEL) const;
    /**
     * Converts a NMPPrintable object (Handle, vtree, etc) into a std::string object, according with the NMPrinter options defined 
     * in its constructor. Optionally, a specific log level (for only this call) may be specified in the second argument. 
     */
    std::string toString(NMPrintable printable, int logLevel = NM_PRINTER_KEEP_DEFAULT_LOG_LEVEL);

    /**
     * This static method was created for testing purposes (Unit tests, actually)
     * It should not be called in normal usage because it makes NMPrinter objects that 
     * prints to file to crash if used after this method is called.
     * Besides, if there is no NMPrinter object using the log file, sudh file is already 
     * closed automatically. So, one doesnt need to call this method unless there is a 
     * very specific situation (i.e.,  to read the log file contents with some NMPrinter 
     * objects that uses the NMP_PRINT_TO_FILE still in memory)
     */
    static void closeLogFile();

    /**
     * Defines () operator in order to allow statements like that:
     *  for_each(mySTLContainer.begin(), mySTLContainer.end(), NMPrinter());
     */
    void operator()(NMPrintable arg, int logLevel = NM_PRINTER_KEEP_DEFAULT_LOG_LEVEL) const;
    
private:

    static FILE* logFile;
    static int numberOfPrintersUsingLogFile;
    
    unsigned long printOptions; 
    int tvPrecision;
    int indentationTabSize;
    int defaultLogLevel;
    int initialIndentation;
    
    bool printToFile;
    
    bool areFromSameType(NMPrintable, NMPrintable) const;
    bool isHandle(NMPrintable) const;
    bool isVtree(NMPrintable) const;
    bool isVtreeIterator(NMPrintable p) const;
    
    void toStream(std::ostream& out, const NMPrintable p, int indentationLevel) const;
    void printHandle(std::ostream& out, pHandle h, int indentationLevel) const;
    void printVTree(std::ostream& out, vtree::iterator top, int indentationLevel) const;
    bool logLevelOk(int logLevel) const;
    void printSpaces(std::ostream& out, int indentationLevel) const;
    std::string getFloatStr(float value) const;
    std::string getFloatStrWithoutTrailingZeros(float value) const;
};

/**
 * Boost visitor for checking real types inside NMPrintable variant.
 */
class are_from_same_type_visitor
: public boost::static_visitor<bool>
{
public:

    template <typename T, typename U>
    bool operator()( const T &, const U & ) const
    {
        return false; // different types
    }

    template <typename T>
    bool operator()( const T & lhs, const T & rhs ) const
    {
        return true; // same types
    }
};

#endif //_NMPRINTER_H_
