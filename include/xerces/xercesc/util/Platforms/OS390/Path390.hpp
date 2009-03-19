/*
 * Copyright 2002,2004 The Apache Software Foundation.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * $Id: Path390.hpp 176026 2004-09-08 13:57:07Z peiyongz $
 */

#ifndef PATH390_HPP
#define PATH390_HPP

#include <xercesc/util/XercesDefs.hpp>

XERCES_CPP_NAMESPACE_BEGIN

class  Path390
{
 public:
    // Constructors and Destructor
    Path390();
    ~Path390();
    Path390(char *s);

    // Set a new path in the object. This will overlay any existing path and
    // re-initialize the object.
    void setPath(char *s);

    // This performs a complete parse of the path. It returns the error code or 0
    // if there was no error.
    int  fullParse();

    // This returns the path in a format as required by fopen
    char * getfopenPath();

    // This returns the parameters in a format as required by fopen
    char * getfopenParms();

    // This returns the type of the path. See the constants defined below.
    int  getPathType();

    // This returns the error code that was found during the parse
    int  getError();

    // Returns whether the path is relative or absolute
    bool isRelative();

    // Returns whether or not type=record shows up in the path.
    bool isRecordType();

 private:
    int _pathtype;
    char * _orgpath;
    int _orglen;
    char * _resultpath;
    bool _absolute;
    bool _uriabsolute;
    bool _dsnabsolute;
    char * _curpos;
    int _parsestate;
    int _numperiods;
    int _numsemicolons;

    int _error;
    char * _orgparms;
    int _orgparmlen;

    char * _lastsemi;
    char * _lastslash;
    char * _lastparen;
    char * _parmStart;
    char * _pathEnd;
    char * _extStart;

    int _typerecord;
    // internal only methods:
    void _determine_uri_abs();
    void _determine_type();
    void _determine_punct();
    void _determine_parms();
    void _parse_rest();

};

// Internal constants for the _parsestate variable:
#define PARSE_NONE         0
#define PARSE_ABSOLUTE_URI 1
#define PARSE_PATHTYPE     2
#define PARSE_PUNCT        3
#define PARSE_PARMS        4
#define PARSE_PARSED       5

// These are the possible error return codes:
#define NO_ERROR                        0
#define ERROR_SEMICOLON_NOT_ALLOWED    101
#define ERROR_PERIOD_NOT_ALLOWED       102
#define ERROR_NO_PAREN_ALLOWED         103
#define ERROR_ABS_PATH_REQUIRED        104
#define ERROR_NO_EXTRA_PERIODS_ALLOWED 105
#define ERROR_MUST_BE_ABSOLUTE         106
#define ERROR_BAD_DD                   107
#define ERROR_BAD_DSN2                 108
#define ERROR_NO_EXTRA_SEMIS_ALLOWED   109

// Constants for the _pathtype variable and the return value from getPathType() method:
#define PATH390_HFS      1
#define PATH390_DSN1     2 // format is dsn:/chrisl/data/xml/member1.
#define PATH390_DSN2     3 // format is dsn://'chrisl.data.xml(member1)'
#define PATH390_DD       4
#define PATH390_OTHER    5

XERCES_CPP_NAMESPACE_END

#endif
