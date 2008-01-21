/** 
 * StringTokenizer.cc
 * 
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#include <cctype>
#include "foreach.h"
#include "StringManipulator.h"
using namespace Util;

#define isvalidtoken(ch) ((ch) == ')' || (ch)== '(' || (ch) == ',' )	

std::string StringManipulator::toUpper(const std::string& str){
    std::string result;

    foreach(char _c, str){
        result.push_back(toupper(_c));
    }

    return result;
}

std::string StringManipulator::toLower(const std::string& str){
    std::string result;

    foreach(char _c, str){
        result.push_back(tolower(_c));
    }

    return result;
}

std::string StringManipulator::clean(const std::string& str){
    std::string result;
    bool started = false;

    foreach(char _c, str){
        if (isalpha(_c) || isdigit(_c) ||
            isvalidtoken(_c) ||  _c == '_' || _c == char(39) ||
            (started && _c == ' ')){
            
            result.push_back((_c > 'Z' && isalpha(_c)) ? (_c - ('z'-'Z')) : _c);
            started = true;
        }
    }

    return result;
}

