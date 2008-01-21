/** 
 * StringTokenizer.cc
 * 
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#include "StringTokenizer.h"
#include "exceptions.h"
#include <cstring>
using namespace Util;


StringTokenizer::StringTokenizer(){
    reset();
    
    str.assign("");
    delimiter.assign("");
}

StringTokenizer::StringTokenizer(const std::string &str, const std::string &delimiter){
    reset();
    
    this->str = str;
    this->delimiter = delimiter;
}

StringTokenizer::~StringTokenizer(){
}
        
std::string StringTokenizer::getString(){
    return str;
}

void StringTokenizer::setString(const std::string &str){
    this->str = str;
}
        
const std::string & StringTokenizer::getDelimiter(){
    return delimiter;
}

void StringTokenizer::setDelimiter(const std::string &str){
    this->delimiter = delimiter;
} 

void StringTokenizer::reset(){
    start = 0;
    end = 0;
}
        
const std::string StringTokenizer::nextToken() {
    cassert(TRACE_INFO, str != "", "StringTokenizer - string should not be empty.");
    cassert(TRACE_INFO, delimiter != "", "StringTokenized - delimiter should not be empty.");
    
    // end of the string
    if(end == str.size()){
        return "";
    }
    
    if(start ==  0 && end == 0){
        end = str.find(delimiter);
        if ( end == std::string::npos ) {
            end = str.size();
        }
        return str.substr(start, end - start);
    }

    do{
        start = end + delimiterSize();
        if (start == str.size()) {
            end = start;
            return "";
        }
        end = str.find(delimiter, start);
        if(end == std::string::npos){ end = str.size(); } 
    } while( str.substr(start, end - start) == delimiter || end == start);
    
    return str.substr(start, end - start);
}

unsigned int StringTokenizer::delimiterSize(){
    return delimiter.size();
} 
