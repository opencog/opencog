/** 
 * StringManipulator.h
 * 
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#ifndef STRINGMANIPULATOR_H_
#define STRINGMANIPULATOR_H_

#include <string>

namespace Util{

class StringManipulator {
	
	public:
	    
        /**
         * Convert a string to upper case
         *
         * @param str The string to be converted in upper cases
         * @return The string converted 
         */
        static std::string toUpper(const std::string& str);

        /**
         * Convert a string to lower case
         *
         * @param str The string to be converted in lower cases
         * @return The string converted 
         */
        static std::string toLower(const std::string& str);

        /**
         * Clean the string
         */
	    static std::string clean(const std::string&str);	

}; // class	
}  // namespace

#endif /*STRINGBUFFER_H_*/
