/** 
 * StringTokenizer.h
 * 
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#ifndef STRINGBUFFER_H_
#define STRINGBUFFER_H_

#include <string>

namespace Util{

class StringTokenizer {
	
	public:
		
		/**
		 * Constructor and destructor
		 */ 
		StringTokenizer();
		StringTokenizer(const std::string &str, const std::string &delimiter);
		~StringTokenizer();
		
		/**
		 * Getter and setter for the string to be parsed, that is, to extract 
		 * its tokens.  
		 */
		std::string getString();
		void setString(const std::string &str);
		
		/**
		 * Getter and setter for the delimiter to the tokens.
		 */
		const std::string & getDelimiter();
		void setDelimiter(const std::string &str);  
		
		/**
		 * Return the next token from the string. If the end of the string is 
		 * reached the method retuns a empty string "" 
		 */
		const std::string nextToken();
		
		/**
		 * Reset the position pointers to init position.
		 */
		void reset(); 
	
	private:
		std::string str;
		std::string delimiter;
		
		// start/end position pointers
		unsigned int start;
		unsigned int end;
		
		/**
		 * Inform the delimiter size
		 */
		unsigned int delimiterSize();
		 
}; // class	
}  // namespace

#endif /*STRINGBUFFER_H_*/
