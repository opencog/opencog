/** 
 * FileMessageCentral.h
 * 
 * Author: Elvys Borges
 * Copyright(c), 2007
 */
#ifndef FILEMESSAGECENTRAL_H_
#define FILEMESSAGECENTRAL_H_

#include "MessageCentral.h"

#include "boost/filesystem.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include <SystemParameters.h>
#include <LADSUtil/exceptions.h>
#include "StringMessage.h"

#include <fstream>
#include <iostream>
#include <exception>

namespace MessagingSystem{

using namespace boost::filesystem;


/**
 * Implements MessageCentral using a map of queue, in memory.
 *
 */
class FileMessageCentral : public MessageCentral {
	
	private:
	
        const Control::SystemParameters& parameters;
        path directory;
		
	public:
	
		/**
		 * Constructors and destructor
		 */
		~FileMessageCentral();

		FileMessageCentral(const Control::SystemParameters &params) throw (RuntimeException, std::bad_exception);
		
		void createQueue(const std::string id, const bool reset = false);
	
        void clearQueue(const std::string id);
        
        void removeQueue(const std::string id);

		const bool isQueueEmpty(const std::string id);		

		const int sizeQueue(const std::string id);		

		const bool existsQueue(const std::string id);		

		void push(const std::string id, Message *message);
		
		Message* pop(const std::string id);
		
}; // class
}  // namespace

#endif 
