#ifndef _UTIL_LOGGING_H
#define _UTIL_LOGGING_H

//for now this is an evil little macro
//maybe later something nicer will go here...

#ifdef DISABLE_SIMPLE_LOGGING
#  define SIMPLE_LOG(logEvent)
#  define SIMPLE_LOGLN(logEvent)
#else
#  include <iostream>
#  include <sstream>
#  define SIMPLE_LOG(logEvent) \
    do { \
      std::stringstream __eda_log_buf; \
      __eda_log_buf << logEvent; \
      std::cout << __eda_log_buf.str(); \
      } while(0);
#  define SIMPLE_LOGLN(logEvent) \
    do { \
      std::stringstream __eda_log_buf; \
      __eda_log_buf << logEvent; \
      std::cout << __eda_log_buf.str() << " @@@ " << __FILE__ << " line " << __LINE__ << std::endl; \
      } while(0);
#endif

#if 1
#  include <iostream>
#  include <sstream>
#  define SIMPLE_PRINT(logEvent) \
    do { \
      std::stringstream __eda_log_buf; \
      __eda_log_buf << logEvent; \
      std::cout << __eda_log_buf.str(); \
      } while(0);
#  define SIMPLE_PRINTLN(logEvent) \
    do { \
      std::stringstream __eda_log_buf; \
      __eda_log_buf << logEvent; \
      std::cout << __eda_log_buf.str() << " @@@ " << __FILE__ << " line " << __LINE__ << std::endl; \
      } while(0);
#endif

#endif
