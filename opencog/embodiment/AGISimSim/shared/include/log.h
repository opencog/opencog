/***************************************************************************
 *  Log class.        
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *
 *	19.01.06	FP	formatting 	
 ****************************************************************************/
 
 
/*	This file has been altered to suit AgiSim
	by Ari Heljakka / Novamente LLC.
	
	The original copyright and license follows:

    This file is part of the Virtual Object System of
    the Interreality project (http://interreality.org).

    Copyright (C) 2001-2003 Peter Amstutz

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA

    Peter Amstutz <tetron@interreality.org>
*/

#ifndef _LOG_HH_
#define _LOG_HH_


/** @file Defines Log.
    @see  LOG */

#include <string>
#include <map>
#include <boost/thread/mutex.hpp>

/** @def LOG(c, l, m) log.hh vos/vos/log.hh

    @param c (string) the logging channel, which will be created if it does not exist
    @param l (int) the logging level. 0 is the highest priority, 6
        is the lowest priority. Generally, you should use 0 for
        critical errors, 1 for warnings, 2 for important information,
        3, 4 and 5 for different levels of debugging spew.
    @param m The actual log message.  This is actually used as an
        output into a stream, so the whole C++ stream output API is available
        In other words, can use expressions like:
        @code
            LOG("foo", 3, "x = " << x << "\n");
        @endcode

    You can control log levels using the setLevel(),
    setDefaultLevel(), setMasterLevel(), and also by setting the
    environment variables VOS_LOG, VOS_LOG_DEFAULT and VOS_LOG_MASTER.
    VOS_LOG sets the log level on specific channel; VOS_LOG_DEFAULT
    sets the default level for newly created log channels, and
    VOS_LOG_MASTER sets a hard cutoff that disables *all* logging
    above the supplied level, overriding VOS_LOG and VOS_LOG_DEFAULT).
    Generally you want to use VOS_LOG and VOS_LOG_DEFAULT, and avoid
    VOS_LOG_MASTER.

    The format of the VOS_LOG environment variable is
    <code>channel1=level1,channel2=level2,...</code>.  Don't use any
    spaces.  VOS_LOG_DEFAULT and VOS_LOG_MASTER should contain a
    single positive integer.  All environment variables are entirely
    optional.

    The LOG macro firsts check the master loglevel.  If the log level
    is greater than the master level, no logging message will be
    printed.  Then the LOG checks to see if the channel exists.  If
    not, it is created and set to the default level.  Finally if the
    log level is less than or equal to the channel's current level,
    the message is printed.

    Note that the log level is checked before actually creating the
    output text, so the logging output string isn't created unless it
    is actually going to be printed.
*/



#ifdef USE_STRSTREAM
# include <strstream>
# define VOS_LOG_MAXLINELENGTH 1024

// Macro to write text in LogFile or LogTerminal
# define LOG(c, l, m) {                						 \
    if(l <= Log::masterLogLevel) {      					 \
        Log* _lg = Log::getLog(c);         					 \
        if(! _lg) { _lg=new Log(c); Log::addChannel(_lg); }  \
        if(_lg->getLevel() >= l) {          				 \
          char _y[VOS_LOG_MAXLINELENGTH];                    \
          memset(_y, 0, sizeof(_y));       					 \
          std::ostrstream _x(_y, sizeof(_y)-1); 			 \
          _x << m;                        					 \
          _lg->log(l, _x.str());           					 \
        } 													 \
    }                                  						 \
}
#else
# include <sstream>
# define LOG(c, l, m) {								         \
    if(l <= Log::masterLogLevel) { 							 \
        Log* _lg = Log::getLog(c); 							 \
        if(! _lg) { _lg=new Log(c); Log::addChannel(_lg); }  \
        if(_lg->getLevel() >= l) {  						 \
          std::ostringstream _x;        					 \
          _x << m;                 							 \
          _lg->log(l, _x.str());    						 \
        }                          							 \
    } 														 \
}
#endif

//------------------------------------------------------------------------------------------------------------
/** Symbolic level names for use with the LOG macro.  The log level threshold
    is typically set at LOG_NOTIFY, or LOG_DETAIL if compiled in debugging mode. */
//------------------------------------------------------------------------------------------------------------
enum LogLevelSym {    
    LOG_ERROR = 0, 	/// Critical, possibly fatal errors    
    LOG_WARN, 		/// Important warnings    
    LOG_NOTIFY,		/// Important information about program state or progress.    
    LOG_DETAIL,		/// Details about program state or progress, which may be useful for users    
    LOG_DEBUG,    	/// Verbose program debugging, useful for developers, and details about VOS state or progress. 
    LOG_MSGDATA		/// Contents of messages.
};

//------------------------------------------------------------------------------------------------------------
/** @class Log log.h

    The logging class.  The main user interface to this class is the
    macro LOG(), but you can set some global options using the static
    methods in this class as well as you can set properties of individual
    channels. */
//------------------------------------------------------------------------------------------------------------
class Log {
private:
    std::string   channelname;
    std::ostream* output;
    int 	  loglevel;
    static int    defaultloglevel;
    static bool   didReadEnv;
    static std::ostream* defaultostream;

    class StrCmp {
    public:
        inline bool operator()(const char* p, const char* q) const {
            return (strcmp(p, q) < 0); //!!! Was passiert hier ? operator() ?
        };
    };

    static boost::mutex& 			  channels_mutex();
    static std::map <const char*, Log*, StrCmp>&  channels();
    static boost::mutex 			  defaultostream_mutex;
    static boost::mutex& 			  log_io_mutex();
public:

    /** A level that applies to all channels. (default is 10) */
    static int masterLogLevel;

    /** Construct a new logging channel.  Usually you
        don't need to call this, as it will be created automatically
        when a new channel is refered to.
        @param channel the channel name that will select this log object
        @param outputstream the output stream this log will write to
        @internal
    */
    Log(const std::string& channel, std::ostream* outputstream);

    /** Construct a new logging channel.  Usually you
        don't need to call this, as it will be created automatically
        when a new channel is refered to.
        @param channel the channel name that will select this log object
        @internal
    */
    Log(const std::string& channel);

    /** Write a line to the log stream.  You probably want to be
        using LOG(), however.
        @param level the level to log at.  If the logging channel
        level is lower than this level, the log message will be suppressed
        @param message the message std::string to print
        @internal
    */
    void log(int level, const std::string& message);

    /** Set the log level for this channel. Log levels for channels
        can also be set with an environment variable VOS_LOG, which
        takes the format "channel1=level1,channel1=level1,..." --
        where channel is the name of a log channel, and level is the
        integer level for that channel.

        @param loglevel the level, messages with a level higher than this will be suppressed.
     */
    void setLevel(int loglevel);

    /** Get the log level for this channel
        @returns loglevel the level, messages with a level higher than this will be suppressed.
     */
    int getLevel();

    /** Log to a particular channel, which will be created if it does
        not exist.  You probably want to be using LOG(), however.
        @param channel the channel to log in
        @param level the level to log at.  If the logging channel
        level is lower than this level, the log message will be suppressed
        @param message the message std::string to print
        @internal
     */
    static void log(const char* channel, int level, const std::string& message);
    static void log(const std::string& channel, int level, const std::string& message);

    /** Obtain the log object for a particular channel
        @param channel the channel desired.
        @returns the log object desired, or 0 if it does not exist
    */
    static Log* getLog(const std::string& channel);

    /** Obtain the log object for a particular channel
        @param channel the channel desired.
        @returns the log object desired, or 0 if it does not exist
    */
    static Log* getLog(const char* channel);

    /** Add a new channel to the channel table.
        @param l the log channel
     */
    static void addChannel(Log* l);

    /** Sets the default output stream for new log objects.  The
        starting default output stream is "clog" (which is generally
        an alias for stderr.)
        @param o the output stream
    */
    static void setDefaultOutputStream(std::ostream* o);

    /** Sets the default log level for new log channels.
        @param loglevel the level, messages with a level higher than
        this will be suppressed.
    */
    static void setDefaultLevel(int loglevel);

    /** Returns the default log level for new log channels. */
    static int getDefaultLevel();

    /** Sets the master log level for all log objects. The master log
        level applies to all logging statements, regardless of channel
        name.
        @param loglevel the new level. Messages with a level higher
        than this will be suppressed.
    */
    static void setMasterLevel(int loglevel);

    /** Returns the master log level. */
    static int getMasterLevel();

    /** Read the environment variable VOS_LOG and set/create the
        appropriate log channels to the specified levels.
        @note This is called exactly once automatically by getLog() so
        you probably don't ever have to call this.
    */

    static void readEnvironment();
};

#endif
