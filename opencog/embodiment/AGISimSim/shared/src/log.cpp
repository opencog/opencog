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

    Peter Amstutz <tetron@interreality.org>*/

#include "simcommon.h"
#include "log.h"
#include <iostream>
#include <time.h>
#ifndef WIN32
#include <sys/time.h>
#endif

#ifdef WIN32
#include "serversocket.h"
#include <sys\timeb.h>
#endif

//------------------------------------------------------------------------------------------------------------
/** @file
    Implements Logging facilities. */
//------------------------------------------------------------------------------------------------------------	
//!!! was passiert hier ?
SINGLETON (boost::mutex, Log::channels_mutex)
std::map<const char*, Log*, Log::StrCmp>&  Log::channels()
{
    static std::map<const char*, Log*, Log::StrCmp>* c = new std::map<const char*, Log*, Log::StrCmp>();
    return *c;
}

boost::mutex  Log::defaultostream_mutex;
std::ostream* Log::defaultostream = &std::clog;

int  Log::masterLogLevel  = 4;
int  Log::defaultloglevel = 4;
bool Log::didReadEnv 	  = false;

SINGLETON(boost::mutex, Log::log_io_mutex)

//------------------------------------------------------------------------------------------------------------
Log::Log (const std::string& channel, std::ostream* outputstream)
    	 : channelname(channel), output(outputstream), loglevel(defaultloglevel)
{ }

//------------------------------------------------------------------------------------------------------------
Log::Log (const std::string& channel)
    : channelname(channel), output(defaultostream), loglevel(defaultloglevel)
{ }

static clock_t clock_offset = 0;

void Log::log(int level, const std::string& message)
{
    if (clock_offset == 0)
        clock_offset = clock(); //Cut off initial offset.
	
    if(level <= loglevel) {
        time_t t;
        time(&t);
        char* c = ctime(&t);
        c[strlen(c)-1] = 0;
			
/*
#ifdef WIN32
		timeval theTime;
//		gettimeofday(&theTime, NULL);
//		float clock_secs = theTime.tv_usec / 1000000.0f;

        boost::mutex::scoped_lock lk1(log_io_mutex());
        *output << "[" << c << " (" << level << ") " << channelname << "] " << message.substr(0, 1024) << '\n';
        output->flush();

#else 
*/
		timeval theTime;

#ifdef WIN32
	struct _timeb timebuffer;
	_ftime( &timebuffer );
	theTime.tv_sec = timebuffer.time;
	theTime.tv_usec = 1000*timebuffer.millitm;
#else

		gettimeofday(&theTime, NULL);

#endif
		float clock_secs = theTime.tv_usec / 1000000.0f;



        boost::mutex::scoped_lock lk1(log_io_mutex());

        *output << "[" << c << " { +" << (int)(clock_secs*1000) << "ms }"

		<< " (" << level << ") " << channelname << "] " << message.substr(0, 1024) << '\n';

        output->flush();

//#endif
//#warning "Config / Log deadlock?"
		try
		{
			if (Config::Exists() && IntConfig("LogOnFile"))
			{

#ifdef ENV_VAR
		char fullpath[1024];//can be [_MAX_PATH]
		char * AgisimEnv = getenv( "AGISIM" );
		if( AgisimEnv == NULL )
		{
			LOG("ENV_VAR", 0, "No Enviournment Variable Found. Can't log"); 
			return;
		}
		for(int i=0;i<=(int)strlen(AgisimEnv);i++){fullpath[i]=AgisimEnv[i];} //<=
#ifdef WIN32
		strcat(fullpath,"\\appdata\\logs\\agisim.log");
#else
		strcat(fullpath,"/appdata/logs/agisim.log");
#endif //WIN32
				FILE *f = fopen(fullpath,"rt+");
				if (f == NULL)
					f = fopen(fullpath,"wt");
				else
					fseek(f, 0L, SEEK_END);
#else

				FILE *f = fopen("agisim.log","rt+");
				if (f == NULL)
					f = fopen("agisim.log","wt");
				else
					fseek(f, 0L, SEEK_END);
#endif //ENV_VAR
		

#ifdef WIN32
				fprintf(f, "%s: %s\n", c, message.substr(0, 1024).c_str() );

#else

				fprintf(f, "%s (%d ms) %s\n",

					c,

					(int)(clock_secs*1000),

					message.substr(0, 1024).c_str() );

#endif
				fclose(f);
			}
		} catch(...) { //Will throw exceptions if config doesn't exist yet
		}

		clock_offset = clock();
    }
}

void Log::setLevel(int l)
{
    loglevel = l;
}

int Log::getLevel()
{
    return loglevel;
}

void Log::log(const std::string& channel, int level, const std::string& message)
{
    log(channel.c_str(), level, message);
}

void Log::log(const char* channel, int level, const std::string& message)
{
    boost::mutex::scoped_lock lk1(channels_mutex());

    if(channels().find(channel) == channels().end()) {
        boost::mutex::scoped_lock lk2(defaultostream_mutex);

        (channels()[channel] = new Log(channel, defaultostream))->setLevel(defaultloglevel);
    }
    channels()[channel]->log(level, message);
}

void Log::addChannel(Log* l) {
    boost::mutex::scoped_lock lk1(channels_mutex());

    channels()[l->channelname.c_str()] = l;
}

void Log::setDefaultOutputStream(std::ostream* o) {
    boost::mutex::scoped_lock lk1(defaultostream_mutex);

    defaultostream = o;
}

void Log::setDefaultLevel(int l) {
    defaultloglevel = l;
}

int Log::getDefaultLevel() {
    return defaultloglevel;
}

void Log::setMasterLevel(int l) {
    masterLogLevel = l;
}

int Log::getMasterLevel() {
    return masterLogLevel;
}

Log* Log::getLog(const std::string& channel) {
    return getLog(channel.c_str());
}

Log* Log::getLog(const char* channel) {
    if(! didReadEnv) readEnvironment();

    boost::mutex::scoped_lock lk1(channels_mutex());

    std::map<const char*, Log*, StrCmp>::const_iterator it = channels().find(channel);
    if(it != channels().end()) return (*it).second;
    else return 0;
}

void Log::readEnvironment()
{
    didReadEnv=true;

    char* env = getenv("AGISIM_LOG_MASTER");
    if(env) {
        setMasterLevel(atoi(env));  // TODO: test that env is numeric?
    }

    env = getenv("AGISIM_LOG_DEFAULT");
    if(!env)
        env = getenv("AGISIM_DEFAULT_LOGLEVEL");   // Deprecated, I guess
    if(env) {
        setDefaultLevel(atoi(env));  // TODO: test that env is numeric?
    }

    env = getenv("AGISIM_LOG");
    if(env) {
        char* e = strdup(env);
        env=e;
        for(;;) {
            char* col = strchr(env, '=');
            if(!col) return;
            *col=0;
            char* com = strchr(col+1, ',');
            if(com) *com=0;

            Log* l = getLog(env);
            if(! l) {
                l = new Log(env);
                addChannel(l);
            }
            time_t t;
            time(&t);
            char* c = ctime(&t);
            c[strlen(c)-1] = 0;

            boost::mutex::scoped_lock lk1(defaultostream_mutex);
            boost::mutex::scoped_lock l2(log_io_mutex());

            *defaultostream << "[" << c << " (" << 0 << ") " << "log" << "] "
                            << "Setting channel " << l->channelname << " to level " << (col+1) << '\n';
            defaultostream->flush();

            l->setLevel(atoi(col+1));

            if(com) env=com+1;
            else break;
        }
        free(e);
    }
}
