/** \file EventTime.h
 **	\date  2005-12-07
 **	\author grymse@alhem.net
**/
/*
Copyright (C) 2005,2007  Anders Hedstrom

This library is made available under the terms of the GNU GPL.

If you would like to use this library in a closed-source application,
a separate license agreement is available. For information about 
the closed-source license agreement for the C++ sockets library,
please visit http://www.alhem.net/Sockets/license.html and/or
email license@alhem.net.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/
#ifndef _SOCKETS_EventTime_H
#define _SOCKETS_EventTime_H

#include "sockets-config.h"
#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


#if defined( _WIN32) && !defined(__CYGWIN__)
typedef __int64 mytime_t;
#else
#include <inttypes.h> // int64_t
typedef int64_t mytime_t;
#endif


/** \defgroup timer EventTimer event handling */

/** EventTime primitive, returns current time as a 64-bit number.
	\ingroup timer */
class EventTime
{
public:
	EventTime();
	EventTime(mytime_t sec,long usec);
	~EventTime();

	static mytime_t Tick();

	operator mytime_t () { return m_time; }
	EventTime operator - (const EventTime& x) const;
	bool operator < (const EventTime& x) const;

private:
	EventTime(const EventTime& ) {} // copy constructor
	EventTime& operator=(const EventTime& ) { return *this; } // assignment operator
	mytime_t m_time;
};



#ifdef SOCKETS_NAMESPACE
}
#endif

#endif // _SOCKETS_EventTime_H

