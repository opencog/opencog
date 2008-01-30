/** \file Event.h
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
#ifndef _SOCKETS_Event_H
#define _SOCKETS_Event_H

#include "sockets-config.h"
#ifdef _WIN32
#else
#include <sys/select.h>
#endif
#include "EventTime.h"

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class IEventOwner;

/** Store information about a timer event.
	\ingroup timer */
class Event
{
public:
	Event(IEventOwner *,long sec,long usec);
	~Event();

	bool operator<(Event&);
	long GetID();
	const EventTime& GetTime() const;
	IEventOwner *GetFrom();

private:
	Event(const Event& ) {} // copy constructor
	Event& operator=(const Event& ) { return *this; } // assignment operator
	IEventOwner *m_from;
	EventTime m_time;
	static long m_unique_id;
	long m_id;
};



#ifdef SOCKETS_NAMESPACE
}
#endif

#endif // _SOCKETS_Event_H

