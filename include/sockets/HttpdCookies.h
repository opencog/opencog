/** \file HttpdCookies.h
*/
/*
Copyright (C) 2003-2007  Anders Hedstrom

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

#ifndef _SOCKETS_HttpdCookies_H
#define _SOCKETS_HttpdCookies_H

#include "sockets-config.h"
#include <list>
#include <string>

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


//! Store the cookies name/value pairs.



//! Retrieve and manage cookies during a cgi call.
class HTTPSocket;

/** HTTP Cookie parse/container class. 
\sa HttpdSocket
\sa HttpdForm 
\ingroup webserver */
class HttpdCookies 
{
	/** list of key/value structs. */
	typedef std::list<std::pair<std::string, std::string> > cookie_v;

public:
	HttpdCookies();
	HttpdCookies(const std::string& query_string);
	~HttpdCookies();

	void add(const std::string& s);

	bool getvalue(const std::string&,std::string&) const;
	void replacevalue(const std::string& ,const std::string& );
	void replacevalue(const std::string& ,long);
	void replacevalue(const std::string& ,int);
	size_t getlength(const std::string& ) const;
	void setcookie(HTTPSocket *,const std::string& d,const std::string& p,const std::string& c,const std::string& v);
	void setcookie(HTTPSocket *,const std::string& d,const std::string& p,const std::string& c,long v);
	void setcookie(HTTPSocket *,const std::string& d,const std::string& p,const std::string& c,int v);
	const std::string& expiredatetime() const;

	cookie_v& GetHttpdCookies() { return m_cookies; }

	void Reset();

private:
	cookie_v m_cookies;
	mutable std::string m_date;
};


#ifdef SOCKETS_NAMESPACE
}
#endif

#endif // _SOCKETS_HttpdCookies_H

