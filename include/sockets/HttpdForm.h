/** \file HttpdForm.h - read stdin, parse cgi input
 **
 **	Written: 1999-Feb-10 grymse@alhem.net
 **/

/*
Copyright (C) 1999-2007  Anders Hedstrom

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

#ifndef _SOCKETS_HttpdForm_H
#define _SOCKETS_HttpdForm_H

#include "sockets-config.h"
#include <string>
#include <list>

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class IFile;

/** Parse/store a http query_string/form-data body. 
	\ingroup webserver */
class HttpdForm
{
	/**
	 * Store the name/value pairs from a GET/POST operation.
	 * "name" does not have to be unique.
	 \ingroup webserver
	*/
	struct CGI
	{
		CGI(const std::string& n,const std::string& v) : name(n),value(v) {}
		CGI(const std::string& n,const std::string& v,const std::string& p) : name(n),value(v),path(p) {}
		std::string name;
		std::string value;
		std::string path;
	};
	/** list of key/value pairs. */
	typedef std::list<CGI *> cgi_v;

public:
	/**
	 * Default constructor (used in POST operations).
	 * Input is read from stdin. Number of characters to read
	 * can be found in the environment variable CONTENT_LENGTH.
	*/
	HttpdForm(IFile *, const std::string& content_type, size_t content_length);
	/**
	 * Another constructor (used in GET operations).
	 * Input is read from the environment variable QUERY_STRING.
	 * @param query_string The httpd server provided QUERY_STRING
	 * @param length Query string length.
	*/
	HttpdForm(const std::string& query_string,size_t length);
	~HttpdForm();

	void EnableRaw(bool);

	/** Encode characters '<' '>' '&' as &lt; &gt; &amp; */
	void strcpyval(std::string&,const char *) const;

	/* get names */
	bool getfirst(std::string& n) const;
	bool getnext(std::string& n) const;

	/* get names and values */
	bool getfirst(std::string& n,std::string& v) const;
	bool getnext(std::string& n,std::string& v) const;

	/* get value */
	int getvalue(const std::string& ,std::string& ) const;
	std::string getvalue(const std::string& ) const;
	size_t getlength(const std::string& ) const;
	cgi_v& getbase();

	const std::string& GetBoundary() const;

private:
	HttpdForm(const HttpdForm& ) {}
	HttpdForm& operator=(const HttpdForm& ) { return *this; }
	cgi_v m_cgi;
	mutable cgi_v::const_iterator m_current;
	std::string m_strBoundary;
	bool raw;
};


#ifdef SOCKETS_NAMESPACE
}
#endif

#endif // _SOCKETS_HttpdForm_H

