/**
 **	\file AjpBaseSocket.h
 **	\date  2007-10-05
 **	\author grymse@alhem.net
**/
/*
Copyright (C) 2007  Anders Hedstrom

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
#ifndef _SOCKETS_AjpBaseSocket_H
#define _SOCKETS_AjpBaseSocket_H

#include "TcpSocket.h"
#include <map>

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif

class AjpBaseSocket : public TcpSocket
{
	class Initializer
	{
	public:
		Initializer();
		virtual ~Initializer() {}

		std::map<int, std::string> Method;
		std::map<int, std::string> Header;
		std::map<int, std::string> Attribute;

		std::map<std::string, int> ResponseHeader;

	};

public:
	AjpBaseSocket(ISocketHandler& h);

	void OnRawData(const char *buf, size_t sz);

	virtual void OnHeader( short id, short len ) = 0;
	virtual void OnPacket( const char *buf, size_t sz ) = 0;

protected:
	unsigned char get_byte(const char *buf, int& ptr);
	bool get_boolean(const char *buf, int& ptr);
	short get_integer(const char *buf, int& ptr);
	std::string get_string(const char *buf, int& ptr);

	void put_byte(char *buf, int& ptr, unsigned char zz);
	void put_boolean(char *buf, int& ptr, bool zz);
	void put_integer(char *buf, int& ptr, short zz);
	void put_string(char *buf, int& ptr, const std::string& zz);

	static Initializer Init;

private:
	int m_state;
	int m_length;
	int m_ptr;
	char m_message[8192];
};


#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif

#endif // _SOCKETS_AjpBaseSocket_H

