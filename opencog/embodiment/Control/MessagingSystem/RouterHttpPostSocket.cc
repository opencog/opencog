/*
 * opencog/embodiment/Control/MessagingSystem/RouterHttpPostSocket.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include "RouterHttpPostSocket.h"

#include <Sockets/HttpClientSocket.h>
#include <Sockets/Parse.h>

#include "util/StringManipulator.h"

RouterHttpPostSocket::RouterHttpPostSocket(ISocketHandler& h,const std::string& url_in) :HttpClientSocket(h, url_in) {
}

RouterHttpPostSocket::~RouterHttpPostSocket() {
}

void RouterHttpPostSocket::SetBody(const std::string& _body) {
    body = _body;
}

void RouterHttpPostSocket::Open() {
    TcpSocket::Open(GetUrlHost(), GetUrlPort());
}

void RouterHttpPostSocket::OnConnect() {
    SetMethod("POST");
    SetHttpVersion( "HTTP/1.1" );
    AddResponseHeader( "Host", GetUrlHost()); // oops - this is actually a request header that we're adding..
    AddResponseHeader( "User-agent", MyUseragent());
    AddResponseHeader( "Accept", "text/html, text/plain, */*;q=0.01" );
    //AddResponseHeader( "Connection", "keep-alive" ); // This was delaying the http request to finish
    AddResponseHeader( "Connection", "close" );
    // TODO: use a method to set the content-type 
    //AddResponseHeader( "Content-type", "application/x-www-form-urlencoded" ); // Works with this content-type as well 
    AddResponseHeader( "Content-type", "text/plain" ); 
    AddResponseHeader( "Content-length", opencog::toString(body.size()) );
    SendRequest();

    // send body
    Send( body );
}

