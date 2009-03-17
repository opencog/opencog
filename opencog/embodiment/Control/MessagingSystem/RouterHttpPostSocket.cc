#include "RouterHttpPostSocket.h"

#include <Sockets/HttpClientSocket.h>
#include <Sockets/Parse.h>

#include "util/StringManipulator.h>

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

