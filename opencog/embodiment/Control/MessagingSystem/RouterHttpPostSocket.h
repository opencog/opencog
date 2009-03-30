/*
 * opencog/embodiment/Control/MessagingSystem/RouterHttpPostSocket.h
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
#include <Sockets/HttpClientSocket.h>

class RouterHttpPostSocket : public HttpClientSocket {
    public:
        /* client constructor,
         * \param url_in = 'http://host:port/resource' 
         */
        RouterHttpPostSocket(ISocketHandler&,const std::string& url_in);
        ~RouterHttpPostSocket();

        /** Add body to post. */
        void SetBody(const std::string& body);

        /** connect to host:port derived from url in constructor */
        void Open();

        /** http post client implemented in OnConnect */
        void OnConnect();

    private: 

        std::string body;

        //std::string m_host;
        //port_t m_port;

};
