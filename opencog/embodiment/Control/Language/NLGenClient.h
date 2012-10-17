/*
 * opencog/embodiment/Control/OperationalAvatarController/NLGenClient.h
 *
 * Copyright (C) 2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Fabricio Silva 
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

#ifndef NLGENCLIENT_H
#define NLGENCLIENT_H

#include <string>
#include <cstdlib>
#include <iostream>
#include <boost/version.hpp>

#define BOOST_MINOR_VERSION (BOOST_VERSION / 100 % 1000)
#define BOOST_ACCEPTED_VERSION 35

#if BOOST_MINOR_VERSION >= BOOST_ACCEPTED_VERSION
#include <boost/asio.hpp>
using boost::asio::ip::tcp;
#endif

namespace opencog { namespace oac {

class NLGenClient {

public:
    NLGenClient( const std::string& host, unsigned int port );
    
    virtual ~NLGenClient( );
    
    bool connect( void );
    
    std::string send(const std::string& text);
    
private:
#if BOOST_MINOR_VERSION >= BOOST_ACCEPTED_VERSION
    boost::asio::io_service io_service;
    tcp::socket *socket;
    std::string host; 
    unsigned int port;
    bool connected;
#endif
};

} } // namespace opencog::oac

#endif //NLGENCLIENT_H

