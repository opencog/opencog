/*
 * opencog/server/Request.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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


#ifndef _OPENCOG_REQUEST_H
#define _OPENCOG_REQUEST_H

#include <string>
#include <list>

#include <Sockets/TcpSocket.h>

#include <opencog/server/Factory.h>

namespace opencog
{

class Request
{

protected:

    TcpSocket*             _sock;
    std::list<std::string> _parameters;
    std::string            _mimeType;

public:

    Request();
    virtual ~Request();

    virtual bool execute       (void) = 0;
    virtual void send          (const std::string& msg) const;

    virtual void setSocket     (TcpSocket*);
    virtual void setParameters (const std::list<std::string>& params);
    virtual void addParameter  (const std::string& param);
};

struct RequestClassInfo : public ClassInfo
{
    std::string description;
    std::string help;

    RequestClassInfo() {};
    RequestClassInfo(const char* i, const char *d, const char* h)
        : ClassInfo(i), description(d), help(h) {};
    RequestClassInfo(const std::string& i, const std::string& d, const std::string& h)
        : ClassInfo(i), description(d), help(h) {};
};

} // namespace 

#endif // _OPENCOG_REQUEST_H
