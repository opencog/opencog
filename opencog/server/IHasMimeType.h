/*
 * src/server/IHasMimeType.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_I_HAS_MIME_TYPE_H
#define _OPENCOG_I_HAS_MIME_TYPE_H

#include <string>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

/**
 * This class defines an interface that sockets compatible with opencog requests
 * must implement. It just defines a 'mimetype' attribute which the socket must
 * set. This is just to allow a request's implementation to render a different
 * view based on which client it is talking to.
 *
 * For instance, a plain text command line server socket would declare its
 * mime-type 'text/plain' so that the requests would print the output in plain
 * text. And a web-service based server socket would declare its mime-type as
 * 'application/xml' or 'text/html' so that the requests would print the output
 * in XMl or HTML.
 */
class IHasMimeType
{

protected:

    std::string _mimeType;

public:

    IHasMimeType(const std::string& s) : _mimeType(s) {}
    virtual ~IHasMimeType() {};

    virtual std::string& mimeType() {
        return _mimeType;
    }

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_I_HAS_MIME_TYPE_H
