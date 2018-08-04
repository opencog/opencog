/*
 * opencog/cogserver/server/RequestClassInfo.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>,
 * Simple-API implementation by Linas Vepstas <linasvepstas@gmail.com>
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


#ifndef _OPENCOG_REQUEST_CLASS_INFO_H
#define _OPENCOG_REQUEST_CLASS_INFO_H

#include <string>

namespace opencog
{

/**
 * This struct defines the extended set of attributes used by opencog requests.
 * The current set of attributes are:
 *     id:          the name of the request
 *     description: a short description of what the request does
 *     help:        an extended description of the request, listing multiple
 *                  usage patterns and parameters
 */
struct RequestClassInfo : public ClassInfo
{
    std::string description;
    std::string help;
    bool is_shell;
    /** Whether default shell should be hidden from help */
    bool hidden;

    RequestClassInfo() : is_shell(false), hidden(false) {};
    RequestClassInfo(const char* i, const char *d, const char* h,
            bool s = false, bool hide = false)
        : ClassInfo(i), description(d), help(h), is_shell(s), hidden(hide) {};
    RequestClassInfo(const std::string& i, 
                     const std::string& d,
                     const std::string& h, 
                     bool s = false,
                     bool hide = false)
        : ClassInfo(i), description(d), help(h), is_shell(s), hidden(hide) {};
};


} // namespace 

#endif // _OPENCOG_REQUEST_CLASS_INFO_H
