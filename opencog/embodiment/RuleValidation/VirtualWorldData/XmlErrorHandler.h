/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/XmlErrorHandler.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef XML_ERROR_HANDLER_H_
#define XML_ERROR_HANDLER_H_

#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>

namespace VirtualWorldData
{

class XmlErrorHandler : public XERCES_CPP_NAMESPACE::ErrorHandler
{
private:
    void resetErrors();

    void warning(const XERCES_CPP_NAMESPACE::SAXParseException& exc);

    void error(const XERCES_CPP_NAMESPACE::SAXParseException& exc);
    void fatalError(const XERCES_CPP_NAMESPACE::SAXParseException& exc);

public:
    XmlErrorHandler() { }

}; // class
}  // namespace

#endif // XML_ERROR_HANDLER_H_
