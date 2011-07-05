/*
 * opencog/embodiment/Control/PerceptionActionInterface/EmbodimentErrorHandler.h
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

#ifndef EMBODIMENT_ERROR_HANDLER_H_
#define EMBODIMENT_ERROR_HANDLER_H_

#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include "PAIUtils.h"

namespace opencog { namespace pai {

using XERCES_CPP_NAMESPACE::ErrorHandler;
using XERCES_CPP_NAMESPACE::SAXParseException;

/**
 * This is an extension of ErrorHandler to log the parse errors according with OpenCog log policy.
 */
class EmbodimentErrorHandler : public ErrorHandler
{
    void warning(const SAXParseException& exc);
    void error(const SAXParseException& exc);
    void fatalError(const SAXParseException& exc);
    void resetErrors();

public:
    EmbodimentErrorHandler() {
        PAIUtils::initializeXMLPlatform();
    }

};

} } // namespace opencog::pai

#endif // EMBODIMENT_ERROR_HANDLER_H_
