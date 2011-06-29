/*
 * opencog/embodiment/Control/PerceptionActionInterface/PetaverseErrorHandler.h
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

#ifndef PETAVERSES_ERROR_HANDLER_H_
#define PETAVERSES_ERROR_HANDLER_H_
/**
 * This is an extension of ErrorHandler to log the parse errors according with Petaverse log policy.
 */

#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include "PAIUtils.h"

namespace opencog { namespace pai {

class PetaverseErrorHandler : public XERCES_CPP_NAMESPACE::ErrorHandler
{
    void warning(const XERCES_CPP_NAMESPACE::SAXParseException& exc);
    void error(const XERCES_CPP_NAMESPACE::SAXParseException& exc);
    void fatalError(const XERCES_CPP_NAMESPACE::SAXParseException& exc);
    void resetErrors();

public:
    PetaverseErrorHandler() {
        PAIUtils::initializeXMLPlatform();
    }

};

} } // namespace opencog::pai

#endif // PETAVERSES_ERROR_HANDLER_H_
