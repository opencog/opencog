/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/XmlLoader.h
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

#ifndef XML_LOADER_H
#define XML_LOADER_H

#include "XmlErrorHandler.h"
#include "VirtualWorldState.h"

#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>


namespace VirtualWorldData
{

class XmlLoader
{

private:

    XERCES_CPP_NAMESPACE::XercesDOMParser * parser;
    VirtualWorldData::XmlErrorHandler errorHandler;

    // Initializes the xerces-c XML Platform
    void initializeXMLPlatform();

    // Teminate the xerces-c XML Platform
    void terminateXMLPlatform();

    // Process the World State XML document
    void processWorldStateDocument(XERCES_CPP_NAMESPACE::DOMDocument * doc,
                                   VirtualWorldState & worldState);

    //
    void processEntityInfo(XERCES_CPP_NAMESPACE::DOMElement * element,
                           VirtualWorldState & worldState);

    //
    void processEntityElement(XERCES_CPP_NAMESPACE::DOMElement * element,
                              VirtualWorldData::VirtualEntity & entity);

    //
    void processAgentElement(XERCES_CPP_NAMESPACE::DOMElement * element,
                             VirtualWorldData::VirtualAgent & agent);

    //
    void processIndefiniteObjectInfo(XERCES_CPP_NAMESPACE::DOMElement * element,
                                     VirtualWorldState & worldState );

    //
    void processWorldStateInfo(XERCES_CPP_NAMESPACE::DOMElement * element,
                               VirtualWorldState & worldState );

public:

    XmlLoader();
    ~XmlLoader();

    bool fromFile(const std::string & filename, VirtualWorldState & worldState);

};
} // namespace

#endif
