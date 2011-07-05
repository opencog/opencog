/*
 * opencog/embodiment/Control/PerceptionActionInterface/EmbodimentDOMParser.cc
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

#include "EmbodimentDOMParser.h"
#include <opencog/util/Logger.h>
#include <xercesc/dom/DOMException.hpp>

using namespace opencog::pai;

void EmbodimentDOMParser::error (const unsigned int    errCode,
                                const XMLCh *const   errDomain,
                                const ErrTypes  type,
                                const XMLCh *const   errorText,
                                const XMLCh *const   systemId,
                                const XMLCh *const   publicId,
                                const XMLSSize_t  lineNum,
                                const XMLSSize_t  colNum)
{
    XERCES_CPP_NAMESPACE::XercesDOMParser::error(errCode, errDomain, type, errorText, systemId, publicId, lineNum, colNum);

    char* errDomainStr = XERCES_CPP_NAMESPACE::XMLString::transcode(errDomain);
    char* errorTextStr  = XERCES_CPP_NAMESPACE::XMLString::transcode(errorText);
    char* systemIdStr  = XERCES_CPP_NAMESPACE::XMLString::transcode(systemId);
    char* publicIdStr  = XERCES_CPP_NAMESPACE::XMLString::transcode(publicId);
    logger().error("EmbodimentDOMParser - XML Parser error:"
                 "\n  code = %u"
                 "\n  domain = %s"
                 "\n  type = %d"
                 "\n  text = %s"
                 "\n  systemId = %s"
                 "\n  publicId = %s"
                 "\n  lineNum = %d"
                 "\n  colNum = %d",
                 errDomainStr,
                 type,
                 errorTextStr,
                 systemIdStr,
                 publicIdStr,
                 lineNum,
                 colNum);
    XERCES_CPP_NAMESPACE::XMLString::release(&errDomainStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&errorTextStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&systemIdStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&publicIdStr);
    throw new XERCES_CPP_NAMESPACE::DOMException(errCode, errorText);
}

