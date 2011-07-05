/*
 * opencog/embodiment/Control/PerceptionActionInterface/EmbodimentDOMParser.h
 *
 * Copyright (C) 2011 OpenCog Foundation
 * Copyright (C) 2002-2009 Novamente LLC
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

#ifndef EMBODIMENT_DOM_PARSER_H_
#define EMBODIMENT_DOM_PARSER_H_

#include <xercesc/parsers/XercesDOMParser.hpp>
#include "PAIUtils.h"

namespace opencog { namespace pai {

/**
 * This is an extension of XercesDOMParser to log the parse errors according
 * with Embodiment log policy.
 */
class EmbodimentDOMParser : public XERCES_CPP_NAMESPACE::XercesDOMParser
{

public:
    EmbodimentDOMParser() {
        PAIUtils::initializeXMLPlatform();
    }
    virtual ~EmbodimentDOMParser() {};

    void error(const unsigned int    errCode,
               const XMLCh *const   errDomain,
               const ErrTypes  type,
               const XMLCh *const   errorText,
               const XMLCh *const   systemId,
               const XMLCh *const   publicId,
               const XMLSSize_t  lineNum,
               const XMLSSize_t  colNum);

};

} } // namespace opencog::pai

#endif // EMBODIMENT_DOM_PARSER_H_
