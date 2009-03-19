/*
 * Copyright 1999-2002,2004 The Apache Software Foundation.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * $Id: NameNodeFilter.hpp 176026 2004-09-08 13:57:07Z peiyongz $
 */

//
//  This file is part of the internal implementation of the C++ XML DOM.
//  It should NOT be included or used directly by application programs.
//
//  Applications should include the file <xercesc/dom/deprecated/DOM.hpp> for the entire
//  DOM API, or DOM_*.hpp for individual DOM classes, where the class
//  name is substituded for the *.
//

#ifndef DOM_NameNodeFilter_HEADER_GUARD_
#define DOM_NameNodeFilter_HEADER_GUARD_


#include "DOM_NodeFilter.hpp"
#include "NodeFilterImpl.hpp"
#include "DOMString.hpp"
#include "DOM_Node.hpp"

XERCES_CPP_NAMESPACE_BEGIN


class DEPRECATED_DOM_EXPORT NameNodeFilter : public NodeFilterImpl
{
	public:
		NameNodeFilter();
		virtual ~NameNodeFilter();

    // The name to compare with the node name. If null, all node names
    //  are successfully matched.
    void setName(DOMString name);

    // Return the name to compare with node name.
    DOMString getName();

    // If match is true, the node name is accepted when it matches.
    //  If match is false, the node name is accepted when does not match.
    void setMatch(bool match) ;

    // Return match value.
    bool getMatch();

    virtual DOM_NodeFilter::FilterAction acceptNode(DOM_Node n);

	private:
    DOMString fName;
    bool fMatch;

};

XERCES_CPP_NAMESPACE_END

#endif
