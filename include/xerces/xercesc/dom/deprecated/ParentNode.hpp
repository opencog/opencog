#ifndef ParentNode_HEADER_GUARD_
#define ParentNode_HEADER_GUARD_
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

//
//  This file is part of the internal implementation of the C++ XML DOM.
//  It should NOT be included or used directly by application programs.
//
//  Applications should include the file <xercesc/dom/deprecated/DOM.hpp> for the entire
//  DOM API, or DOM_*.hpp for individual DOM classes, where the class
//  name is substituded for the *.
//

/*
 * $Id: ParentNode.hpp 176026 2004-09-08 13:57:07Z peiyongz $
 */

/**
 * ParentNode inherits from ChildImpl and adds the capability of having child
 * nodes. Not every node in the DOM can have children, so only nodes that can
 * should inherit from this class and pay the price for it.
 * <P>
 * ParentNode, just like NodeImpl, also implements NodeList, so it can
 * return itself in response to the getChildNodes() query. This eliminiates
 * the need for a separate ChildNodeList object. Note that this is an
 * IMPLEMENTATION DETAIL; applications should _never_ assume that
 * this identity exists.
 * <P>
 * While we have a direct reference to the first child, the last child is
 * stored as the previous sibling of the first child. First child nodes are
 * marked as being so, and getNextSibling hides this fact.
 * <P>Note: Not all parent nodes actually need to also be a child. At some
 * point we used to have ParentNode inheriting from NodeImpl and another class
 * called ChildAndParentNode that inherited from ChildNode. But due to the lack
 * of multiple inheritance a lot of code had to be duplicated which led to a
 * maintenance nightmare. At the same time only a few nodes (Document,
 * DocumentFragment, Entity, and Attribute) cannot be a child so the gain is
 * memory wasn't really worth it. The only type for which this would be the
 * case is Attribute, but we deal with there in another special way, so this is
 * not applicable.
 *
 * <p><b>WARNING</b>: Some of the code here is partially duplicated in
 * AttrImpl, be careful to keep these two classes in sync!
 *
 **/

#include <xercesc/util/XercesDefs.hpp>
#include "ChildNode.hpp"

XERCES_CPP_NAMESPACE_BEGIN

class DEPRECATED_DOM_EXPORT ParentNode: public ChildNode {
public:
    DocumentImpl            *ownerDocument; // Document this node belongs to

    ChildNode                *firstChild;

public:
    ParentNode(DocumentImpl *ownerDocument);
    ParentNode(const ParentNode &other);

    virtual DocumentImpl * getOwnerDocument();
    virtual void setOwnerDocument(DocumentImpl *doc);

    virtual NodeListImpl *getChildNodes();
    virtual NodeImpl * getFirstChild();
    virtual NodeImpl * getLastChild();
    virtual unsigned int getLength();
    virtual bool        hasChildNodes();
    virtual NodeImpl    *insertBefore(NodeImpl *newChild, NodeImpl *refChild);
    virtual NodeImpl    *item(unsigned int index);
    virtual NodeImpl    * removeChild(NodeImpl *oldChild);
    virtual NodeImpl    *replaceChild(NodeImpl *newChild, NodeImpl *oldChild);
    virtual void        setReadOnly(bool isReadOnly, bool deep);

    //Introduced in DOM Level 2
    virtual void	normalize();

    // NON-DOM
    // unlike getOwnerDocument this never returns null, even for Document nodes
    virtual DocumentImpl * getDocument();
protected:
    void cloneChildren(const NodeImpl &other);
    ChildNode * lastChild();
    void lastChild(ChildNode *);

    /** Cached node list length. */
    int fCachedLength;

    /** Last requested child. */
    ChildNode * fCachedChild;

    /** Last requested child index. */
    int fCachedChildIndex;
};

XERCES_CPP_NAMESPACE_END

#endif
