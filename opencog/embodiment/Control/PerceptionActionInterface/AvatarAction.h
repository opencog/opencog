/*
 * opencog/embodiment/Control/PerceptionActionInterface/AvatarAction.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi, Carlos Lopes
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

#ifndef AVATARACTION_H
#define AVATARACTION_H

#include "ActionType.h"
#include "ActionParameter.h"

#include <list>
#include <string>
#include <exception>

namespace opencog { namespace pai {

using namespace std;

/**
 * AvatarAction class represents an action element that is part of an pet action-plan.
 * This is a generic container, that is, it can be any type of action defined by ESC XML Scheme.
 */
class AvatarAction
{
private:

    /**
     * type of the action
     */
    ActionTypeCode type;

    /**
     * the sequence number of the action in an action plan
     */
    unsigned int sequence;

    /**
     * the parameters of the action
     */
    list<ActionParameter> parameters;

    /**
     * Convenience method to parse XML
     */
    static char *getAttribute(XERCES_CPP_NAMESPACE::DOMElement *element, const char *attrName);

    /**
     * Convenience method to parse XML
     */
    static XERCES_CPP_NAMESPACE::DOMNodeList *getChildren(XERCES_CPP_NAMESPACE::DOMElement *element, const char *tagName);


public:

    /**
     * Empty constructor
     */
    AvatarAction();

    /**
     * Constructor
     */
    AvatarAction(const ActionType& type);

    /**
     * Destructor
     */
    ~AvatarAction();

    /**
     * Adds a parameter to this action. The order of the parameters in this action depends on
     * the order this method is called to add each parameter.
     * @param param ActionParameter object to be added to pet action
     *
     * @throws InvalidParamException if the maximal number of parameters had been
     * reached already or if the parameter to be added is not of the expected type.
     */
    void addParameter(ActionParameter param) throw (opencog::InvalidParamException, std::bad_exception);

    /**
     * Gets the list of parameter of this action.
     * @return The list of ActionParamters of this action.
     */
    const list<ActionParameter>& getParameters() const;

    /**
     * Check if the parameters of this AvatarAction are OK with its ActionType (i.e., if they have the right number of parameters
     * and if each parameter has the right type, according with its position).
     *
     * @return true, if the added parameters are valid. False, owhterwise
     */
    bool containsValidParameters() const;

    /**
     * For a given action type, check if the given parameters are valid.
     * @param actionType the type of action which the parameters will be verified
     * @param params a list of parameters to be verified against the action type.
     * @return an empty string, if the paraemters are valid. Otherwise, an error message with one reason why the parameters are not valid
     */
    static std::string validateParameters(const ActionType& actionType, const list<ActionParameter>& params);

    /**
     * Create an action element in a DOMDocument XML document. An action element has the form:
     *   <action name="" sequence="">
     *     [<param name="" type="" ... />
     *     ...
     *     ]
     *   </action>
     *
     * @param doc The XML document where the element will be inserted.
     * @param parent The parent element where the element will created inside.
     * @return The DOMElement just created.
     */
    XERCES_CPP_NAMESPACE::DOMElement* createPVPXmlElement(XERCES_CPP_NAMESPACE::DOMDocument* doc,
            XERCES_CPP_NAMESPACE::DOMElement* parent) const;

    // TODO: implement an assignment operator (=) and change the return type of the method below to AvatarAction
    /**
     * Builds a AvatarAction using a DOMElement (usually created by a XML parser).
     *
     * The DOMElement follows the same convention described in method createPVPXmlElement().
     */
    static AvatarAction *factory(XERCES_CPP_NAMESPACE::DOMElement *domElement) throw (opencog::InvalidParamException, std::bad_exception);

    /**
     * Sets the type of this action.
     * @param The pet action type.
     * @throws RuntimeException if the already added parameters become invalid for the given action type.
     */
    void setType(const ActionType& type) throw (opencog::InvalidParamException, std::bad_exception);

    /**
     * @return The pet action type.
     */
    const ActionType& getType() const;

    /**
     * @return The pet action name.
     */
    const string& getName() const;

    /**
     * @param The pet action sequence value in a list of pet actions.
     */
    void setSequence(unsigned int sequence);

    /**
     * @return The pet action sequence value in a list of pet actions.
     */

    unsigned int getSequence() const;

    /**
     * @return A string which describes the action
     */
    std::string stringRepresentation() const;

}; // class
} }  // namespace opencog::pai

#endif
