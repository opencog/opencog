/*
 * opencog/embodiment/Control/PerceptionActionInterface/ActionPlan.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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

#ifndef ACTIONPLAN_H_
#define ACTIONPLAN_H_

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include "PetAction.h"

#include <list>
#include <vector>
#include <exception>

namespace PerceptionActionInterface
{

typedef std::string ActionPlanID;

class ActionPlan
{

private:
    ActionPlanID id;
    std::vector<PetAction> actions;
    std::vector<bool> actionTried;
    std::vector<bool> actionDone;
    std::vector<bool> actionFailed;

    bool isSeqNumberValid(unsigned int seqNumber) const;

public:

    /**
     * Basic constructor: needed by std::lists or any other stl container.
     * However, an ActionPlan object built this way will have an undefined id.
     */
    ActionPlan();

    /**
     * Constructor
     * @param id the ID of this action plan
     */
    ActionPlan(ActionPlanID id);

    /**
     * Gets the ID of this action plan
     */
    ActionPlanID getID() const;

    /**
     * Adds an action to this plan
     *
     * @param action A reference to the action to be added. A new PetAction object is created from this argument and
     *         its sequence number is set, according with its position in this action plan.
     * @return The position of the added action in this plan.
     * @throws InvalidParamException if the action contains invalid parameters for its type.
     */
    unsigned int addAction(const PetAction& action) throw (opencog::InvalidParamException, std::bad_exception);

    /**
     * Return true iff there is no actions in the ActionPlan
     */
    bool empty() const;

    /**
     * Gets the number of actions in this action plan
     */
    unsigned int size() const;

    /**
     * Gets the action of this action plan, given its sequence number
     */
    const PetAction& getAction(unsigned int seqNumber) const throw (opencog::IndexErrorException, std::bad_exception);

    /**
     * Generates the corresponding PVP's message in XML format for this action plan
     * @param petId the id of the pet for which the PVP message will be sent.
     */
    string getPVPmessage(const std::string& petId) const;

    /**
	 * Generates the corresponding PVP's message in XML format for a given atomic action
	 * of this action plan.
	 *
	 * @param petId the id of the pet
	 * @param actionSeqNum the action sequence number to be capsulated in XML.
	 */
	string getPVPmessage(const std::string& petId, unsigned int actionSeqNum) const;

    /**
     * Check if the action with the given sequence number was marked tried
     * @param seqNumber the sequence number of the action inside this action plan
     * @return true if the specific action was marked as tried. False, otherwise
     */
    bool isTried(unsigned int seqNumber) const;

    /**
     * Check if the action with the given sequence number was marked done
     * @param seqNumber the sequence number of the action inside this action plan
     * @return true if the specific action was marked as done. False, otherwise
     */
    bool isDone(unsigned int seqNumber) const;

    /**
     * Check if the action with the given sequence number was marked failed
     * @param seqNumber the sequence number of the action inside this action plan
     * @return true if the specific action was marked as failed. False, otherwise
     */
    bool isFailed(unsigned int seqNumber) const;

    /**
     * Mark the action with the given sequence number as tried
     * @param seqNumber the sequence number of the action inside this action plan
     */
    void markAsTried(unsigned int seqNumber);

    /**
     * Mark the action with the given sequence number as done
     * @param seqNumber the sequence number of the action inside this action plan
     */
    void markAsDone(unsigned int seqNumber);

    /**
     * Mark the action with the given sequence number as failed
     * @param seqNumber the sequence number of the action inside this action plan
     */
    void markAsFailed(unsigned int seqNumber);

    /**
     * Check if the action plan has finished, i.e., all its actions are marked as
     * done or failed.
     * @return true if this action plan is finished. False otherwise.
     */
    bool isFinished() const;

    /**
     * Check if the action plan has failed, i.e., any of its actions is marked as
     * failed.
     * @return true if this action plan has failed. False otherwise.
     */
    bool hasFailed() const;


private:

    /**
     * Creates a DOMDocument object that will contain the DOM tree
     * representing the XML message to be sent to the PVP.
     *
     * @return The DOMDocument just created.
     */
    XERCES_CPP_NAMESPACE::DOMDocument* createPetaverseXMLDocument() const throw (opencog::XMLException, std::bad_exception);

    /**
     * Create a action-plan element in the DOMDocument XML document. An
     * action-plan element has the form:
     *
     *             <action-plan id="" pet-id="">
     *
     * @param doc The XML document where the element will be inserted.
     * @param parent The parent element where the action-plan element will created inside.
     * @param petId The id of the pet
     *
     * @return The DOMElement just created.
     */
    XERCES_CPP_NAMESPACE::DOMElement* createActionPlanElement(XERCES_CPP_NAMESPACE::DOMDocument* doc,
            XERCES_CPP_NAMESPACE::DOMElement* parent,
            const std::string& petId) const;

};

} // namespace

#endif // ACTIONPLAN_H_ 
