
/*
 * opencog/embodiment/Control/PerceptionActionInterface/EventResponder.cc
 *
 * Copyright (C) 2011 OpenCog Foundation
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Shujing ke (rainkekekeke@gmail.com)

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
/** EventResponder.cc
 *
 *  Do responses to the external events .
 */
#include <ostream>
#include "EventResponder.h"

using namespace opencog::pai;
using namespace opencog::control;
using namespace opencog;

EventResponder* EventResponder::instance = NULL;

EventResponder* EventResponder::getInstance()
{
    return EventResponder::instance;
}

EventResponder::EventResponder(PAI& _pai , AtomSpace& _atomSpace):
    pai(_pai),atomSpace(_atomSpace)
{
    // EventResponder is allowed only one in an oac
    if (EventResponder::instance != NULL)
    {
        logger().info("There is already an EventResponder, you cannot create another one!\n");
        return;
    }
    // register all the functions process the corresponding parameters
    paraFuncMap["force"] = &opencog::pai::EventResponder::processForce;

    EventResponder::instance = this;

}

EventResponder::~EventResponder()
{

}

void EventResponder::destroy()
{
    if (EventResponder::instance != NULL)
        delete EventResponder::instance;
}

// TODO: it now can only solve one single action, not exactly an event.
// We expect we can response to an real event in future.
void EventResponder::response(std::string actionName, Handle instanceNode, Handle actorNode, Handle targetNode,std::vector<Handle> actionparams, unsigned long timestamp)
{
    // process all the parameters in this action
    ActionParametersprocess(actionName,instanceNode,actorNode, targetNode,actionparams);

    // execute the scheme scripts to do response

#if HAVE_GUILE

    // Initialize scheme evaluator
    SchemeEval* evaluator = new SchemeEval(&atomSpace);
    std::string scheme_expression, scheme_return_value;

    unsigned long handleInt = instanceNode.value();
    std::ostringstream expression;
    expression << "(apply_stimulus_rule " << handleInt << " )";

    scheme_expression = expression.str();

    logger().debug("EventResponder::response scheme_expression = %s",scheme_expression.c_str());
    // Run the Procedure that update Demands and get the updated value
    scheme_return_value = evaluator->eval(scheme_expression);

    if ( evaluator->eval_error() ) {
        logger().error( "EventResponder::response - Failed to execute scheme function '%s'",
                         scheme_expression.c_str()
                      );
    }

    // reuse the string stream.
    expression.clear();
    expression.str("");
    expression << "(do_attitude_processing " << timestamp << " )";
    scheme_expression = expression.str();

    logger().debug("EventResponder::response process attitudes. scheme_expression = %s",scheme_expression.c_str());
    scheme_return_value = evaluator->eval(scheme_expression);

    if ( evaluator->eval_error() ) {
        logger().error( "EventResponder::response - Failed to execute scheme function '%s'",
                         scheme_expression.c_str()
                      );
    }
    delete evaluator;

#endif // HAVE_GUILE
}

// capsule all the parameters this action carries into the format the responder rules use
// and add them into Atomspa
void EventResponder::ActionParametersprocess(std::string actionName, Handle instanceNode, Handle actorNode, Handle targetNode, std::vector<Handle> actionparams)
{
     //assert(&atomSpace != NULL);

     // travel the parameter vector
     std::string paraName, parakind;
     Handle evalLink, predicateNode,paraNode;
     std::vector<Handle>::iterator iter = actionparams.begin();
     std::map<std::string, ParaFunc_Ptr>::iterator mapiter;
     ParaFunc_Ptr paraFun;
     for (; iter != actionparams.end(); iter ++)
     {
         // Get the parameter kind:
         // Because all the parameter of a specific ation are writen in the format of actionName:parameterName, such as kick:force
         // We just get the parameter name after ":" , e.g. we get "force" from "kick:force"

         // the handle is an EVALUATION_LINK
         evalLink = (Handle)(*iter);
         assert(atomSpace.getType(evalLink) == EVALUATION_LINK );
         predicateNode = atomSpace.getOutgoing( evalLink, 0);
         paraName = atomSpace.getName(predicateNode);
         int index = paraName.find_first_of(":");
         assert(index != -1);
         parakind = paraName.substr(index + 1, paraName.size() - index - 1);

         mapiter = paraFuncMap.find(parakind);

         if (mapiter != paraFuncMap.end())
         {
             // if we have a process function to process this parameter, we process it with the function
             paraFun = (*mapiter).second;

             // replace the evallink with the new evallink
             paraNode = (EventResponder::instance->*paraFun)(actionName,actorNode,targetNode,evalLink);

             if (paraNode != Handle::UNDEFINED)
             {
                 HandleSeq predicateListLinkOutgoing;
                 predicateListLinkOutgoing.push_back(instanceNode);
                 predicateListLinkOutgoing.push_back(paraNode);
                 Handle predicateListLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, predicateListLinkOutgoing);
                 HandleSeq evalLinkOutgoing;
                 evalLinkOutgoing.push_back(predicateNode);
                 evalLinkOutgoing.push_back(predicateListLink);
                 evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);
             }
         }

     }

}

// process the parameter: force , return a concept node that indicates the degree of force:
// extremely_high, high, medium, low, extremely_low
Handle EventResponder::processForce(std::string actionName, Handle actorNode, Handle targetNode, Handle evalLink)
{
    // get the force value
    Handle listLink = atomSpace.getOutgoing( evalLink, 1);
    Handle forceNode = atomSpace.getOutgoing(listLink,1);

    std::string forceStr = atomSpace.getName( forceNode);
    double force = atof(forceStr.c_str());

    // it's a touch action
    if (actionName == "touch")
    {

        double weight = pai.getAvatarWeight(targetNode);

        double normalizedForce = force / weight / 50.0;
        if (normalizedForce < 0.5)
            return AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, EXTREMELY_LOW);
        else if (normalizedForce < 0.75)
            return AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, LOW);
        else if (normalizedForce < 1.0)
            return AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, MEDIUM);
        else if (normalizedForce < 1.5)
            return AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, HIGH);
        else
            return AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, EXTREMELY_HIGH);


    }

    return Handle::UNDEFINED;

}
