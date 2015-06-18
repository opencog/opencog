/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiModulatorUpdaterAgent.cc
 *
 * @author Jinhua Chua <JinhuaChua@gmail.com>
 * @date   2011-11-22
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

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/octime.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>

#include <lib/json_spirit/json_spirit.h>

#include "PsiModulatorUpdaterAgent.h"

using namespace opencog;

bool PsiModulatorUpdaterAgent::Modulator::runUpdater (AtomSpace & atomSpace)
{
    std::string modulatorUpdater = this->modulatorName + "ModulatorUpdater";

#if HAVE_GUILE

    // Initialize scheme evaluator
    SchemeEval evaluator1(&atomSpace);
    std::string scheme_expression, scheme_return_value;

    scheme_expression = "( " + modulatorUpdater + " )";

    // Run the Procedure that update Modulators and get the updated value
    scheme_return_value = evaluator1.eval(scheme_expression);

    if ( evaluator1.eval_error() ) {
        logger().error( "PsiModulatorUpdaterAgent::Modulator::%s - Failed to execute '%s'",
                         __FUNCTION__,
                         scheme_expression.c_str()
                      );

        return false;
    }

    // Store the updated modulator value (result)
    this->currentModulatorValue = atof( scheme_return_value.c_str() );

    logger().debug("PsiModulatorUpdaterAgent::Modulator::%s - The level of modulator '%s' will be set to '%f'",
                   __FUNCTION__,
                   this->modulatorName.c_str(),
                   this->currentModulatorValue
                  );

#endif // HAVE_GUILE

    return true;
}

bool PsiModulatorUpdaterAgent::Modulator::updateModulator (AtomSpace & atomSpace, const octime_t timeStamp)
{
    // Update LatestLink containig latest modulator level
    std::string predicateName = this->modulatorName + "Modulator";
    Handle modulatorPredicateNode = atomSpace.addNode(PREDICATE_NODE, predicateName.c_str());
    TruthValuePtr stv = SimpleTruthValue::createTV(this->currentModulatorValue, 1);

    std::vector <Handle> outgoings;
    Handle listLink = atomSpace.addLink(LIST_LINK, outgoings);
    outgoings.push_back(modulatorPredicateNode);
    outgoings.push_back(listLink);
    Handle evaluationLink = atomSpace.addLink(EVALUATION_LINK, outgoings);
    atomSpace.setTV(evaluationLink, stv);

//    Handle evaluationLink = AtomSpaceUtil::setPredicateValue(atomSpace, predicateName, stv);
    TimeServer timeServer(atomSpace);
    Handle atTimeLink = timeServer.addTimeInfo(evaluationLink,
        timeStamp, stv);

    AtomSpaceUtil::updateLatestModulator(atomSpace, atTimeLink, modulatorPredicateNode);

#if HAVE_GUILE

    // Initialize scheme evaluator
    SchemeEval evaluator1(&atomSpace);
    std::string scheme_expression, scheme_return_value;

    // Store the updated Modulator levels to AtomSpace
    // set_modulator_or_demand_value would create a new NumberNode and SimilarityLink
    //
    // Note: Since OpenCog would forget (remove) those Nodes and Links gradually,
    //       unless you create them to be permanent, don't worry about the overflow of memory.
    scheme_expression = "( set_modulator_or_demand_value \"" +
                               this->modulatorName + "Modulator\" " +
                               boost::lexical_cast<std::string>(this->currentModulatorValue) + " " +
                               boost::lexical_cast<std::string>(timeStamp) + " " +
                         ")";

    // Run the scheme procedure
    scheme_return_value = evaluator1.eval(scheme_expression);

    if ( evaluator1.eval_error() ) {
        logger().error( "PsiModulatorUpdaterAgent::Modulator::%s - Failed to execute '%s'",
                         __FUNCTION__,
                         scheme_expression.c_str()
                      );

        return false;
    }

    logger().debug("PsiModulatorUpdaterAgent::Modulator::%s - Updated the value of '%s' modulator to %f and store it to AtomSpace",
                   __FUNCTION__,
                   this->modulatorName.c_str(),
                   this->currentModulatorValue
                  );

    return true;

#else // HAVE_GUILE
    logger().error( "PsiModulatorUpdaterAgent::Modulator::%s - Guile is required",
                    __FUNCTION__);
    return false;
#endif // HAVE_GUILE
}

PsiModulatorUpdaterAgent::~PsiModulatorUpdaterAgent()
{
    logger().info("[PsiModulatorUpdaterAgent] destructor");
}

PsiModulatorUpdaterAgent::PsiModulatorUpdaterAgent(CogServer& cs) : Agent(cs)
{
    this->cycleCount = 0;

    // This is needed for time stamping
    initReferenceTime();

    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

void PsiModulatorUpdaterAgent::init()
{
    logger().debug( "PsiModulatorUpdaterAgent::%s - Initialize the Agent [ cycle = %d ]",
                    __FUNCTION__,
                    this->cycleCount
                  );

    // Clear old modulator list
    this->modulatorList.clear();

    // Get modulator names from the configuration file
    std::string modulatorNames = config()["PSI_MODULATORS"];

    // Process Modulators one by one
    boost::char_separator<char> sep(", ");
    boost::tokenizer< boost::char_separator<char> > modulatorNamesTok (modulatorNames, sep);

    std::string modulatorName, modulatorUpdater;

    for ( boost::tokenizer< boost::char_separator<char> >::iterator iModulatorName = modulatorNamesTok.begin();
          iModulatorName != modulatorNamesTok.end();
          iModulatorName ++ ) {

        modulatorName = (*iModulatorName);
        modulatorUpdater = modulatorName + "ModulatorUpdater";

        this->modulatorList.push_back(Modulator(modulatorName));

        logger().debug("PsiModulatorUpdaterAgent::%s - Store the meta data of modulator '%s' successfully [cycle = %d]",
                        __FUNCTION__,
                        modulatorName.c_str(),
                        this->cycleCount);
    }// for

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiModulatorUpdaterAgent::run()
{
    this->cycleCount ++;

    logger().debug( "PsiModulatorUpdaterAgent::%s - Executing run %d times",
                     __FUNCTION__, this->cycleCount);

    // Get AtomSpace
    AtomSpace& atomSpace = _cogserver.getAtomSpace();

    // Get current time stamp
    // TODO: Should use the TimeServer class for making time series analysis
    octime_t timeStamp = getElapsedMillis();

    // Initialize the Agent (modulatorMetaMap etc)
    if ( !this->bInitialized )
        this->init();

    // Run modulator updaters
    for (Modulator & modulator : this->modulatorList) {
        modulator.runUpdater(atomSpace);
    }

    // Set the updated value to AtomSpace
    for (Modulator & modulator : this->modulatorList) {
        modulator.updateModulator(atomSpace, timeStamp);
    }

#if HAVE_GUILE
    // Initialize scheme evaluator
    SchemeEval evaluator1(&atomSpace);

    std::string scheme_expression, scheme_return_value;

    scheme_expression = "( get_pleasure_value )";

    // Run the scheme procedure
    scheme_return_value = evaluator1.eval(scheme_expression);

    if ( evaluator1.eval_error() ) {
        logger().error( "PsiModulatorUpdaterAgent::Modulator::%s - Failed to execute '%s'",
                         __FUNCTION__,
                         scheme_expression.c_str()
                      );
    }

    this->pleasure = atof( scheme_return_value.c_str() );
#endif // HAVE_GUILE
}
