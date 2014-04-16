/*
 * opencog/embodiment/Control/Procedure/BuiltInProcedure.h
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

#ifndef BUILTINPROCEDURE_H_
#define BUILTINPROCEDURE_H_
/**
 * Base class for BuiltIn Procedures
 * @author Welter Luigi
 *
 */
#include "GeneralProcedure.h"
#include <opencog/comboreduct/combo/vertex.h>
#include <list>

namespace opencog { namespace Procedure {

using namespace combo;
        
class BuiltInProcedure : public GeneralProcedure
{

protected:

    unsigned int minArity;
    unsigned int optionalArity;

public:
    virtual ~BuiltInProcedure() {}

    virtual vertex execute(const std::vector<vertex>& arguments) const = 0;

    ProcedureType getType() const {
        return BUILT_IN;
    }

    /**
     * Indicates if this procedure is an Pet action schemata.
     * If so, its execute method always return the ActionPlanId for the action sent to Proxy
     */
    virtual bool isAvatarAction() const {
        return false;
    }

    /**
     * Return the mandatory arity for the builtin action
     */
    unsigned int getArity() const {
        return minArity;
    }

    /**
     * Return the optional arity for the buitlin action
     */
    unsigned int getOptionalArity() {
        return optionalArity;
    }

    /**
     * Return the max arity (min + optional arities) for the builtin action
     */
    unsigned int getMaxArity() const {
        return (minArity + optionalArity);
    }
};

}} // ~namespace opencog::Procedure

#endif /*BUILTINPROCEDURE_H_*/
