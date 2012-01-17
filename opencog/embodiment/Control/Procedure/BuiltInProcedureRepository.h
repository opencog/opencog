/*
 * opencog/embodiment/Control/Procedure/BuiltInProcedureRepository.h
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

#ifndef BUILTIN_PROCEDURE_REPOSITORY_H
#define BUILTIN_PROCEDURE_REPOSITORY_H

#include <string>
#include <map>

#include "BuiltInProcedure.h"
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

namespace opencog { namespace Procedure {

class BuiltInProcedureRepository
{

private:
    typedef std::map<std::string, BuiltInProcedure*> Name2ProcedureMap;
    Name2ProcedureMap _map;

    void add(BuiltInProcedure* proc);

public:
    BuiltInProcedureRepository(pai::PAI&);
    ~BuiltInProcedureRepository();

    bool contains(const std::string& name) const;
    const BuiltInProcedure& get(const std::string& name) const;

    /// Update the variable contents of the schemata
    bool update(AtomSpace& atomspace);
};

}} // ~namespace opencog::Procedure

#endif
