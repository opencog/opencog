/*
 * opencog/embodiment/Control/Procedure/ComboProcedureRepository.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi, Nil Geisweiller
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

#ifndef COMBO_PROCEDURE_REPOSITORY_H
#define COMBO_PROCEDURE_REPOSITORY_H

#include <string>
#include <map>
#include <boost/noncopyable.hpp>

#include <opencog/persist/file/SavableRepository.h>
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/procedure_repository.h>
#include "ComboProcedure.h"


namespace opencog { namespace Procedure {

//noncopyable because combo_trees may point at procedures in the repository
class ComboProcedureRepository : public combo::procedure_repository, public SavableRepository, public boost::noncopyable
{

    //private:
    //typedef std::map<std::string, ComboProcedure> Name2ProcedureMap;
    //Name2ProcedureMap _map;

public:
    //the location of the standard library
    static const std::string stdlibPath;

    ComboProcedureRepository();

    //parse and load procedures from a stream - returns # of procedures loaded
    unsigned int loadFromStream(std::istream& in);

    bool contains(const std::string& name) const;
    const ComboProcedure& get(const std::string& name) const;
    void add(const ComboProcedure& cp);

    //set procedure calls to point to the procedures stored in this repository
    void instantiateProcedureCalls(combo::combo_tree& tr, bool warnOnDefiniteObj = false) const;

    // Methods from SavableRepository interface
    const char* getId() const;
    void saveRepository(FILE*) const;
    void loadRepository(FILE*, HandMapPtr);
    void clear();
};

}} // ~namespace opencog::Procedure

#endif
