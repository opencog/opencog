/*
 * opencog/embodiment/Control/Procedure/ComboSelectProcedureRepository.h
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

#ifndef COMBO_SELECT_PROCEDURE_REPOSITORY_H
#define COMBO_SELECT_PROCEDURE_REPOSITORY_H

#include "ComboSelectProcedure.h"
#include "ComboProcedureRepository.h"

#include <opencog/persist/file/SavableRepository.h>

#include <string>
#include <map>
#include <boost/noncopyable.hpp>

namespace opencog { namespace Procedure {

typedef std::map<std::string, ComboSelectProcedure> Name2ProcedureMap;
typedef std::map<std::string, ComboSelectProcedure>::const_iterator Name2ProcedureMapIterator;

//noncopyable because combo_trees may point at procedures in the repository
class ComboSelectProcedureRepository : public SavableRepository , public boost::noncopyable
{

public:

    // creates a ComboSelectProcedureRepository. The comboRepository used as
    // argument MUST be the same global comboRepository. It is used mostly
    // to save and load first and second scripts
    ComboSelectProcedureRepository(ComboProcedureRepository& );

    // inform if the repository contains a ComboSelectProcedure with the
    // given name
    bool contains(const std::string& name) const;

    // get the ComboSelectProcedure
    const ComboSelectProcedure& get(const std::string& name) ;

    // add a ComboSelectProcedure to repository.
    void add(const ComboSelectProcedure& procedure);

    // remove a ComboSelectProcedure from repository. The first and second
    // ComboProcedures stored within the ComboRepository are not removed
    // since they can be reused.
    void remove(const std::string& name);

    // return a reference to the ComboRepository
    ComboProcedureRepository& getComboRepository();

    unsigned int loadFromStream(std::istream& in);

    // Methods from SavableRepository interface
    const char* getId() const;
    void saveRepository(FILE*) const;
    void loadRepository(FILE*, HandleMap<Atom *>*);
    void clear();

private:

    Name2ProcedureMap procedureMap;
    ComboProcedureRepository& comboRepository;

}; // class

}} // ~namespace opencog::Procedure

#endif
