/*
 * opencog/embodiment/Control/Procedure/ProcedureRepository.h
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

#ifndef PROCEDUREREPOSITORY_H_
#define PROCEDUREREPOSITORY_H_

#include <opencog/persist/file/SavableRepository.h>
#include "ComboProcedureRepository.h"
#include "ComboSelectProcedureRepository.h"
#include "BuiltInProcedureRepository.h"

#include <string>
#include <map>

namespace opencog { namespace Procedure {

class ProcedureRepository : public SavableRepository
{

private:
    ComboProcedureRepository comboRepository;
    BuiltInProcedureRepository& builtInRepository;
    ComboSelectProcedureRepository * comboSelectRepository;

public:
    ProcedureRepository(pai::PAI&);
    virtual ~ProcedureRepository();
    bool contains(const std::string& name) const;
    void remove(const std::string& name); //remove a combo procedure_call
    const GeneralProcedure& get(const std::string& name) const;
    void add(const ComboProcedure& cp);
    void add(const ComboSelectProcedure& cp);

    const ComboProcedureRepository& getComboRepository() const;
    const ComboSelectProcedureRepository& getComboSelectRepository() const;
    const BuiltInProcedureRepository& getBuiltInRepository() const;

    // Methods from SavableRepository interface
    const char* getId() const;
    void saveRepository(FILE*) const;

    int loadComboFromStream(std::istream& in);
    int loadComboSelectFromStream(std::istream& in);

    void loadRepository(FILE*, HandleMap<Atom *>*);
    void clear();
};

}} // ~namespace opencog::Procedure

#endif /*PROCEDUREREPOSITORY_H_*/
