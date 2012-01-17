/*
 * opencog/embodiment/Control/Procedure/ProcedureRepository.cc
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

#include "ProcedureRepository.h"

namespace opencog { namespace Procedure {

ProcedureRepository::ProcedureRepository(pai::PAI& pai) :
        builtInRepository(*(new BuiltInProcedureRepository(pai)))
{

    // combo select repository needs a ComboRepository to store
    // ComboSelectProcedure first and second combo scripts
    this->comboSelectRepository = new ComboSelectProcedureRepository(comboRepository);
}

ProcedureRepository::~ProcedureRepository()
{
    delete &builtInRepository;
    delete comboSelectRepository;
}

bool ProcedureRepository::contains(const std::string& name) const
{
    return comboRepository.contains(name)   ||
           builtInRepository.contains(name) ||
           comboSelectRepository->contains(name);
}

void ProcedureRepository::remove(const std::string& name)
{
    if (comboRepository.contains(name)) {
        comboRepository.remove(name);
    } else if (comboSelectRepository->contains(name)) {
        comboSelectRepository->remove(name);
    }
}

const GeneralProcedure& ProcedureRepository::get(const std::string& name) const
{
    if (comboRepository.contains(name)) {
        return comboRepository.get(name);
    } else if (comboSelectRepository->contains(name)) {
        return comboSelectRepository->get(name);
    } else {
        return builtInRepository.get(name);
    }
}

void ProcedureRepository::add(const ComboProcedure& cp)
{
    comboRepository.add(cp);
}

void ProcedureRepository::add(const ComboSelectProcedure& cp)
{
    comboSelectRepository->add(cp);
}

const ComboProcedureRepository& ProcedureRepository::getComboRepository() const
{
    return comboRepository;
}

const ComboSelectProcedureRepository& ProcedureRepository::getComboSelectRepository() const
{
    return *comboSelectRepository;
}

const BuiltInProcedureRepository& ProcedureRepository::getBuiltInRepository() const
{
    return builtInRepository;
}

const char* ProcedureRepository::getId() const
{
    return "ProcedureRepository";
}

void ProcedureRepository::saveRepository(FILE* dump) const
{
    comboRepository.saveRepository(dump);
    comboSelectRepository->saveRepository(dump);
}

void ProcedureRepository::loadRepository(FILE* dump, HandleMap<Atom *>* conv)
{
    comboRepository.loadRepository(dump, conv);
    comboSelectRepository->loadRepository(dump, conv);
}

int ProcedureRepository::loadComboFromStream(std::istream& in)
{
    return comboRepository.loadFromStream(in);
}

int ProcedureRepository::loadComboSelectFromStream(std::istream& in)
{
    return comboSelectRepository->loadFromStream(in);
}


void ProcedureRepository::clear()
{
    comboRepository.clear();
    comboSelectRepository->clear();
}

}} // ~namespace opencog::Procedure
