/*
 * opencog/embodiment/Control/Procedure/ComboSelectProcedureRepository.cc
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

#include "ComboSelectProcedureRepository.h"
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

#include <stdio.h>
#include <opencog/comboreduct/combo/procedure_call.h>

using namespace PetCombo;
using namespace Procedure;

ComboSelectProcedureRepository::ComboSelectProcedureRepository(ComboProcedureRepository& _r) : comboRepository(_r)
{
}

bool ComboSelectProcedureRepository::contains(const std::string& name) const
{
    return (procedureMap.find(name) != procedureMap.end());
}

const ComboSelectProcedure& ComboSelectProcedureRepository::get(const std::string& name)
{
    if (!contains(name)) {
        throw opencog::RuntimeException(TRACE_INFO, "ComboSelectProcedureRepository - No store procedure with name '%s'.", name.c_str());
    }
    return this->procedureMap[name];
}

void  ComboSelectProcedureRepository::add(const ComboSelectProcedure& procedure)
{
    this->procedureMap[procedure.getName()] = procedure;

    if (!this->comboRepository.contains(procedure.getFirstScriptName())) {
        logger().debug(
                     "ComboSelectProcedureRepository - Adding script to comboRepo: '%s'.",
                     procedure.getFirstScriptName().c_str());

        this->comboRepository.add(procedure.getFirstScript());
    }

    if (!this->comboRepository.contains(procedure.getSecondScriptName())) {
        logger().debug(
                     "ComboSelectProcedureRepository - Adding script to comboRepo: '%s'.",
                     procedure.getSecondScriptName().c_str());

        this->comboRepository.add(procedure.getSecondScript());
    }
}

void  ComboSelectProcedureRepository::remove(const std::string& name)
{
    this->procedureMap.erase(name);
}

ComboProcedureRepository& ComboSelectProcedureRepository::getComboRepository()
{
    return this->comboRepository;
}

unsigned int ComboSelectProcedureRepository::loadFromStream(std::istream& in)
{
    unsigned int n = 0;

    while (in.good()) {
        while (in.peek() == ' ' || in.peek() == '\n' || in.peek() == '\t')
            in.get();

        if (in.peek() == '#') { //a comment line
            char tmp[LINE_CHAR_MAX];
            in.getline(tmp, LINE_CHAR_MAX);
            continue;
        }

        if (!in.good()) {
            break;
        }

        std::string tmp;
        std::string str;
        int nparen = 0;

        //place name(arity) in str
        do {
            in >> tmp;
            nparen += count(tmp.begin(), tmp.end(), '(') - count(tmp.begin(), tmp.end(), ')');
            str += tmp + ' ';
            tmp.assign("");

        } while (in.good() && nparen > 0);

        if (nparen != 0 || !in.good()) {
            logger().debug(
                         "ComboSelectProcedureRepository - nparen error '%d' for '%s'.", nparen, str.c_str());
            return 0;
        }

        //affect arity and name
        std::string::size_type lparen = str.find('(');
        std::string::size_type rparen = str.find(')');
        if (lparen == std::string::npos || rparen == std::string::npos || lparen > rparen) {
            logger().debug(
                         "ComboSelectProcedureRepository - parentesis mismatch for '%s'.", str.c_str());
            return 0;
        }

        std::string name = str.substr(0, lparen);
        unsigned int arity;
        try {
            arity = boost::lexical_cast<unsigned int>(str.substr(lparen + 1, rparen - lparen - 1));
        } catch (...) {
            logger().debug(
                         "ComboSelectProcedureRepository - Cannot get arity for '%s'.", str.c_str());
            return 0;
        }

        //recognize {
        in >> tmp;
        if (tmp != "|=" || !in.good()) {
            logger().debug(
                         "ComboSelectProcedureRepository - Found no '|=' symbol for '%s'.", str.c_str());
            return 0;
        }

        combo::procedure_call pc1 = load_procedure_call < avatar_builtin_action, avatar_perception,
                                    avatar_action_symbol, avatar_indefinite_object > (in, false);

        combo::procedure_call pc2 = load_procedure_call < avatar_builtin_action, avatar_perception,
                                    avatar_action_symbol, avatar_indefinite_object > (in, false);

//        if (!in.good()){
//            break;
//        }

        if (pc1 && pc2) {
            ComboProcedure * firstScript = new ComboProcedure(*pc1);
            ComboProcedure * secondScript = new ComboProcedure(*pc2);

            comboRepository.add(*firstScript);
            comboRepository.add(*secondScript);

            delete firstScript;
            delete secondScript;

            add(ComboSelectProcedure(name,
                                     this->comboRepository.get(pc1->get_name()),
                                     this->comboRepository.get(pc2->get_name())));

            logger().fine(
                         "ComboSelectProcedureRepository - Loaded combo scripts f: '%s' arity '%d' and s: '%s' arity '%d'.",
                         pc1->get_name().c_str(), pc1->arity(), pc2->get_name().c_str(), pc2->arity());

            n++;

        } else {
            logger().error(
                         "ComboSelectProcedureRepository - Error parsing combo function.");
        }
        delete(pc1);
        delete(pc2);
    }

    return n;
}

// Methods from SavableRepository interface
const char*  ComboSelectProcedureRepository::getId() const
{
    return "ComboSelectProcedureRepository";
}

void  ComboSelectProcedureRepository::saveRepository(FILE* dump) const
{
    logger().debug("ComboSelectProcedureRepository - Saving %s (%ld)", getId(), ftell(dump));

    fprintf(dump, "%zu", procedureMap.size());

    Name2ProcedureMapIterator it;
    for (it = procedureMap.begin(); it != procedureMap.end(); it++) {

        const std::string &name = it->first;
        int nameLength = name.length();
        fwrite(&nameLength, sizeof(int), 1, dump);
        fwrite(name.c_str(), sizeof(char), nameLength + 1, dump);

        const std::string &firstScriptName = it->second.getFirstScriptName();
        int firstScriptNameLength = firstScriptName.length();
        fwrite(&firstScriptNameLength, sizeof(int), 1, dump);
        fwrite(firstScriptName.c_str(), sizeof(char), firstScriptNameLength + 1, dump);

        const std::string &secondScriptName = it->second.getSecondScriptName();
        int secondScriptNameLength = secondScriptName.length();
        fwrite(&secondScriptNameLength, sizeof(int), 1, dump);
        fwrite(secondScriptName.c_str(), sizeof(char), secondScriptNameLength + 1, dump);
    }
}

void  ComboSelectProcedureRepository::loadRepository(FILE* dump, HandleMap<Atom *>* conv)
{

    logger().debug("ComboSelectProcedureRepository - Loading %s (%ld)", getId(), ftell(dump));

    int size;
    char buffer[1<<16];
    fscanf(dump, "%d", &size);

    for (int i = 0; i < size; i++) {
        // get the procedure name
        int nameLength;
        fread(&nameLength, sizeof(int), 1, dump);
        fread(buffer, sizeof(char), nameLength + 1, dump);
        std::string name(buffer);

        fread(&nameLength, sizeof(int), 1, dump);
        fread(buffer, sizeof(char), nameLength + 1, dump);
        std::string firstScriptName(buffer);

        fread(&nameLength, sizeof(int), 1, dump);
        fread(buffer, sizeof(char), nameLength + 1, dump);
        std::string secondScriptName(buffer);

        if (!this->comboRepository.contains(firstScriptName)) {
            throw opencog::RuntimeException(TRACE_INFO,
                                            "ComboSelectProcedureRepository - No procedure for '%s' in ComboProcedureRepository.",
                                            firstScriptName.c_str());
        }

        if (!this->comboRepository.contains(secondScriptName)) {
            throw opencog::RuntimeException(TRACE_INFO,
                                            "ComboSelectProcedureRepository - No procedure for '%s' in ComboProcedureRepository.",
                                            secondScriptName.c_str());
        }

        add(ComboSelectProcedure(name,
                                 this->comboRepository.get(firstScriptName),
                                 this->comboRepository.get(secondScriptName)));
    }
}

void  ComboSelectProcedureRepository::clear()
{
    this->procedureMap.clear();
}

