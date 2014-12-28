/*
 * opencog/embodiment/Control/Procedure/ComboProcedureRepository.cc
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

#include "ComboProcedureRepository.h"
#include <fstream>

#include <opencog/util/Logger.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/util/Config.h>
#include <opencog/util/macros.h>

#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>

using namespace AvatarCombo;
using namespace std;

namespace opencog { namespace Procedure {

ComboProcedureRepository::ComboProcedureRepository()
{

}

//parse and load procedures from a stream - returns # of functions read
unsigned int ComboProcedureRepository::loadFromStream(istream& in)
{

    bool tc = config().get_bool("TYPE_CHECK_LOADING_PROCEDURES");

    unsigned int n = 0;

    while (in.good()) {
        while (in.peek() == ' ' || in.peek() == '\n' || in.peek() == '\t')
            in.get();

        if (in.peek() == '#') { //a comment line
            char tmp[LINE_CHAR_MAX];
            in.getline(tmp, LINE_CHAR_MAX);
            continue;
        }

        procedure_call pc = load_procedure_call < avatar_builtin_action, avatar_perception,
                            avatar_action_symbol, avatar_indefinite_object > (in, false);

        if (!in.good()) {
            break;
        }

        if (pc) {
            ComboProcedure* cp = new ComboProcedure(*pc);
            add(*cp);
            ++n;
            logger().fine(
                         "ComboProcedureRepository - Loaded '%s' with arity '%d'.",
                         pc->get_name().c_str(), pc->arity());

        } else {
            logger().error(
                         "ComboProcedureRepository - Error parsing combo function.");
        }
        delete(pc);
    }
    //doing the resolution and type checking here
    //allows mutual recursion amongst functions to be defined in the input
    instantiate_procedure_calls(true);
    if (tc) {
        bool type_check_success = infer_types_repo();
        if (!type_check_success) {
            stringstream ss;
            toStream(ss, true);
            print(true);
            OC_ASSERT(type_check_success,
                             "Type Checking Error, one or more function are ill formed. See the list of function and there types :\n%s", ss.str().c_str());
        }
    }
    return n;
}

bool ComboProcedureRepository::contains(const std::string& name) const
{
    return does_contain(name);
}

const ComboProcedure& ComboProcedureRepository::get(const std::string& name) const
{
    procedure_call pc = instance(name);
    OC_ASSERT(pc,
                     "No ComboProcedure matches that name, you cannot get it");
    return dynamic_cast<const ComboProcedure&>(*pc);
}

void ComboProcedureRepository::add(const ComboProcedure& cp)
{
    combo::combo_tree newTr = cp.getComboTree();
    instantiateProcedureCalls(newTr, true);
    ComboProcedure* newCp = new ComboProcedure(cp.getName(),
            cp.getArity(),
            newTr);
    procedure_repository::add(const_cast<procedure_call_base*>(dynamic_cast<procedure_call>(newCp)));
}

void ComboProcedureRepository::instantiateProcedureCalls(combo::combo_tree& tr, bool warnOnDefiniteObj) const
{
    procedure_repository::instantiate_procedure_calls(tr, warnOnDefiniteObj);
}

const char* ComboProcedureRepository::getId() const
{
    return "ComboProcedureRepository";
}

void ComboProcedureRepository::saveRepository(FILE* dump) const
{
    logger().debug("Saving %s (%ld)\n", getId(), ftell(dump));
    fprintf(dump, "%zu", _repo.size());
    for (str_proc_map_const_it itr = _repo.begin(); itr != _repo.end(); ++itr) {
        const string &name = itr->first;
        int nameLength = name.length();
        fwrite(&nameLength, sizeof(int), 1, dump);
        fwrite(name.c_str(), sizeof(char), nameLength + 1, dump);
        int arity = itr->second->arity();
        fwrite(&arity, sizeof(int), 1, dump);
        string treeStr = toString(itr->second->get_body());
        logger().fine("name: %s\ntree: %s\n", name.c_str(), treeStr.c_str());
        int treeStrLength = treeStr.length();
        fwrite(&treeStrLength, sizeof(int), 1, dump);
        fwrite(treeStr.c_str(), sizeof(char), treeStrLength + 1, dump);
    }
}

void ComboProcedureRepository::loadRepository(FILE* dump, HandMapPtr)
{
    logger().debug("Loading %s (%ld)\n", getId(), ftell(dump));
    char buffer[1<<16];
    bool tc = config().get_bool("TYPE_CHECK_LOADING_PROCEDURES");

    int size;
    bool b_read = (fscanf(dump, "%d", &size) > 0);
    for (int i = 0; i < size; i++) {
        // get the name
        int nameLength;
        FREAD_CK(&nameLength, sizeof(int), 1, dump);
        FREAD_CK(buffer, sizeof(char), nameLength + 1, dump);
        string name(buffer);
        int arity;
        FREAD_CK(&arity, sizeof(int), 1, dump);
        // get the combo_tree
        int treeStrLength;
        FREAD_CK(&treeStrLength, sizeof(int), 1, dump);
        FREAD_CK(buffer, sizeof(char), treeStrLength + 1, dump);
        logger().fine("name: %s\ntree: %s\n", name.c_str(), buffer);
        std::stringstream ss(buffer);
        combo_tree tr;
        ss >> tr;
        // add the new entry
        add(ComboProcedure(name, arity, tr));
    }
    CHECK_FREAD;
    //doing the resolution here allows for mutual recursion amongst functions defined in the input
    instantiate_procedure_calls(true);
    // TODO: if combo_tree contains Handles, they must be replaced by the new ones, according with the passed handleMap

    //perform type checking
    if (tc) {
        bool type_check_success = infer_types_repo();
        if (!type_check_success) {
            stringstream ss;
            toStream(ss, true);
            print(true);
            OC_ASSERT(type_check_success,
                             "Type Checking Error, one or more function are ill formed. See the list of function and there types :\n%s", ss.str().c_str());
        }
    }
}

void ComboProcedureRepository::clear()
{
    procedure_repository::clear();
}

}} // namespace opencog::Procedure
