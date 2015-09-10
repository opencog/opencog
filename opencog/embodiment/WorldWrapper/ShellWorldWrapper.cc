/*
 * opencog/embodiment/WorldWrapper/ShellWorldWrapper.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller, Moshe Looks
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


#include "ShellWorldWrapper.h"
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

namespace opencog { namespace world {

using namespace AvatarCombo;

/**
 * Constructor, destructor
 */
ShellWorldWrapper::ShellWorldWrapper() : _isFailed(false),
        _isFinished(true) {}

ShellWorldWrapper::~ShellWorldWrapper() {}

/**
 * public methods
 */

bool ShellWorldWrapper::isPlanFinished() const
{
    return _isFinished;
}

bool ShellWorldWrapper::isPlanFailed() const
{
    return _isFailed;
}

bool ShellWorldWrapper::sendSequential_and(sib_it from, sib_it to)
{
    using namespace AvatarCombo;
    combo_tree tr(id::sequential_and);
    pre_it head = tr.begin();
    for (sib_it sib = from; sib != to; ++sib) {
        tr.replace(tr.append_child(head), sib);
    }
    std::cout << "What should the result of sending plan "
              << tr << "be (action_success or action_failure)?"
              << std::endl;
    std::cout << "> ";
    vertex v;
    std::cin >> v;
    _isFailed = v != id::action_success;
    _isFinished = true;
    return true;
}

combo::vertex ShellWorldWrapper::evalPerception(pre_it per, combo::variable_unifier& vu)
{
    using namespace combo;
    std::cout << "What should the result of "
              << combo_tree(per) << "be (true or false)?" << std::endl;
    std::cout << "> ";
    vertex v;
    std::cin >> v;
    return v;
}

combo::vertex ShellWorldWrapper::evalIndefiniteObject(combo::indefinite_object io, combo::variable_unifier& vu)
{
    using namespace combo;
    std::cout << "What should " << io << " return?" << std::endl;
    std::cout << "> ";
    vertex v;
    std::cin >> v;
    return v;
}

} } // namespace opencog::world

