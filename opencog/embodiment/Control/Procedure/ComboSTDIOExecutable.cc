/*
 * opencog/embodiment/Control/Procedure/ComboSTDIOExecutable.cc
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

#include "ComboInterpreter.h"
#include "ComboProcedureRepository.h"

#include <opencog/embodiment/WorldWrapper/ShellWorldWrapper.h>
#include <opencog/embodiment/PetComboVocabulary/PetComboVocabulary.h>
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/util/mt19937ar.h>
#include <fstream>
#include <iostream>


int main(int argc, char** argv)
{
    using namespace std;
    using namespace PetCombo;
    using namespace Procedure;

    //instantiate a Logger that does not print messages to not interfere with
    //standard IO
    //opencog::Logger* NoIOLogger = new opencog::Logger();
    opencog::logger().setPrintToStdoutFlag(false);
    //opencog::Logger::initMainLogger(NoIOLogger);

    ComboProcedureRepository cpr;

    if (argc > 2) {
        cerr << "usage: " << argv[0] << " [function-file]" << endl;
        cerr << "some trivial sample functions are in scripts/funcs.combo" << endl;
        exit(1);
    } else if (argc == 2) {
        ifstream fin(argv[1]);
        int n = cpr.loadFromStream(fin);
        std::cout << "loaded " << n << " combo functions from " << argv[1] << std::endl;
    }

    opencog::MT19937RandGen rng(0);

    combo_tree tr;
    world::ShellWorldWrapper sww;
    while (cin.good()) {
        cout << "> ";
        if (cin.peek() == ' ' ||
                cin.peek() == '\n' ||
                cin.peek() == '\t')
            cin.get();
        while (cin.peek() == ' ' ||
                cin.peek() == '\n' ||
                cin.peek() == '\t') {
            if (cin.peek() == '\n')
                cout << "> ";
            cin.get();
        }
        if (cin.peek() == '#') { //a comment line
            char tmp[1024];
            cin.getline(tmp, 1024);
            continue;
        }
        cin >> tr;
        if (!cin.good())
            break;
        cout << "running " << tr << endl;
        try {
            cpr.instantiateProcedureCalls(tr, true);
            std::vector<combo::vertex> empty_arguments;
            RunningComboProcedure rp(sww, tr, rng, empty_arguments);
            while (!rp.isFinished()) {
                rp.cycle();
            }
            if (rp.isFailed()) {
                cout << "execution failed!" << endl;
            } else {
                if (rp.getResult() != id::action_success)
                    cout << "result: " << rp.getResult() << endl;
                cout << "execution succeeded!" << endl;
            }
        } catch (...) {
            cout << "execution failed (threw exception)" << endl;
        }
    }
    cout << endl;
    return 0;
}
