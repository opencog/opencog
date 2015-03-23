/*
 * opencog/spatial/TangentBugTestExec.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Dan Zwell
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

#include <opencog/spatial/LocalSpaceMap2DUtil.h>
#include <opencog/spatial/TangentBug.h>
#include <list>
#include <boost/lexical_cast.hpp>
#include <boost/tuple/tuple.hpp>
#include <opencog/util/mt19937ar.h>

#ifdef TB_PRINT_NCURSES
#include <ncurses.h>
#endif

using namespace std;

using namespace opencog;
using namespace opencog::spatial;

int main(int argc, char * argv[])
{
    int seed;
    if (argc > 1)
        try {
            seed = boost::lexical_cast<int, char*>(argv[1]);
        } catch (boost::bad_lexical_cast &) {
            cerr << "Usage:\n";
            cerr << "If ncurses printing is enabled, you had better redirect stderr:\n";
            cerr << argv[0] << " [integral random seed] 2>log.txt\n";
            cerr << "Otherwise:\n";
            cerr << argv[0] << " [integral random seed]\n";
            exit(1);
        }
    else
        seed = time(NULL);

    cout << "Random seed is " << seed << endl;

    //typedef LocalSpaceMap2D<Handle, double, hashHandle, spatial::ObjMetaData > LSM;
    typedef LocalSpaceMap2D LSM;
    typedef TangentBug TB;

    LSM lsm(500, 999, 200,
            500, 999, 200,
            //5);
            6);

    TB::CalculatedPath action_plan;
    TB tb(lsm, action_plan);
    tb.init_ncurses();

    // Randomly generate a map:
    populateRandom(lsm, 20, tb.getCenter());
    //populateRandom<Handle,double,hashHandle,spatial::ObjMetaData>(lsm, 20, tb.getCenter());
    tb.place_pet_randomly(tb.getCenter());
    tb.place_goal_randomly(tb.getCenter());

    // Just aesthetic, does nothing with NCurses off:
    tb.trace_path(tb.getCenter(), tb.getGoal());
    tb.trace_path(tb.getCenter(), tb.getCurrPos());

    bool success = tb.seek_goal();
    //tb.print_action_plan();
    tb.print();
    //call to getch commented since its implementation is empty, anyway...
    //getch();
    tb.stop_ncurses();
    return (success ? 0 : 1);
}
