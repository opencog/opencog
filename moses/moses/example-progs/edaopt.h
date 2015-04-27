/*
 * moses/moses/main/edaopt.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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

#include <assert.h>
#include <iostream>
#include <vector>
#include <boost/lexical_cast.hpp>

using namespace std;

// WARNING: the additional arguments must be handled by the caller,
// but it is done here for checking and usage printing.
// This is an ugly but quick fix for the demo programs.
struct optargs
{
    optargs(int argc, char** argv,
            const vector<string>& additional_args = vector<string>()) { 
        if (argc != (5 + static_cast<int>(additional_args.size()))) {
            cerr << "Error: wrong  number of args.\n"
                 << "Usage: " << argv[0] 
                 << " <rand seed> <length> <population size> <num generations> "
                 << usage(additional_args) << endl;
            exit(1);
        }
        try {
            assert(argc >= 5);
            rand_seed = boost::lexical_cast<int>(argv[1]);
            length = boost::lexical_cast<int>(argv[2]);
            popsize = boost::lexical_cast<int>(argv[3]);
            n_select = popsize;
            n_generate = popsize/2;
            max_gens = boost::lexical_cast<int>(argv[4]);
        } catch (...) {
            cerr << "Error: invalid argument\nUsage: " << argv[0]
                 << " <rand seed> <length> <population size> <num generations> "
                 << usage(additional_args) << endl;
            exit(1);
        }
    }
    int rand_seed;
    int length;
    int popsize;
    int n_select;
    int n_generate;
    int max_gens;

private:
    string usage(const vector<string>& args)
    {
        string res;
        for (vector<string>::const_iterator i = args.begin();
            i != args.end(); ++i) {
            res += *i + string(" ");
        }
        return res;
    }
};
