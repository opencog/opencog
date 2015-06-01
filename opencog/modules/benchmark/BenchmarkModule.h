/*
 * opencog/benchmark/BenchmarkModule.h
 *
 * Copyright (C) 2014 Cosmo Harrigan
 * All Rights Reserved
 *
 * Written by Cosmo Harrigan
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

#ifndef _OPENCOG_BENCHMARK_MODULE_H
#define _OPENCOG_BENCHMARK_MODULE_H

#include <string>

#include <opencog/server/Module.h>
#include <opencog/server/CogServer.h>

namespace opencog
{

class CogServer;

/**
 *
 **/
class BenchmarkModule;
typedef std::shared_ptr<BenchmarkModule> BenchmarkModulePtr;

class BenchmarkModule : public Module
{
    private:
        AtomSpace* as;

        /*
         * Runs a performance benchmark on the AtomSpace by creating a fully
         * connected graph with bidirectional directed edges
         * (requires n^2 - n edges).
         *
         * Invoked from the CogServer shell. Syntax:
         *   benchmark-fully-connected OPTION COUNT THREADS
         *
         * where OPTION is 'concurrent', or 'reset', COUNT is an integer
         * number of nodes, and THREADS is an integer number of threads.
         * If no arguments are specified, defaults to:
         *   concurrent 500 2
         * indicating multithreaded execution with 500 nodes and 2 threads.
         *
         * The 'concurrent' option will use a multithreaded version of for_each
         * from the GNU libstdc++ parallel mode OpenMP library
         */
        DECLARE_CMD_REQUEST(BenchmarkModule, "benchmark-fully-connected",
           do_fullyConnectedTest,
           "Run a simple AtomSpace performance test",
           "Usage: benchmark-fully-connected",
           false, false)
        int fullyConnectedTestConcurrent(int);

        /*
         * Performs multithreaded setSTI operations on all atoms
         */
        int updateSTITestConcurrent(void);

    public:
        BenchmarkModule(CogServer&);
        virtual ~BenchmarkModule();
        virtual void run();

        static const char *id(void);
        virtual void init(void);
};

}

#endif
