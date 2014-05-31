/*
 * opencog/benchmark/BenchmarkModule.cc
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

#include <future>
#include <iostream>
#include <iomanip>
#include <thread>
#include <time.h>

#include <boost/range/irange.hpp>

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/dynamics/attention/atom_types.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>
#include <opencog/util/Config.h>
#include <opencog/util/oc_omp.h>

#include "BenchmarkModule.h"

using namespace std;
using namespace opencog;

DECLARE_MODULE(BenchmarkModule)

BenchmarkModule::BenchmarkModule(CogServer& cs) : Module(cs)
{
    logger().info("[BenchmarkModule] constructor");
    this->as = &cs.getAtomSpace();

    do_fullyConnectedTest_register();
}

void BenchmarkModule::init(void)
{
    logger().info("Initializing BenchmarkModule.");
}

void BenchmarkModule::run()
{
}

BenchmarkModule::~BenchmarkModule()
{
    logger().info("Terminating BenchmarkModule.");

    do_fullyConnectedTest_unregister();
}

int BenchmarkModule::fullyConnectedTestConcurrent(int numAtoms)
{
    // Multithreaded using GNU libstdc++ parallel mode
    // Add numAtoms ConceptNodes
    auto ir = boost::irange((size_t)0, (size_t)numAtoms);
    vector<size_t> indices(ir.begin(), ir.end());

    OMP_ALGO::for_each(indices.begin(), indices.end(), [this](int index)
    {
        as->addPrefixedNode(CONCEPT_NODE, "test_atom_");
    });

    // Retrieve all the ConceptNodes into an iterator
    HandleSeq atoms;
    as->getHandlesByType(back_inserter(atoms), CONCEPT_NODE);

    // Create a fully connected graph between them with bidirectional directed
    // edges: requires n^2 - n edges
    OMP_ALGO::for_each(atoms.begin(), atoms.end(),
    [&atoms, this](Handle handleSource)
    {
        for_each(atoms.begin(), atoms.end(),
            [&handleSource, this](Handle handleTarget)
        {
            as->addLink(ASYMMETRIC_HEBBIAN_LINK, handleSource, handleTarget, 
                        SimpleTruthValue::createTV(0, 1));
        });
    });

    return atoms.size();
}

int BenchmarkModule::updateSTITestConcurrent()
{
    srand(time(NULL));

    HandleSeq atoms;
    as->getHandlesByType(back_inserter(atoms), ATOM, true);

    OMP_ALGO::for_each(atoms.begin(), atoms.end(),
        [this](Handle handle)
    {
        int newSTI = rand() % 1000;
        as->setSTI(handle, newSTI);
    });

    return atoms.size();
}

std::string
BenchmarkModule::do_fullyConnectedTest(Request *dummy,
                                       std::list<std::string> args)
{
    std::vector<std::string> argv{ std::begin(args), std::end(args) };

    std::string option = argv.size() > 0 ? argv[0] : "concurrent";

    int numNewNodes;
    try
    {
        numNewNodes = argv.size() > 1 ? std::stoi(argv[1]) : 500;
    }
    catch(std::invalid_argument& e)
    {
        return "Error: argument COUNT must be an integer.\n";
    }

    int numThreads;
    try
    {
        // User can specify the number of threads. Otherwise, defaults to 2
        numThreads = argv.size() > 2 ? std::stoi(argv[2]) : 2;
    }
    catch(std::invalid_argument& e)
    {
        return "Error: argument THREADS must be an integer.\n";
    }
    // Set the number of threads for OpenMP parallelization
    setting_omp(numThreads);

    const clock_t begin_time_cpu = clock();
    const time_t begin_time_wall = time(NULL);

    int numNodes;
    std::string result;

    if (option == "reset")
    {
        // Delete all ConceptNodes and their incoming sets
        HandleSeq atoms;
        as->getHandlesByType(back_inserter(atoms), CONCEPT_NODE);

        OMP_ALGO::for_each(atoms.begin(), atoms.end(),
            [this](Handle handle)
        {
            as->removeAtom(handle, true);
        });

        return "All ConceptNodes and their incoming sets deleted.\n";
    }
    else if (option == "concurrent")
    {
        numNodes = fullyConnectedTestConcurrent(numNewNodes);
        result = "Fully connected graph of " +
                std::to_string(numNodes) +
                " total nodes including " +
                std::to_string(numNewNodes) +
                " new nodes.\n";
    }
    else if (option == "sti")
    {
        int numAtoms = updateSTITestConcurrent();
        result = std::to_string(numAtoms) +
                " atoms updated with random STI values.\n";
    }
    else
    {
        return "Error, unrecognized argument. Usage:\n"
               "  benchmark-fully-connected OPTION COUNT THREADS\n"
               "where OPTION is 'concurrent', 'reset' or 'sti', COUNT is an "
               "integer number of nodes,\nand THREADS is an integer number of "
               "threads. If no arguments are specified,\ndefaults to:\n"
               "  concurrent 500 2\n";
    }

    const clock_t end_time_cpu = clock();
    const time_t end_time_wall = time(NULL);

    std::string message;
    message = result +
            "Wall clock time: " +
            std::to_string(end_time_wall - begin_time_wall) +
            " seconds\nCPU clock time: " +
            std::to_string((end_time_cpu - begin_time_cpu) / CLOCKS_PER_SEC) +
            " seconds\nNumber of threads used: " +
            std::to_string(numThreads) + "\n";
    return message;
}
