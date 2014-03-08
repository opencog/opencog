
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "AtomSpaceBenchmark.h"

using namespace std;

int main(int argc, char** argv)
{
    const char* benchmark_desc = "Benchmark tool OpenCog AtomSpace\n"
     "Usage: atomspace_bm [-m <method>] [options]\n"
     "-t        \tPrint information on type sizes\n"
     "-A        \tBenchmark all methods\n"
     "-x        \tTest the AtomSpaceImpl API\n"
     "-X        \tTest the AtomTable API\n"
     "-g        \tTest the Scheme API\n"
     "-M        \tMemoize Scheme expressions\n"
     "-C        \tCompile Scheme expressions\n"
     "-c        \tTest the Python API\n"
     "-m <methodname>\tMethod to benchmark\n" 
     "-l        \tList valid method names to benchmark\n"
     "-n <int>  \tHow many times to call the method in the measurement loop\n" 
     "          \t(default: 100000)\n"
     "-r <int>  \tLooping count; how many times a python/scheme operation is looped\n"
     "          \t(default: 1)\n"
     "-S <int>  \tHow many random atoms to add after each measurement\n"
     "          \t(default: 0)\n"
     "-- Build test data --\n"
     "-p <float> \tSet the connection probability or coordination number\n"
     "         \t(default: 0.2)\n"
     "         \t(-p impact behaviour of -S too)\n"
     "-s <int> \tSet how many atoms are created (default: 256K)\n"
     "-d <float> \tChance of using default truth value (default: 0.8)\n"
     "-- Saving data --\n"
     "-k       \tCalculate stats (warning, this will affect rss memory reporting)\n"
     "-f       \tSave a csv file with records for every repeated event\n"
     "-i <int> \tSet interval of data to save\n";

    int c;

    if (argc==1) {
        fprintf (stderr, "%s", benchmark_desc);
        return 0;
    }
    opencog::AtomSpaceBenchmark benchmarker;

    opterr = 0;
    benchmarker.testKind = opencog::AtomSpaceBenchmark::BENCH_AS;

    while ((c = getopt (argc, argv, "tAXgMCcm:ln:r:S:p:s:d:kfi:")) != -1) {
       switch (c)
       {
           case 't':
             benchmarker.showTypeSizes = true;
             break;
           case 'A':
             benchmarker.buildTestData = true;
             benchmarker.setMethod("noop");
             benchmarker.setMethod("addNode");
             benchmarker.setMethod("addLink");
             benchmarker.setMethod("getType");
             benchmarker.setMethod("getTV");
//             benchmarker.setMethod("getTVZmq");
             benchmarker.setMethod("setTV");
             benchmarker.setMethod("getNodeHandles");
             benchmarker.setMethod("getOutgoingSet");
             benchmarker.setMethod("getIncomingSet");
             benchmarker.setMethod("removeAtom");
             benchmarker.setMethod("getHandleSet");
             break;
           case 'X':
             benchmarker.testKind = opencog::AtomSpaceBenchmark::BENCH_TABLE;
             break;
           case 'g':
#ifdef HAVE_GUILE
             benchmarker.testKind = opencog::AtomSpaceBenchmark::BENCH_SCM;
#else
             cerr << "Fatal Error: Benchmark not compiled with scheme support!" << endl;
             exit(1);
#endif
             break;
           case 'C':
             benchmarker.compile = true;
             break;
           case 'M':
             benchmarker.memoize = true;
             break;
           case 'c':
#ifdef HAVE_CYTHON
             benchmarker.testKind = opencog::AtomSpaceBenchmark::BENCH_PYTHON;
#else
             cerr << "Fatal Error: Benchmark not compiled with cython support!" << endl;
             exit(1);
#endif
             break;
           case 'm':
             benchmarker.buildTestData = true;
             benchmarker.setMethod(optarg);
             break;
           case 'l':
             benchmarker.showMethods();
             exit(0);
             break;
           case 'n':
             benchmarker.Nreps = (unsigned int) atoi(optarg);
             break;
           case 'r':
             benchmarker.Nloops = (unsigned int) atoi(optarg);
             break;
           case 'S':
             benchmarker.sizeIncrease = atoi(optarg);
             break;
           case 'p':
             benchmarker.percentLinks = atof(optarg);
             break;
           case 's':
             benchmarker.atomCount = (long) atof(optarg);
             break;
           case 'd':
             benchmarker.chanceUseDefaultTV = atof(optarg);
             break;
           case 'k':
             benchmarker.doStats = true;
             break;
           case 'f':
             benchmarker.saveToFile = true;
             break;
           case 'i':
             benchmarker.saveInterval = atoi(optarg);
             break;
           case '?':
             fprintf (stderr, "%s", benchmark_desc);
             return 0;
           default:
             fprintf (stderr, "Unknown option %c ", optopt);
             abort ();
       }
    }

    if (opencog::AtomSpaceBenchmark::BENCH_SCM != benchmarker.testKind)
    {
        if (1 != benchmarker.Nloops)
        {
            cerr << "Error: the python and atomspace tests currently do not support looping\n";
            benchmarker.Nloops = 1;
        }
        if (true == benchmarker.memoize or true == benchmarker.compile)
        {
            cerr << "Fatal Error: Compiling and memoization supported only for the scheme tests\n";
            exit(1);
        }
    }

    if (opencog::AtomSpaceBenchmark::BENCH_SCM == benchmarker.testKind)
    {
        if (true == benchmarker.memoize and true == benchmarker.compile)
        {
            cerr << "Fatal Error: Can compile of memoize, but not both!\n";
            exit(-1);
        }
    }

    benchmarker.startBenchmark();
    return 0;
}
