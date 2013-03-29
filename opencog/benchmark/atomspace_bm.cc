
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "AtomSpaceBenchmark.h"


int main(int argc, char** argv) {

    const char* benchmark_desc = "Benchmark tool OpenCog AtomSpace\n"
     "Usage: atomspace_bm [-m <method>] [options]\n"
     "-t        \tPrint information on type sizes\n"
     "-A        \tBenchmark all methods\n"
     "-x        \tTest the AtomSpaceImpl backend instead of the public AtomSpace API\n"
     "-X        \tTest the AtomTable instead of the public AtomSpace API\n"
     "-m <methodname>\tMethod to benchmark\n" 
     "-l        \tList valid method names to benchmark\n"
     "-n <int>  \tHow many times to call the method in the measurement loop\n" 
     "          \t(default: 100000)\n"
     "-S <int>  \tHow many random atoms to add after each measurement\n"
     "          \t(default: 0)\n"
     "-- Build test data --\n"
     "-b        \tBefore benchmark, build a graph of random nodes/links\n"
     "-p <float> \tSet the connection probability or coordination number\n"
     "         \t(default: 0.2)\n"
     "         \t(-p impact behaviour of -S too)\n"
     "-s <int> \tSet how many atoms are created (default: 65536)\n"
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
    while ((c = getopt (argc, argv, "tAxXm:ln:S:bp:s:d:kfi:")) != -1) {
       switch (c)
       {
           case 't':
             benchmarker.showTypeSizes = true;
             break;
           case 'A':
             benchmarker.setMethod("addNode");
             benchmarker.setMethod("addLink");
             benchmarker.setMethod("getType");
             benchmarker.setMethod("getTV");
//             benchmarker.setMethod("getTVZmq");
             benchmarker.setMethod("setTV");
             benchmarker.setMethod("getHandleSet");
             benchmarker.setMethod("getNodeHandles");
             benchmarker.setMethod("getOutgoingSet");
             break;
           case 'x':
             benchmarker.testBackend = true;
             break;
           case 'X':
             benchmarker.testTable = true;
             break;
           case 'm':
             benchmarker.setMethod(optarg);
             break;
           case 'l':
             benchmarker.showMethods();
             exit(0);
             break;
           case 'n':
             benchmarker.Nreps = atoi(optarg);
             break;
           case 'S':
             benchmarker.sizeIncrease = atoi(optarg);
             break;
           case 'b':
             benchmarker.buildTestData = true;
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

    benchmarker.startBenchmark();

}
