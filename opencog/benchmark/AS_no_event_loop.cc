
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "AtomSpaceBenchmark.h"


int main(int argc, char** argv) {

    const char* benchmark_desc = "Benchmark of OpenCog AtomSpace\n"
     "-l <float> \tSet the connection probability or coordination number\n"
     "-n <int> \tSet how many thousands of nodes are created\n";

    double percent_links = 0.1;
    long node_count = 1 << 16;
    int c;
    opterr = 0;
    while ((c = getopt (argc, argv, "l:n:")) != -1) {
       switch (c)
       {
           case 'l':
             percent_links = atof(optarg);
             break;
           case 'n':
             node_count = 1000 * (long) atoi(optarg);
             break;
           case '?':
             fprintf (stderr, "%s", benchmark_desc);
             return 0;
           default:
             fprintf (stderr, "Unknown option %c ", optopt);
             abort ();
       }
    }

    opencog::AtomSpaceBenchmark benchmarker;
    benchmarker.startBenchmark();

}
