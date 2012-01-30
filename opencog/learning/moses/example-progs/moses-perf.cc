/*
 * moses-perf.cc
 *
 * Measure moses performance.
 *
 * This program measures performance by repeating a caluclation multiple times,
 * but with different random seeds, and then computes the aveage runtime.
 *
 * This program takes the same arguments as the moses-exec executable.
 *
 * Linas Vepstas January 2012
 */

#include <sys/time.h>
#include "../main/moses_exec.h"

using namespace std;
using namespace opencog::moses;

void measure(vector<string> arguments)
{
    struct timeval total;
    total.tv_sec = 0;
    total.tv_usec = 0;

    int nreps = 10;

    printf("Will run %d repetitions with different random seeds\n", nreps);

    for (int i=0; i<nreps; i++)
    {
        // Each run gets a new random seed (use the -r option for this).
        vector<string> args = arguments;
        stringstream ss;
        ss << "-r" << i;
        args.push_back(ss.str());

        // Run and measure.
        struct timeval start, stop, elapsed;
        gettimeofday(&start, NULL);
        moses_exec(args);
        gettimeofday(&stop, NULL);
        timersub(&stop, &start, &elapsed);

        timeradd(&total, &elapsed, &total);
        printf("Run %d Time %ld.%06ld seconds\n", i, elapsed.tv_sec, elapsed.tv_usec);
    }
    printf("Total time %ld.%06ld seconds\n", total.tv_sec, total.tv_usec);

    time_t avg_sec = total.tv_sec / nreps;
    suseconds_t avg_usec = total.tv_sec - nreps * avg_sec;
    avg_usec *= 1000000;
    avg_usec += total.tv_usec;
    avg_usec /= nreps;

    printf("Average time %ld.%06ld seconds\n", avg_sec, avg_usec);
}

int main(int argc, char *argv[])
{
    vector<string> arguments;
    arguments.push_back("moses-exec");
    for (int i=1; i<argc; i++)
        arguments.push_back(argv[i]);

    measure(arguments);

    return 0;
}
