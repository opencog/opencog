/*
 * moses-perf.cc
 *
 * Measure moses performance.
 *
 * This program measures performance by repeating a calculation multiple times,
 * but with different random seeds, and then computes the aveage runtime.
 *
 * This program takes the same arguments as the moses-exec executable.
 *
 * Caveat: this measures elapsed wall-clock time, and not cpu time.
 *
 * Linas Vepstas January 2012
 */

#include <math.h>
#include <sys/time.h>
#include "../main/moses_exec.h"

using namespace std;
using namespace opencog::moses;

void measure(vector<string> arguments)
{
    struct timeval total;
    total.tv_sec = 0;
    total.tv_usec = 0;

    double tsq = 0.0; // for computing variance

    int nreps = 20;

    printf("Will run %d repetitions with different random seeds\n", nreps);
    fflush (stdout);

    for (int i=0; i<nreps; i++)
    {
        // Each run gets a new random seed (use the -r option for this).
        vector<string> args = arguments;
        stringstream ss;
#if 0
        ss << "-r" << i;
#else
        int nrep = 10000*(1<<i);
        ss << "-m" << nrep;
#endif
        args.push_back(ss.str());

#if 1
        // Do NOT include a blank space after -f !!
        stringstream fss;
        // fss << "-fmoses-perf-k5-r" << i << ".log";
        fss << "-fmoses-perf-bank-nn29-m" << nrep << ".log";
        args.push_back(fss.str());
#endif

        // Run and measure.
        struct timeval start, stop, elapsed;
        gettimeofday(&start, NULL);
        moses_exec(args);
        gettimeofday(&stop, NULL);
        timersub(&stop, &start, &elapsed);

        timeradd(&total, &elapsed, &total);
        printf("Run %d Time %ld.%06ld seconds\n", i, elapsed.tv_sec, elapsed.tv_usec);
        fflush (stdout);

        // We also want the variance...
        tsq += elapsed.tv_sec * elapsed.tv_sec;
        tsq += 1.0e-6 * ((double) 2 * elapsed.tv_sec * elapsed.tv_usec);
        tsq += 1.0e-12 * ((double) elapsed.tv_usec * elapsed.tv_usec);
    }
    printf("Total time %ld.%06ld seconds\n", total.tv_sec, total.tv_usec);

    time_t avg_sec = total.tv_sec / nreps;
    suseconds_t avg_usec = total.tv_sec - nreps * avg_sec;
    avg_usec *= 1000000;
    avg_usec += total.tv_usec;
    avg_usec /= nreps;

    printf("Caution: average and variance are meaningless if time is log distributed!\n");
    printf("Average time %ld.%06ld seconds\n", avg_sec, avg_usec);

    // Now compute the rms.
    double avg = avg_sec + 1.0e-6 *((double) avg_usec);
    tsq /= nreps;
    tsq -= avg*avg;
    tsq = sqrt(tsq);

    printf("RMS time: %f seconds\n", tsq);
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
