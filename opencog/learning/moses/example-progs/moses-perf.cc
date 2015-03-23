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
#include <sstream>
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

    int nstart = 0;
    int nreps = 10;

#define DO_SEEDS 1
#if DO_THREADING
    double baseline = 0;
    nstart = 1;
    nreps = 16;
    printf("Will run test with 1 to %d threads\n", nreps);
    fflush (stdout);

#elif DO_SEEDS
    printf("Will run %d repetitions with different random seeds\n", nreps);
    fflush (stdout);
#elif DO_REPS
    printf("Will run increasing cutoffs for %d reps\n", nreps);
    fflush (stdout);
#endif

    for (int i=nstart; i<nstart+nreps; i++)
    {
        // Each run gets a new random seed (use the -r option for this).
        vector<string> args = arguments;
        stringstream ss;
#if DO_THREADING
        ss << "-j" << i;
#elif DO_SEEDS
        ss << "-r" << i;
#elif DO_REPS
        int nrep = 10000*(1<<i);
        ss << "-m" << nrep;
#endif
        args.push_back(ss.str());

#if 1
        // Do NOT include a blank space after -f !!
        stringstream fss;
        // fss << "-fmoses-perf-pa-k4-occam-r" << i << ".log";
        // fss << "-fmoses-perf-bank-rev7131-m" << nrep << ".log";
        // fss << "-fmoses-perf-iris-r" << i << ".log";
        // fss << "-fmoses-perf-magic-bi-m" << i << ".log";
        // fss << "-fmoses-perf-yeast-m" << i << ".log";
        fss << "-fmoses-perf-mixed-probe-r" << i << ".log";
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
#if DO_THREADING
        double felapsed = elapsed.tv_sec + elapsed.tv_usec / 1000000.0;
        if (i==1) baseline = felapsed;
        double normed = felapsed * (double) i;
        double slowdown = 100.0 * (normed-baseline) / baseline;
        printf("Run %d Normalized Time %f seconds slowdown=%f percent\n", i, normed, slowdown);
        if (1 < i) {
            double speedup = baseline / felapsed;
            double parallel = (1.0 - 1.0/speedup) / (1.0 - 1.0/double(i));
            double max_speed = 1.0 / (1.0-parallel);
            printf("speedup = %f parallelizable fraction = %f max speedup = %f\n",
                speedup, parallel, max_speed);
                
        }
#endif
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
