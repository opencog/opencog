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
        moses_exec(arguments);
        gettimeofday(&stop, NULL);
        timersub(&stop, &start, &elapsed);

        timeradd(&total, &elapsed, &total);
        cout << "Run "<< i << " Time: "
             << elapsed.tv_sec <<" " << elapsed.tv_usec << endl;
    }
    cout << "Total time: "
         << total.tv_sec <<" secs " << total.tv_usec << " usecs" << endl;

    time_t avg_secs = total.tv_sec / nreps;
    suseconds_t avg_usecs = total.tv_sec - nreps * avg_secs;
    avg_usecs *= 1000000;
    avg_usecs += total.tv_usec;
    avg_usecs /= nreps;

    cout << "Average time: "
         << avg_secs <<" secs " << avg_usecs << " usecs" << endl;
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
