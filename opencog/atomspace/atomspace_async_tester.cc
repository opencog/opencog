#include "AtomSpaceAsync.h"
#include "ClassServer.h"
#include <opencog/util/mt19937ar.h>

#include <boost/thread.hpp>
#include <ctime>

using namespace opencog;
using namespace std;

int n_threads = 5;
int N = 1000000;
MT19937RandGen* rng;
AtomSpaceAsync* as;

Type randomType(Type t)
{
    int numberOfTypes = classserver().getNumberOfClasses();
    Type randomType = NOTYPE;
    while (!classserver().isA(randomType, t))
        randomType = ATOM + rng->randint(numberOfTypes-1);
    return randomType;
}

void producer(int thread_i) {
    for (int i = 0; i< N/n_threads; i++ ) {
        Type t = randomType(NODE);
        as->getImpl().addNode(t,"test");
        as->getImpl().getAtomTable().getHandle(t,"test");
    }
    cout << "Done sending requests from thread " << thread_i << "." << endl;
}


int main(int argc, char** argv) {
    AtomSpaceAsync a;
    rng = new opencog::MT19937RandGen((unsigned long) time(NULL));

    const char* benchmark_desc = "Test for concurrent queue with locks\n"
     "-t <int> \tSet the number of threads sending reqeuest\n"
     "-n <int> \tSet how many requests are sent\n";
    int c;
    opterr = 0;
    while ((c = getopt (argc, argv, "t:n:")) != -1) {
       switch (c)
       {
           case 't':
             n_threads = (long) atoi(optarg);
             break;
           case 'n':
             N = (long) atoi(optarg);
             break;
           case '?':
             fprintf (stderr, "%s", benchmark_desc);
             return 0;
           default:
             fprintf (stderr, "Unknown option %c ", optopt);
             abort ();
       }
    }
    
    as = &a;
    clock_t tStart = clock();
    boost::thread_group thread_pool;
    for (int i=0; i < n_threads-1; i++) {
        boost::thread *t = new boost::thread(producer, i);
        thread_pool.add_thread(t);
    }
    
    for (int i = 0; i< N/n_threads; i++ ) {
        Type t = randomType(NODE);
        a.getImpl().addNode(t,"test");
        a.getImpl().getAtomTable().getHandle(t,"test");
    }
    cout << "Done sending requests from main thread" << endl;
    /*for (int i = 0; i< N/n_threads; i++ ) {
        assert (hrs[i]->get_result() == Handle(i));
    }
    cout << "Done getting results and checking requests" << endl;*/

    thread_pool.join_all();
    while (!a.isQueueEmpty()) usleep(10);
    clock_t tEnd = clock();
    cout << "Counter is " << a.get_counter() << endl;
    cout << "Time taken is " << (double)(tEnd - tStart)/CLOCKS_PER_SEC;
    cout << " - " << ((double)(tEnd - tStart)/CLOCKS_PER_SEC) / (N*2) << " per request." << endl;

}
