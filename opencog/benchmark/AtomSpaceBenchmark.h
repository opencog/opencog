#ifndef _OPENCOG_AS_BENCHMARK_H
#define _OPENCOG_AS_BENCHMARK_H

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/util/mt19937ar.h>

namespace opencog
{

class AtomSpaceBenchmark
{
    typedef std::pair<size_t,clock_t> size_by_time;

    struct TimeStats {
        clock_t t_max;
        clock_t t_min;
        clock_t t_mean;
        clock_t t_std;
        long t_N;
        TimeStats(const std::vector<size_by_time> records);
    };

    void dumpToCSV(std::string filename, std::vector<size_by_time> records) const;

    float linkSize_mean;
    float linkSize_std;

    Type defaultLinkType;
    float chanceOfNonDefaultLink;
    Type defaultNodeType;
    float chanceOfNonDefaultNode;

    float percentLinks;
    float atomCount; //! number of nodes to build atomspace with before testing
    float maxSize; //! never make the atomspace bigger than this while building it

    AtomSpace a;
    MT19937RandGen* rng;

    Type randomType(Type t);

    clock_t makeRandomNode(const std::string& s);
    clock_t makeRandomLink();

    int counter;

public:
    AtomSpaceBenchmark();
    ~AtomSpaceBenchmark();

    void startBenchmark(int numThreads=1);

    void buildAtomSpace(long atomspaceSize=(1 << 16), float percentLinks = 0.1);

    void bm_addNode(int N);
    void bm_addLink(int N, int arity = 0);
    void bm_getHandleSet(int N);


    void bm_getType(int N);
    void bm_getHandleNode(int N);
    void bm_getHandleLink(int N);
    void bm_getName(int N);

    // Get and set TV and AV
    void bm_TruthValue(int N);
    void bm_TV(int N);

    void bm_erase();

};

} // namespace opencog

#endif //_OPENCOG_AS_BENCHMARK_H
