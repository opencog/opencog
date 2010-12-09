#include "AtomSpaceBenchmark.h"

#include <opencog/atomspace/types.h>
#include <ctime>

#include <iostream>
#include <fstream>

namespace opencog {

using std::cout;
using std::cerr;
using std::flush;
using std::endl;
using std::clock;
using std::time;

AtomSpaceBenchmark::AtomSpaceBenchmark()
{
    percentLinks = 0.2;
    atomCount = (1 << 16);
    defaultNodeType = CONCEPT_NODE;
    chanceOfNonDefaultNode = 0.4f;
    defaultLinkType = INHERITANCE_LINK;
    chanceOfNonDefaultLink = 0.4f;
    linkSize_mean = 2.0f;
    linkSize_std = 0.5f;
    counter = 0;

    rng = new opencog::MT19937RandGen((unsigned long) time(NULL));

}
AtomSpaceBenchmark::~AtomSpaceBenchmark() {}

void AtomSpaceBenchmark::startBenchmark(int numThreads)
{
    bm_addNode(100000);
    bm_addLink(10000);
}

Type AtomSpaceBenchmark::randomType(Type t)
{
    int numberOfTypes = classserver().getNumberOfClasses();
    OC_ASSERT(t < numberOfTypes);
    Type randomType = NOTYPE;
    while (!classserver().isA(randomType, t))
        randomType = ATOM + rng->randint(numberOfTypes-1);
    return randomType;
}

clock_t AtomSpaceBenchmark::makeRandomNode(const std::string& s) {
    double p = rng->randdouble();
    Type t = defaultNodeType;
    if (p < chanceOfNonDefaultNode)
        t=randomType(NODE);
    if (s.size() > 0) {
        clock_t t_begin = clock();
        a.addNode(t,s); 
        return clock() - t_begin;
    } else {
        std::ostringstream oss;
        counter++;
        oss << "node " << counter;
        clock_t t_begin = clock();
        a.addNode(t,oss.str()); 
        return clock() - t_begin;
    }
}

clock_t AtomSpaceBenchmark::makeRandomLink() {
    Type t = defaultLinkType;
    double p = rng->randdouble();
    HandleSeq outgoing;
    //clock_t tRandomStart, tRandomEnd;
    clock_t tAddLinkStart;
    if (p < chanceOfNonDefaultLink) t=randomType(LINK);

    int arity = rng->pos_gaussian_rand(linkSize_std, linkSize_std);
    if (arity==0) { ++arity; };

    for (int j=0; j < arity; j++) {
        //tRandomStart = clock();
        // We need this TLB access as the only alternative to
        // getting a random handle this way scales badly:
        //Handle h = a.getAtomTable().getRandom(rng);
        Handle h(rng->randint(TLB::getMaxUUID()-2)+1);
        //tRandomEnd = clock();
        outgoing.push_back(h);
    }
    tAddLinkStart = clock();
    a.addLink(t,outgoing);
    return clock() - tAddLinkStart;
}

void AtomSpaceBenchmark::buildAtomSpace(long atomspaceSize, float _percentLinks)
{
    clock_t tStart = clock();
    cout << "Building atomspace with " << atomCount << " atoms (" <<
        percentLinks*100.0 << "\% links)" << endl;
    
    // Add nodes
    long nodeCount = atomCount * (1.0f - percentLinks);
    int i;
    for (i=0; i<nodeCount; i++) {
        makeRandomNode("");
        if (i % 10000 == 0) cerr << "." << flush;
    }
    cout << endl << "Finished adding " << nodeCount << " nodes..." << endl;

    // Add links
    for (; i < atomCount; i++) {
        makeRandomLink();
        if ((i-nodeCount) % 1000 == 0) {
            //printf("random select: %.5fs, ",
            //         (double)(tRandomEnd - tRandomStart)/CLOCKS_PER_SEC);
            //printf("addlink: %.5fs\n",
            //         (double)(tAddLinkEnd - tAddLinkStart)/CLOCKS_PER_SEC);
            cerr << "." << flush;
        }
    }
    cout << endl << "Finished adding " << atomCount - nodeCount << " links..." << endl;

    printf("Built atomspace, execution time: %.2fs\n",
         (double)(clock() - tStart)/CLOCKS_PER_SEC);

}

void AtomSpaceBenchmark::bm_addNode(int N)
{
    std::vector<size_by_time> records;
    for (int i = 0; i < N; i++) {
        size_t atomspaceSize = a.getSize();
        clock_t timeTaken = makeRandomNode("");
        size_by_time dataPoint(atomspaceSize,timeTaken);
        records.push_back(dataPoint);
        if (i % 1000 == 0) cerr << "." << flush;
    }
    AtomSpaceBenchmark::TimeStats t(records);
    cout << endl << "mean " << t.t_mean << endl;
    cout << "min " << t.t_min << endl;
    cout << "max " << t.t_max << endl;
    cout << "std " << t.t_std << endl;
    dumpToCSV("bm_addNode.csv", records);
}

void AtomSpaceBenchmark::bm_addLink(int N, int arity)
{
    std::vector<size_by_time> records;
    for (int i = 0; i < N; i++) {
        size_t atomspaceSize = a.getSize();
        clock_t timeTaken = makeRandomLink();
        size_by_time dataPoint(atomspaceSize,timeTaken);
        records.push_back(dataPoint);
        if (i % 1000 == 0) cerr << "." << flush;
    }
    AtomSpaceBenchmark::TimeStats t(records);
    cout << endl << "mean " << t.t_mean << endl;
    cout << "min " << t.t_min << endl;
    cout << "max " << t.t_max << endl;
    cout << "std " << t.t_std << endl;
    dumpToCSV("bm_addLink.csv", records);
}


AtomSpaceBenchmark::TimeStats::TimeStats(
        const std::vector<size_by_time> records) {
    double sum;
    t_min = 1 << 31;
    t_max = 0;
    foreach (size_by_time record, records) {
        sum += record.second;
        if (record.second > t_max) t_max = record.second;
        else if (record.second < t_min) t_min = record.second;
    }
    t_N = records.size();
    t_mean = sum / t_N;
    sum = 0.0;
    foreach (size_by_time record, records) {
        clock_t value = (record.second - t_mean);
        sum += (value*value);
    }
    t_std = sqrt(sum/(t_N-1));
}

void AtomSpaceBenchmark::dumpToCSV(std::string filename,
        std::vector<size_by_time> records) const
{
    std::ofstream myfile;
    myfile.open (filename.c_str());
    foreach (size_by_time record, records) {
        myfile << record.first << "," << record.second << endl;
    }
    myfile.close();
}

}

