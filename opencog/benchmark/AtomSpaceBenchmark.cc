
/** AtomSpaceBenchmark.cc */

#include <ctime>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <sys/resource.h>

#include <boost/tuple/tuple_io.hpp>

#include <opencog/util/oc_assert.h>
#include <opencog/util/random.h>

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/cython/PythonEval.h>
#include <opencog/cython/PythonModule.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/server/CogServer.h>  // Ugh. Needed for python to work right.

#include "AtomSpaceBenchmark.h"

namespace opencog {

using namespace boost;
using std::cout;
using std::cerr;
using std::flush;
using std::endl;
using std::clock;
using std::time;

#define DIVIDER_LINE "------------------------------"
#define PROGRESS_BAR_LENGTH 10

AtomSpaceBenchmark::AtomSpaceBenchmark()
{
    percentLinks = 0.2;

    // Create an atomspace with a quarter-million atoms
    // This is quasi-realistic for an atomspace doing language processing.
    // Maybe a bit on the small side ...
    atomCount = (1 << 18);
    defaultNodeType = CONCEPT_NODE;
    chanceOfNonDefaultNode = 0.4f;
    defaultLinkType = INHERITANCE_LINK;
    chanceOfNonDefaultLink = 0.4f;
    linkSize_mean = 2.0f;
    prg = new std::poisson_distribution<unsigned>(linkSize_mean);

    counter = 0;
    showTypeSizes = false;
    Nreps = 100000;
    Nloops = 1;
    memoize = false;
    compile = false;
    sizeIncrease = 0;
    saveToFile = false;
    saveInterval = 1;
    buildTestData = false;
    chanceUseDefaultTV = 0.8f;
    doStats = false;
    testKind = BENCH_AS;

    asp = NULL;
    atab = NULL;
#if HAVE_CYTHON
    cogs = NULL;
    pymo = NULL;
#endif

    rng = new opencog::MT19937RandGen((unsigned long) time(NULL));
}

AtomSpaceBenchmark::~AtomSpaceBenchmark()
{
    delete prg;
    delete rng;
}

// This is wrong, because it fails to also count the amount of RAM
// used by the AtomTable to store indexes.
size_t AtomSpaceBenchmark::estimateOfAtomSize(Handle h)
{
    size_t total = 0;
    if (h->getTruthValue() != TruthValue::DEFAULT_TV())
    {
        switch (h->getTruthValue()->getType()) {
        case SIMPLE_TRUTH_VALUE:
            total += sizeof(SimpleTruthValue);
            break;
        case COUNT_TRUTH_VALUE:
            total += sizeof(CountTruthValue);
            break;
        case INDEFINITE_TRUTH_VALUE:
            total += sizeof(IndefiniteTruthValue);
            break;
        default:
            break;
        }
    }

    if (h->getAttentionValue() != AttentionValue::DEFAULT_AV())
    {
        total += sizeof(AttentionValue);
    }

    if (asp->isNode(h))
    {
        NodePtr n(NodeCast(h));
        total = sizeof(Node);
        total += n->getName().capacity();
    }
    else
    {
        LinkPtr l(LinkCast(h));
        total = sizeof(Link);
        total += l->getOutgoingSet().capacity() * sizeof(Handle);
        for (Handle ho: l->getOutgoingSet())
        {
            total += estimateOfAtomSize(ho);
        }
    }

    return total;

}

long AtomSpaceBenchmark::getMemUsage()
{
    // getrusage is the best option it seems...
    // on linux /proc/pid/status and other files may have more detail
    struct rusage *s = (struct rusage *) malloc(sizeof(struct rusage));
    getrusage(RUSAGE_SELF,s);
    long rss = s->ru_maxrss;
    free(s);
    return rss;
}

void AtomSpaceBenchmark::printTypeSizes()
{
    // Note that these are just the type size, it doesn't include the size of
    // data/classes that these might point to.
    //cout << "CLOCKS_PER_SEC = " << CLOCKS_PER_SEC << endl;
    cout << "==sizeof() on various classes==" << endl;
    cout << "FIXME: the report below is only for the sizes of the C++ objects\n"
         << "themselves.  In addition, every atom consumes from 5 to 15 times\n"
         << "the sizeof(Handle) in the AtomTable indexes. This depends on the\n"
         << "atom type; Links store more than twice the the sizeof(Handle) per\n"
         << "outgoing atoms.\n";
    cout << "Type = " << sizeof(Type) << endl;
    cout << "Handle = " << sizeof(Handle) << endl;
    cout << "Atom = " << sizeof(Atom) << endl;
    cout << "Node = " << sizeof(Node) << endl;
    cout << "Link = " << sizeof(Link) << endl;
    cout << "SimpleTruthValue = " << sizeof(SimpleTruthValue) << endl;
    cout << "CountTruthValue = " << sizeof(CountTruthValue) << endl;
    cout << "IndefiniteTruthValue = " << sizeof(IndefiniteTruthValue) << endl;
    cout << "AttentionValue = " << sizeof(AttentionValue) << endl;
    cout << "IncomingSet = " << sizeof(IncomingSet) << endl;
    cout << "AtomSignal = " << sizeof(AtomSignal) << endl;
    cout << "AtomPairSignal = " << sizeof(AtomPairSignal) << endl;
    cout << DIVIDER_LINE << endl;

#define ND(T,S) ({Handle n(createNode(T,S)); n;})
#define LK(T,A,B) ({Handle l(createLink(T,A,B)); l;})
    Handle h = ND(CONCEPT_NODE, "this is a test");
    cout << "ConceptNode \"this is a test\" = "
         << estimateOfAtomSize(h) << endl;

    HandleSeq empty;
    h = Handle(createLink(LIST_LINK, empty));
    cout << "Empty ListLink = " << estimateOfAtomSize(h) << endl;

    Handle na = ND(CONCEPT_NODE, "first atom");
    Handle nb = ND(CONCEPT_NODE, "second atom");
    Handle ll = LK(LIST_LINK, na, nb);
    cout << "ListLink with two ConceptNodes = "
         << estimateOfAtomSize(ll) << endl;

    Handle np = ND(PREDICATE_NODE, "some predicate");
    Handle el = LK(EVALUATION_LINK, np, ll);
    cout << "EvaluationLink with two ConceptNodes = "
         << estimateOfAtomSize(el) << endl;
}

void AtomSpaceBenchmark::showMethods() {
    /// @todo should really encapsulate each test method in a struct or class
    cout << "Methods that can be tested:" << endl;
    cout << "  addNode" << endl;
    cout << "  addLink" << endl;
    cout << "  getType" << endl;
    cout << "  getTruthValue" << endl;
    cout << "  setTruthValue" << endl;
    cout << "  getHandleSet" << endl;
    cout << "  getNodeHandles" << endl;
    cout << "  getOutgoingSet" << endl;

}

void AtomSpaceBenchmark::setMethod(std::string _methodName)
{
    if (_methodName == "noop") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_noop);
    } else if (_methodName == "addNode") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_addNode);
    } else if (_methodName == "addLink") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_addLink);
    } else if (_methodName == "removeAtom") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_rmAtom);
    } else if (_methodName == "getType") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_getType);
    } else if (_methodName == "getTV") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_getTruthValue);
#ifdef ZMQ_EXPERIMENT
    } else if (_methodName == "getTVZmq") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_getTruthValueZmq);
#endif
    } else if (_methodName == "setTV") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_setTruthValue);
    } else if (_methodName == "getHandleSet") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_getHandleSet);
    } else if (_methodName == "getNodeHandles") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_getNodeHandles);
    } else if (_methodName == "getOutgoingSet") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_getOutgoingSet);
    } else if (_methodName == "getIncomingSet") {
        methodsToTest.push_back( &AtomSpaceBenchmark::bm_getIncomingSet);
    } else {
        std::cerr << "Error: specified a bad test name: " << _methodName << std::endl;
        exit(1);
    }
    methodNames.push_back( _methodName);
}

#define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(ptrToMember))
void AtomSpaceBenchmark::doBenchmark(const std::string& methodName,
                                     BMFn methodToCall)
{
    clock_t sumAsyncTime = 0;
    long rssStart;
    std::vector<record_t> records;
    cout << "Benchmarking ";
    switch (testKind) {
        case BENCH_AS:  cout << "AtomSpace's "; break;
        case BENCH_TABLE:  cout << "AtomTable's "; break;
#if HAVE_GUILE
        case BENCH_SCM:  cout << "Scheme's ";
        if (memoize) cout << "memoized ";
        else if (compile) cout << "compiled ";
        else cout << "interpreted ";
        break;
#endif /* HAVE_GUILE */
#if HAVE_CYTHON
        case BENCH_PYTHON: cout << "Python's "; break;
#endif /* HAVE_CYTHON */
    }
    cout << methodName << " method " << (Nreps*Nloops) << " times ";
    std::ofstream myfile;
    if (saveToFile)
    {
        myfile.open ((methodName + "_benchmark.csv").c_str());
    }
    int diff = (Nreps / PROGRESS_BAR_LENGTH);
    if (!diff) diff = 1;
    int counter=0;
    rssStart = getMemUsage();
    long rssFromIncrease = 0;
    timeval tim;
    gettimeofday(&tim, NULL);
    double t1 = tim.tv_sec + (tim.tv_usec/1000000.0);
    for (unsigned int i=0; i < Nreps; i++)
    {
        if (sizeIncrease)
        {
            long rssBeforeIncrease = getMemUsage();
            buildAtomSpace(sizeIncrease, percentLinks, false);
            // Try to negate the memory increase due to adding atoms
            rssFromIncrease += (getMemUsage() - rssBeforeIncrease);
        }
        size_t atomspaceSize;
        if (testKind == BENCH_TABLE)
            atomspaceSize = atab->getSize();
        else
            atomspaceSize = asp->getSize();
        timepair_t timeTaken = CALL_MEMBER_FN(*this, methodToCall)();
        sumAsyncTime += get<0>(timeTaken);
        counter++;
        if (saveInterval && counter % saveInterval == 0)
        {
            // Only save datapoints every saveInterval calls
            record_t dataPoint(atomspaceSize,get<0>(timeTaken),getMemUsage()-rssStart-rssFromIncrease);
            // Only save datapoints if we have to calculate the stats
            // afterwards, otherwise it affects memory usage
            if (doStats) {
                if (get<0>(timeTaken) < 0) cout << "ftumf" << endl;
                records.push_back(dataPoint);
            }
            // otherwise, we might write directly to a file
            if (saveToFile) recordToFile(myfile,dataPoint);
        }
        if (i % diff == 0) cerr << "." << flush;
    }
    Handle rh = getRandomHandle();
    gettimeofday(&tim, NULL);
    double t2=tim.tv_sec+(tim.tv_usec/1000000.0);
    printf("\n%.6lf seconds elapsed (%.2f per second)\n", t2-t1, 1.0f/((t2-t1)/Nreps));
    // rssEnd = getMemUsage();
    cout << "Sum clock() time for all requests: " << sumAsyncTime << " (" <<
        (float) sumAsyncTime / CLOCKS_PER_SEC << " seconds, "<<
        1.0f/(((float)sumAsyncTime/CLOCKS_PER_SEC) / (Nreps*Nloops)) << " requests per second)" << endl;
    //cout << "Memory (max RSS) change after benchmark: " <<
    //    (rssEnd - rssStart - rssFromIncrease) / 1024 << "kb" << endl;

    if (saveInterval && doStats)
    {
        // Only calculate stats if we've actually been saving datapoints
        // the option to calculate them is enabled
        AtomSpaceBenchmark::TimeStats t(records);
        t.print();
    }
    cout << DIVIDER_LINE << endl;
    if (saveToFile) { myfile.close(); }
}

void AtomSpaceBenchmark::startBenchmark(int numThreads)
{
    // Make sure we are using the correct link mean!
    if (prg) delete prg;
    prg = new std::poisson_distribution<unsigned>(linkSize_mean);

    // num threads does nothing at the moment;
    if (showTypeSizes) printTypeSizes();

    for (unsigned int i = 0; i < methodNames.size(); i++) {
        UUID_begin = TLB::getMaxUUID();
        UUID_end = TLB::getMaxUUID();
        if (testKind == BENCH_TABLE) {
            atab = new AtomTable();
        }
        else {
#if HAVE_CYTHON
            cogs = new CogServer();
            if (pymo == NULL) pymo = new PythonModule(*cogs);
            pymo->init();
            asp = &cogs->getAtomSpace();
            pyev = &PythonEval::instance();

            // And now ... create a Python instance of the atomspace.
            // Pass in the raw C++ atomspace address into cython.
            // Kind-of tacky, but I don't see any better way.
            // (We must do this because otherwise, the benchmark would
            // run on a different atomspace, than the one containing
            // all the atoms.  And that would give bad results.
            std::ostringstream dss;
            dss << "from opencog.atomspace import AtomSpace, types, Handle, TruthValue\n"
                << "aspace = AtomSpace(" << asp << ")\n";
            pyev->eval(dss.str());
#else
            asp = new AtomSpace();
#endif
#if HAVE_GUILE
            scm = new SchemeEval(asp);
#endif
        }
        numberOfTypes = classserver().getNumberOfClasses();

        if (buildTestData) buildAtomSpace(atomCount, percentLinks, false);
        UUID_end = TLB::getMaxUUID();

        doBenchmark(methodNames[i], methodsToTest[i]);

        if (testKind == BENCH_TABLE)
            delete atab;
        else {
#if HAVE_GUILE
            delete scm;
#endif
#if HAVE_CYTHON
            delete cogs;
#else
            delete asp;
#endif
        }
    }

    //cout << estimateOfAtomSize(Handle(2)) << endl;
    //cout << estimateOfAtomSize(Handle(1020)) << endl;
}

std::string
AtomSpaceBenchmark::memoize_or_compile(std::string exp)
{
#ifdef HAVE_GUILE
    if (memoize)
    {
        std::ostringstream dss;
        dss << "(define (mk) " << exp << ")\n";
        scm->eval(dss.str());
        return "(mk)\n";
    }
    if (compile)
    {
        std::ostringstream dss;
        dss << "(compile '(define (mk) " << exp
            << ") #:env (current-module))\n";
        scm->eval(dss.str());
        return "(mk)\n";
    }
#endif /* HAVE_GUILE */

    return exp;
}

Type AtomSpaceBenchmark::randomType(Type t)
{
    OC_ASSERT(t < numberOfTypes);
    Type randomType = ATOM + rng->randint(numberOfTypes-1);
    while (!classserver().isA(randomType, t))
        randomType = ATOM + rng->randint(numberOfTypes-1);
    return randomType;
}

clock_t AtomSpaceBenchmark::makeRandomNode(const std::string& csi)
{
#ifdef FIXME_LATER
    // Some faction of the time, we create atoms with non-default
    // truth values.  XXX implement this for making of links too...
    bool useDefaultTV = (rng->randfloat() < chanceUseDefaultTV);
    SimpleTruthValue stv(TruthValue::DEFAULT_TV());

    if (not useDefaultTV) {
        float strength = rng->randfloat();
        float conf = rng->randfloat();
        stv = SimpleTruthValue(strength, conf);
    }
#endif

    double p = rng->randdouble();
    Type t = defaultNodeType;
    if (p < chanceOfNonDefaultNode)
        t = randomType(NODE);

    std::string scp(csi);
    if (csi.size() ==  0) {
        std::ostringstream oss;
        counter++;

        // std::strtod in AtomTable::getHandle(Type t, std::string name) const
        // crashes if we pass in non-numeric values for the node name for
        // NUMBER_NODE types.
        if (NUMBER_NODE == t)
            oss << counter;
        else
            oss << "node " << counter;
        scp = oss.str();
    }

    switch (testKind) {
    case BENCH_TABLE: {
        clock_t t_begin = clock();
        atab->add(createNode(t, scp), false);
        return clock() - t_begin;
    }
    case BENCH_AS: {
        clock_t t_begin = clock();
        asp->addNode(t, scp);
        return clock() - t_begin;
    }
#if HAVE_GUILE
    case BENCH_SCM: {
        std::ostringstream ss;
        for (unsigned int i=0; i<Nloops; i++) {
            ss << "(cog-new-node '"
               << classserver().getTypeName(t)
               << " \"" << scp << "\")\n";

            p = rng->randdouble();
            t = defaultNodeType;
            if (p < chanceOfNonDefaultNode)
                t = randomType(NODE);

            scp = csi;
            if (csi.size() ==  0) {
                std::ostringstream oss;
                counter++;
                oss << "node " << counter;
                scp = oss.str();
            }
        }
        std::string gs = memoize_or_compile(ss.str());

        clock_t t_begin = clock();
        scm->eval_h(gs);
        return clock() - t_begin;
    }
#endif /* HAVE_GUILE */

#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        dss << "aspace.add_node (" << t << ", \"" << scp << "\")\n";
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */
    }

    return 0;
}

clock_t AtomSpaceBenchmark::makeRandomLink()
{
    Type t = defaultLinkType;
    double p = rng->randdouble();
    if (p < chanceOfNonDefaultLink) t = randomType(LINK);

    size_t arity = (*prg)(randgen);
    if (arity == 0) { ++arity; };

    // AtomSpace will throw if the context link has bad arity
    if (CONTEXT_LINK == t) arity = 2;

    HandleSeq outgoing;
    for (size_t j=0; j < arity; j++) {
        Handle h(getRandomHandle());
        outgoing.push_back(h);
    }

    switch (testKind) {
#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        dss << "aspace.add_link (" << t << ", [";
        for (size_t j=0; j < arity; j++) {
            dss << "Handle( " << outgoing[j].value() << ")";
            if (j < arity-1) dss  << ", ";
        }
        dss << " ] )\n";
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */

#if HAVE_GUILE
    case BENCH_SCM: {
        // This is somewhat more cumbersome and slower than what
        // anyone would actually do in scheme, because handles are
        // never handled in this way, but wtf, not much choice here.
        // I guess its quasi-realistic as a stand-in for other work
        // that might be done anyway...
        std::ostringstream ss;
        for (unsigned int i=0; i<Nloops; i++) {
            if (25 < arity) arity = 25;
            for (size_t j = 0; j < arity; j++) {
                char c = 'a' + j;
                ss << "(define h" << c
                   << " (cog-atom " << outgoing[j].value() << "))\n";
            }
            ss << "(cog-new-link '"
               << classserver().getTypeName(t);
            for (size_t j = 0; j < arity; j++) {
                char c = 'a' + j;
                ss << " h" << c;
            }
            ss << ")\n";

            t = defaultLinkType;
            p = rng->randdouble();
            if (p < chanceOfNonDefaultLink) t = randomType(LINK);

            arity = (*prg)(randgen);
            if (arity == 0) { ++arity; };

            // AtomSpace will throw if the context link has bad arity
            if (CONTEXT_LINK == t) arity = 2;

            outgoing.clear();
            for (size_t j=0; j < arity; j++) {
                outgoing.push_back(getRandomHandle());
            }
        }
        std::string gs = memoize_or_compile(ss.str());

        clock_t t_begin = clock();
        scm->eval_h(gs);
        return clock() - t_begin;
    }
#endif /* HAVE_GUILE */
    case BENCH_TABLE: {
        clock_t tAddLinkStart = clock();
        atab->add(createLink(t, outgoing), false);
        return clock() - tAddLinkStart;
    }
    case BENCH_AS: {
        clock_t tAddLinkStart = clock();
        asp->addLink(t, outgoing);
        return clock() - tAddLinkStart;
    }}
    return 0;
}

void AtomSpaceBenchmark::buildAtomSpace(long atomspaceSize, float _percentLinks, bool display)
{
    BenchType saveKind = testKind;
#if HAVE_CYTHON
    if (testKind == BENCH_PYTHON)
       testKind = BENCH_AS;
#endif /* HAVE_CYTHON */
#if HAVE_GUILE
    if (testKind == BENCH_SCM)
       testKind = BENCH_AS;
#endif /* HAVE_GUILE */

    clock_t tStart = clock();
    if (display) {
        cout << "Building atomspace with " << atomspaceSize << " atoms (" <<
            _percentLinks*100.0 << "\% links)" << endl;
    }

    // Add nodes
    long nodeCount = atomspaceSize * (1.0f - _percentLinks);
    int i;
    if (display) cout << "Adding " << nodeCount << " nodes ";
    int diff = nodeCount / PROGRESS_BAR_LENGTH;
    if (!diff) diff = 1;
    for (i=0; i<nodeCount; i++) {
        makeRandomNode("");
        if (display && i % diff == 0) cerr << "." << flush;
    }
    UUID_end = TLB::getMaxUUID();

    // Add links
    if (display) cout << endl << "Adding " << atomspaceSize - nodeCount << " links " << flush;
    diff = ((atomspaceSize - nodeCount) / PROGRESS_BAR_LENGTH);
    if (!diff) diff = 1;
    for (; i < atomspaceSize; i++) {
        makeRandomLink();
        if (display && (i-nodeCount) % diff == 0) { cerr << "." << flush; }
    }

    if (display) {
        cout << endl;
        printf("Built atomspace, execution time: %.2fs\n",
             (double)(clock() - tStart)/CLOCKS_PER_SEC);
        cout << DIVIDER_LINE << endl;
    }

    UUID_end = TLB::getMaxUUID();
    testKind = saveKind;
}

timepair_t AtomSpaceBenchmark::bm_noop()
{
    // Benchmark clock overhead.
    clock_t t_begin;
    clock_t time_taken;
    t_begin = clock();
    time_taken = clock() - t_begin;
    return timepair_t(time_taken,0);
}

timepair_t AtomSpaceBenchmark::bm_addNode()
{
    //cout << "Benchmarking AtomSpace::addNode" << endl;
    return timepair_t(makeRandomNode(""),0);
}

timepair_t AtomSpaceBenchmark::bm_addLink()
{
    //cout << "Benchmarking AtomSpace::addLink" << endl;
    return timepair_t(makeRandomLink(),0);
}

timepair_t AtomSpaceBenchmark::bm_rmAtom()
{
    Handle h = getRandomHandle();

    // Can't remove something that has incoming links, so find something
    // that doesn't.
    while (0 < h->getIncomingSetSize()) {
        h = getRandomHandle();
    }
    clock_t t_begin;
    clock_t time_taken;
    switch (testKind) {
#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        for (unsigned int i=0; i<Nloops; i++) {
            dss << "aspace.remove(Handle(" << h.value() << "))\n";
            h = getRandomHandle();
            // XXX FIXME --- this may have trouble finding anything if
            // Nloops is bigger than the number of links in the atomspace !
            while (0 < h->getIncomingSetSize()) {
                h = getRandomHandle();
            }
        }
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */
#if HAVE_GUILE
    case BENCH_SCM: {
        std::ostringstream ss;
        for (unsigned int i=0; i<Nloops; i++) {
            ss << "(cog-delete-recursive (cog-atom " << h.value() << "))\n";
            h = getRandomHandle();
            // XXX FIXME --- this may have trouble finding anything if
            // Nloops is bigger than the number of links in the atomspace !
            while (0 < h->getIncomingSetSize()) {
                h = getRandomHandle();
            }
        }
        std::string gs = memoize_or_compile(ss.str());

        clock_t t_begin = clock();
        scm->eval(gs);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
#endif /* HAVE_GUILE */
    case BENCH_TABLE: {
        t_begin = clock();
        atab->extract(h);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
    case BENCH_AS: {
        t_begin = clock();
        asp->removeAtom(h);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }}
    return timepair_t(0,0);
}

Handle AtomSpaceBenchmark::getRandomHandle()
{
    Handle h(UUID_begin + rng->randint(UUID_end-1-UUID_begin));
    // operator->() can return NULL when there's no atom for the uuid,
    // because the atom was deleted in a previous pass! Dohh!
    while (NULL == h.operator->()) {
        h = getRandomHandle();
    }
    return h;
}

timepair_t AtomSpaceBenchmark::bm_getType()
{
    Handle h = getRandomHandle();
    clock_t t_begin;
    clock_t time_taken;
    switch (testKind) {
#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        for (unsigned int i=0; i<Nloops; i++) {
            dss << "aspace.get_type(Handle(" << h.value() << "))\n";
            h = getRandomHandle();
        }
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */
#if HAVE_GUILE
    case BENCH_SCM: {
        std::ostringstream ss;
        for (unsigned int i=0; i<Nloops; i++) {
            ss << "(cog-type (cog-atom " << h.value() << "))\n";
            h = getRandomHandle();
        }
        std::string gs = memoize_or_compile(ss.str());

        clock_t t_begin = clock();
        scm->eval(gs);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
#endif /* HAVE_GUILE */
    case BENCH_TABLE: {
        t_begin = clock();
        h->getType();
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
    case BENCH_AS: {
        t_begin = clock();
        asp->getType(h);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }}
    return timepair_t(0,0);
}

timepair_t AtomSpaceBenchmark::bm_getTruthValue()
{
    Handle h = getRandomHandle();
    clock_t t_begin;
    clock_t time_taken;
    switch (testKind) {
#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        dss << "aspace.get_tv(Handle(" << h.value() << "))\n";
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */
#if HAVE_GUILE
    case BENCH_SCM: {
        std::ostringstream ss;
        for (unsigned int i=0; i<Nloops; i++) {
            ss << "(cog-tv (cog-atom " << h.value() << "))\n";
            h = getRandomHandle();
        }
        std::string gs = memoize_or_compile(ss.str());

        clock_t t_begin = clock();
        scm->eval(gs);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
#endif /* HAVE_GUILE */
    case BENCH_TABLE: {
        t_begin = clock();
        h->getTruthValue();
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
    case BENCH_AS: {
        t_begin = clock();
        asp->getTV(h);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }}
    return timepair_t(0,0);
}

#ifdef ZMQ_EXPERIMENT
timepair_t AtomSpaceBenchmark::bm_getTruthValueZmq()
{
    Handle h = getRandomHandle();
    clock_t t_begin = clock();
    asp->getTVZmq(h);
    return clock() - t_begin;
}
#endif

timepair_t AtomSpaceBenchmark::bm_setTruthValue()
{
    Handle h = getRandomHandle();

    float strength = rng->randfloat();
    float conf = rng->randfloat();

    clock_t t_begin;
    clock_t time_taken;
    switch (testKind) {
#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        dss << "aspace.set_tv(Handle(" << h.value()
            << "), TruthValue(" << strength << ", " << conf << "))\n";
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */
#if HAVE_GUILE
    case BENCH_SCM: {
        std::ostringstream ss;
        for (unsigned int i=0; i<Nloops; i++) {
            ss << "(cog-set-tv! (cog-atom " << h.value() << ")"
               << "   (cog-new-stv " << strength << " " << conf << ")"
               << ")\n";

            h = getRandomHandle();
            strength = rng->randfloat();
            conf = rng->randfloat();
        }
        std::string gs = memoize_or_compile(ss.str());

        clock_t t_begin = clock();
        scm->eval(gs);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
#endif /* HAVE_GUILE */
    case BENCH_TABLE: {
        t_begin = clock();
        TruthValuePtr stv(SimpleTruthValue::createTV(strength, conf));
        h->setTruthValue(stv);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
    case BENCH_AS: {
        t_begin = clock();
        TruthValuePtr stv(SimpleTruthValue::createTV(strength, conf));
        asp->setTV(h,stv);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }}
    return timepair_t(0,0);
}

timepair_t AtomSpaceBenchmark::bm_getNodeHandles()
{
    HandleSeq results;
    HandleSeq results2;
    // Build node name first
    std::ostringstream oss;
    counter++;
    oss << "node " << (rng->randint((atomCount-1) * percentLinks)+1);

    clock_t t_begin;
    clock_t time_taken;
    switch (testKind) {
#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        dss << "aspace.get_atoms_by_name(types.Node, \"" << oss.str() << "\", True)\n";
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */
#if HAVE_GUILE
    case BENCH_SCM: {
        // Currently not expose in the SCM API
        return timepair_t(0,0);
    }
#endif /* HAVE_GUILE */
    case BENCH_TABLE: {
        t_begin = clock();
        atab->getHandlesByName(back_inserter(results2), oss.str(), NODE, true);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
    case BENCH_AS: {
        t_begin = clock();
        asp->getHandlesByName(back_inserter(results), oss.str(), NODE, true);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }}
    return timepair_t(0,0);
}

timepair_t AtomSpaceBenchmark::bm_getHandleSet()
{
    Type t = randomType(ATOM);
    switch (testKind) {
#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        dss << "aspace.get_atoms_by_type(" << t << ", True)\n";
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */
#if HAVE_GUILE
    case BENCH_SCM: {
        // Currently not expose in the SCM API
        return timepair_t(0,0);
    }
#endif /* HAVE_GUILE */
    case BENCH_TABLE: {
        clock_t t_begin = clock();
        HandleSeq results;
        atab->getHandlesByType(back_inserter(results), t, true);
        clock_t time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
    case BENCH_AS: {
        HandleSeq results;
        clock_t t_begin = clock();
        asp->getHandlesByType(back_inserter(results), t, true);
        clock_t time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }}
    return timepair_t(0,0);
}

timepair_t AtomSpaceBenchmark::bm_getOutgoingSet()
{
    Handle h = getRandomHandle();
    clock_t t_begin;
    clock_t time_taken;
    switch (testKind) {
#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        dss << "aspace.get_outgoing(Handle(" << h.value() << "))\n";
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */
#if HAVE_GUILE
    case BENCH_SCM: {
        std::ostringstream ss;
        for (unsigned int i=0; i<Nloops; i++) {
            ss << "(cog-outgoing-set (cog-atom " << h.value() << "))\n";
            h = getRandomHandle();
        }
        std::string gs = memoize_or_compile(ss.str());

        clock_t t_begin = clock();
        scm->eval(gs);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
#endif /* HAVE_GUILE */
    case BENCH_TABLE: {
        t_begin = clock();
        LinkPtr l(atab->getLink(h));
        if (l) l->getOutgoingSet();
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
    case BENCH_AS: {
        t_begin = clock();
        asp->getOutgoing(h);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }}
    return timepair_t(0,0);
}

timepair_t AtomSpaceBenchmark::bm_getIncomingSet()
{
    Handle h = getRandomHandle();
    clock_t t_begin;
    clock_t time_taken;
    switch (testKind) {
#if HAVE_CYTHON
    case BENCH_PYTHON: {
        std::ostringstream dss;
        dss << "aspace.get_incoming(Handle(" << h.value() << "))\n";
        std::string ps = dss.str();
        clock_t t_begin = clock();
        pyev->eval(ps);
        return clock() - t_begin;
    }
#endif /* HAVE_CYTHON */
#if HAVE_GUILE
    case BENCH_SCM: {
        std::ostringstream ss;
        for (unsigned int i=0; i<Nloops; i++) {
            ss << "(cog-incoming-set (cog-atom " << h.value() << "))\n";
            h = getRandomHandle();
        }
        std::string gs = memoize_or_compile(ss.str());

        clock_t t_begin = clock();
        scm->eval(gs);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
#endif /* HAVE_GUILE */
    case BENCH_TABLE: {
        t_begin = clock();
        h->getIncomingSet();
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }
    case BENCH_AS: {
        t_begin = clock();
        asp->getIncoming(h);
        time_taken = clock() - t_begin;
        return timepair_t(time_taken,0);
    }}
    return timepair_t(0,0);
}

AtomSpaceBenchmark::TimeStats::TimeStats(
        const std::vector<record_t>& records)
{
    double sum = 0;
    t_min = 1 << 30;
    t_max = 0;
    for (record_t record : records) {
        sum += get<1>(record);
        if (get<1>(record) > t_max) t_max = get<1>(record);
        if (get<1>(record) < t_min) t_min = get<1>(record);
    }
    t_total = sum;
    t_N = records.size();
    t_mean = sum / t_N;
    sum = 0.0;
    for (record_t record : records) {
        clock_t value = (get<1>(record) - t_mean);
        sum += (value*value);
    }
    t_std = sqrt(sum/(t_N-1));
}

void AtomSpaceBenchmark::TimeStats::print()
{
    cout << "Per operation stats, in CPU clock ticks: " << endl;
    cout << "  N: " << t_N << endl;
    cout << "  mean: " << t_mean << endl;
    cout << "  min: " << t_min << endl;
    cout << "  max: " << t_max << endl;
    cout << "  std: " << t_std << endl;
}

void AtomSpaceBenchmark::recordToFile(std::ofstream& myfile, record_t record) const
{
    myfile << tuples::set_open(' ');
    myfile << tuples::set_close(' ');
    myfile << tuples::set_delimiter(',');
    myfile << record;
    myfile << "," << (float) get<1>(record) / CLOCKS_PER_SEC << endl;
}

}

