#include <iostream>
#include <pthread.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <opencog/util/concurrent_queue.h>

#include "AtomSpaceImpl.h"
#include "ASRequest.h"
#include "Handle.h"
#include "types.h"

namespace opencog {

class AtomSpaceAsync {

    bool processingRequests;
    boost::thread m_Thread;
    mutable pthread_mutex_t atomSpaceLock;
    int counter;

    AtomSpaceImpl atomspace;
public: 
    concurrent_queue< boost::shared_ptr<ASRequest> > requestQueue;
    AtomSpaceAsync() {
        pthread_mutex_init(&atomSpaceLock, NULL); 
        processingRequests = false;
        counter = 0;
        // Start event loop
        startEventLoop();
    }
    ~AtomSpaceAsync() {
        stopEventLoop();
    };

    int get_counter() { return counter; }
    void eventLoop()
    {
        try {
            while (processingRequests) {
                boost::shared_ptr<ASRequest> req;
                requestQueue.wait_and_pop(req);
                counter++;
                req->run();
            }
        } catch (concurrent_queue< boost::shared_ptr<ASRequest> >::Canceled &e) {
            cout << "End AtomSpace event loop" << endl;
        }
    }

    void startEventLoop()
    {
        processingRequests = true;
        m_Thread = boost::thread(&AtomSpaceAsync::eventLoop, this);
    }

    void stopEventLoop()
    {
        // Tell request worker thread to exit
        processingRequests=false;
        requestQueue.cancel();
        // rejoin thread
        m_Thread.join();
    }

    HandleRequest addNode(Type t, const std::string& str,
            const TruthValue& tvn = TruthValue::DEFAULT_TV() ) {
        HandleRequest hr(new AddNodeASR(&atomspace,t,str,tvn));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Retrieve from the Atom Table the Handle of a given node
     *
     * @param t     Type of the node
     * @param str   Name of the node
    */
    HandleRequest getHandle(Type t, const std::string& str) {
        HandleRequest hr(new GetNodeHandleASR(&atomspace,t,str));
        requestQueue.push(hr);
        return hr;
    }

    /**
     * Retrieve from the Atom Table the Handle of a given link
     * @param t        Type of the node
     * @param outgoing a reference to a HandleSeq containing
     *        the outgoing set of the link.
    */
    HandleRequest getHandle(Type t, const HandleSeq& outgoing) {
        HandleRequest hr(new GetLinkHandleASR(&atomspace,t,outgoing));
        requestQueue.push(hr);
        return hr;
    }

};

} // namespace opencog
