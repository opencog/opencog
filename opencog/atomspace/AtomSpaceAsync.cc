#include "AtomSpaceAsync.h"

using namespace opencog;

AtomSpaceAsync::AtomSpaceAsync()
{
    processingRequests = false;
    counter = 0;
    // Start event loop
    startEventLoop();
}

AtomSpaceAsync::~AtomSpaceAsync()
{
    stopEventLoop();
};

void AtomSpaceAsync::startEventLoop()
{
    processingRequests = true;
    m_Thread = boost::thread(&AtomSpaceAsync::eventLoop, this);
}

void AtomSpaceAsync::stopEventLoop()
{
    // Tell request worker thread to exit
    processingRequests=false;
    requestQueue.cancel();
    // rejoin thread
    m_Thread.join();

}

void AtomSpaceAsync::eventLoop()
{
try
    {
        while (processingRequests)
        {
            ASRequestPtr req;
            requestQueue.wait_and_get(req);
            counter++;
            req->run();
            requestQueue.pop();
        }
    }
    catch (concurrent_queue<ASRequestPtr>::Canceled &e)
    {
        //cout << "End AtomSpace event loop" << endl;
    }
}

