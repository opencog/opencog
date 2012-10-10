#include "AtomSpaceAsync.h"
#include "TimeServer.h"

using namespace opencog;

AtomSpaceAsync::AtomSpaceAsync()
{
    processingRequests = false;
    counter = 0;
    spaceServer = new SpaceServer(*this);
    timeServer = new TimeServer(*this,spaceServer);
    spaceServer->setTimeServer(timeServer);
    atomspace.setSpaceServer(spaceServer);
    // Start event loop
    startEventLoop();
}

AtomSpaceAsync::~AtomSpaceAsync()
{
    stopEventLoop();
    spaceServer->setTimeServer(NULL);
    delete timeServer;
    delete spaceServer;
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
            boost::shared_ptr<ASRequest> req;
            requestQueue.wait_and_get(req);
            counter++;
            req->run();
            requestQueue.pop();
        }
    }
    catch (concurrent_queue< boost::shared_ptr<ASRequest> >::Canceled &e)
    {
        //cout << "End AtomSpace event loop" << endl;
    }
}

