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
    zmq_context = new zmq::context_t(1);
    m_zmq_Thread = boost::thread(&AtomSpaceAsync::zmqLoop, this);
}

void AtomSpaceAsync::stopEventLoop()
{
    // Tell request worker thread to exit
    processingRequests=false;
    requestQueue.cancel();
    // rejoin thread
    m_Thread.join();
    delete zmq_context;
    m_zmq_Thread.join();

}

void AtomSpaceAsync::zmqLoop()
{
    //  Prepare our context and socket
    zmq::socket_t socket (*zmq_context, ZMQ_REP);
    socket.bind ("inproc://gettv");

    while (processingRequests) {
        zmq::message_t request;

        //  Wait for next request from client
        if (!socket.recv (&request, ZMQ_NOBLOCK)) {
            sleep(1);
            continue;
        }
        printf ("Received request");
        // Here we should interpret the type of the request and then dispatch
        // to the appropriate worker


        //  Send reply back to client
        zmq::message_t reply (5);
        memcpy ((void *) reply.data (), "World", 5);
        socket.send (reply);
    }
}

void AtomSpaceAsync::eventLoop()
{
    try {
        while (processingRequests) {
            boost::shared_ptr<ASRequest> req;
            requestQueue.wait_and_pop(req);
            counter++;
            req->run();
        }
    } catch (concurrent_queue< boost::shared_ptr<ASRequest> >::Canceled &e) {
        //cout << "End AtomSpace event loop" << endl;
        return;
    }
}

