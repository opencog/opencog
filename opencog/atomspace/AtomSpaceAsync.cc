#include "AtomSpaceAsync.h"
#include "TimeServer.h"

using namespace opencog;

#ifdef ZMQ_EXPERIMENT
zmq::context_t* AtomSpaceAsync::zmqContext= new zmq::context_t(1);
#endif

AtomSpaceAsync::AtomSpaceAsync()
{
#ifdef ZMQ_EXPERIMENT
	zmqClientEnabled = false;
	zmqServerEnabled = false;
#endif
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
#ifdef ZMQ_EXPERIMENT
	if(zmqServerEnabled)
	{
		//TODO send signal to exit gracefully
		//zmqServerThread.join();
	}
#endif

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

#ifdef ZMQ_EXPERIMENT

void AtomSpaceAsync::enableZMQClient(string networkAddress)
{
	assert(!zmqClientEnabled && "EnableZMQClient can only be called once.");
	zmqClientEnabled = true;
    zmqClientSocket = new zmq::socket_t(*zmqContext, ZMQ_REQ);
    zmqClientSocket->connect (networkAddress.c_str());
}

void AtomSpaceAsync::enableZMQServer(string networkAddress)
{
	assert(!zmqServerEnabled && "EnableZMQServer can only be called once.");
	zmqServerEnabled = true;
    zmqServerThread = boost::thread(boost::bind(&AtomSpaceAsync::zmqLoop, this, networkAddress));
}

void AtomSpaceAsync::zmqLoop(string networkAddress)
{
	//  Prepare our context and socket
    zmq::socket_t zmqServerSocket (*zmqContext, ZMQ_REP);
    zmqServerSocket.bind (networkAddress.c_str());

    while (zmqServerEnabled)
    {
        //  Wait for next request from client
        zmq::message_t request;
 cout<<"Server listening..." << endl;
        zmqServerSocket.recv (&request);
 cout<<"Message received..." << endl;
        //TODO check for exit signal
 	 	//TODO check for errors, log and retry?
        ZMQRequestMessage requestMessage;
		requestMessage.ParseFromArray(request.data(), request.size());

		ZMQReplyMessage replyMessage;
		switch(requestMessage.function())
		{
			case ZMQgetAtom:
			{
				boost::shared_ptr<Atom> atom = getAtom(Handle(requestMessage.handle()))->get_result();
				atom->writeToZMQMessage(replyMessage.mutable_atom());
				break;
			}
			case ZMQgetName:
			{
				string name = getName(Handle(requestMessage.handle()))->get_result();
				replyMessage.set_str(name);
				break;
			}
				//TODO add other functions
			default:
				assert(!"Invalid ZMQ function");
		}

		// Send reply back to client
		string strReply = replyMessage.SerializeAsString();
		zmq::message_t reply (strReply.size());
		memcpy ((void *) reply.data (), strReply.c_str(), strReply.size()); //TODO in place init
		zmqServerSocket.send (reply);
    }
}
#endif

void AtomSpaceAsync::eventLoop()
{
try
    {
        while (processingRequests)
        {
            boost::shared_ptr<ASRequest> req;
            requestQueue.wait_and_get(req);
            counter++;
#ifdef ZMQ_EXPERIMENT
			if(zmqClientEnabled)
			{
				ZMQRequestMessage requestMessage;
				req->copyParametersToZMQRequest(requestMessage);

				// Send request to server
				string strRequest = requestMessage.SerializeAsString();
				zmq::message_t request(strRequest.size());
				memcpy((void *) request.data (), strRequest.c_str(), strRequest.size()); //TODO use copyless init from data
				zmqClientSocket->send(request);

				// Wait for reply
				zmq::message_t reply;
				zmqClientSocket->recv(&reply);
				ZMQReplyMessage replyMessage;
				replyMessage.ParseFromArray(reply.data(), reply.size());

				req->copyResultFromZMQReply(replyMessage);
			}
			else
				req->run();
#else
			req->run();
#endif
            requestQueue.pop();
        }
    }
    catch (concurrent_queue< boost::shared_ptr<ASRequest> >::Canceled &e)
    {
        //cout << "End AtomSpace event loop" << endl;
    }
}

