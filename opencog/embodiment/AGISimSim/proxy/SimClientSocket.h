#ifndef _SIMCLIENTSOCKET_H
#define _SIMCLIENTSOCKET_H

#include <Sockets/TcpSocket.h>

#include "AsynchronousMessageReceiver.h"

#include <string>

class ISocketHandler;
class TcpSocket;

class SimClientSocket : public TcpSocket
{
public:
	SimClientSocket(ISocketHandler&, AsynchronousMessageReceiver* receiver, bool echoing = false);
	~SimClientSocket();

	// Overiden methods
	void OnConnect();
	void OnConnectFailed();
	void OnLine(const std::string&);
	void Send(const std::string &str);
    void markWaitingForResponse();

    // Specific methods
	bool isWaitingForResponse();
    void cancelWaitingForResponse();
	std::string getResponse();
    bool ConnectionFailed();

private:
    bool m_bEchoing;
	bool m_bWaitingForResponse;
	bool m_bConnectionFailed;
	std::string m_sResponse;
    AsynchronousMessageReceiver* receiver;
};




#endif // _SIMCLIENTSOCKET_H
