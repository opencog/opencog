
#include "ConsoleSocket.h"
#include "IHasMimeType.h"
#include "IRPCSocket.h"
#include "SocketHolder.h"

#include <Sockets/Lock.h>
#include <Sockets/ISocketHandler.h>
#include <Sockets/Socket.h>

#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

using namespace opencog;

SocketHolder::SocketHolder(void)
{
    _sock = NULL;
}

SocketHolder::~SocketHolder()
{
}

void SocketHolder::setSocket (TcpSocket *s)
{
    _sock = s;
}

int SocketHolder::AtomicInc(int inc)
{
    if (0 > inc)
    {
       if (_sock) {
           IRPCSocket* _rpcsock = dynamic_cast<IRPCSocket*>(_sock);
           if (_rpcsock) {
               _rpcsock->OnRequestComplete();
           }
       }
    }
    return 0;
}

void SocketHolder::send(const std::string& msg) const
{
    logger().debug("[SocketHolder] send\n");
    if (_sock) {
        Lock l(_sock->MasterHandler().GetMutex());
        _sock->Send(msg);
    }
}

void SocketHolder::SetCloseAndDelete(void)
{
    if (_sock) {
        _sock->SetCloseAndDelete();
    }
}

void SocketHolder::SetLineProtocol(bool b)
{
    if (_sock) {
        _sock->SetLineProtocol(b);
    }
}

std::string SocketHolder::mimeType(void)
{
    if (_sock) {
        IHasMimeType* ihmt = dynamic_cast<IHasMimeType*>(_sock);
        if (ihmt == NULL)
            throw RuntimeException(TRACE_INFO,
                "invalid socket: it does not have a mime-type.");
        return ihmt->mimeType();
    }
    return "";
}

void SocketHolder::SetShell(GenericShell *gs)
{
    if (_sock) {
       ConsoleSocket *cs = dynamic_cast<ConsoleSocket *>(_sock);
       if (cs) cs->SetShell(gs);
    }
}
