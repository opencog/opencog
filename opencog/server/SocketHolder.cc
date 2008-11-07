
#include "SocketHolder.h"

#include <Sockets/Lock.h>
#include <Sockets/ISocketHandler.h>
#include <Sockets/Socket.h>

#include <opencog/util/Logger.h>

using namespace opencog;

SocketHolder::SocketHolder(void)
{
    _sock = NULL;
}

SocketHolder::~SocketHolder()
{
}

void SocketHolder::send(const std::string& msg) const
{
    logger().debug("[SocketHolder] send\n");
    if (_sock) {
        Lock l(_sock->MasterHandler().GetMutex());
        _sock->Send(msg);
    }
}

