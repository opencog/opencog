#include <string>
#include <queue>

#include <opencog/util/platform.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

class Foo
{
public:
    Foo() {
        printf("hello world\n");
        CogServer::getAtomSpace();
    }
};

Foo f;

extern "C" std::string cmd_hello(std::queue<std::string> &args)
{
    return "hello world";
}
