#include <platform.h>
#include <CogServer.h>
#include <string>
#include <queue>

using namespace opencog;

class Foo
{
public:
    Foo() { 
        printf("hello world\n"); 
        AtomSpace *space = CogServer::getAtomSpace();
    }
};

Foo f;

extern "C" std::string cmd_hello(std::queue<std::string> &args)
{
    return "hello world";
}
