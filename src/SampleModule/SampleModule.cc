#include <platform.h>
#include <CogServer.h>

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
