#include <iostream>
#include "../main/moses_exec.h"
#include "moses_service.h"

using namespace std;

namespace opencog {
namespace moses {

moses_service::moses_service()
{
    cout << "MOSES Service instanciated\n";
}

moses_service::~moses_service()
{
}

void moses_service::run(int argc, char** argv)
{
    // Later, this method will be extended to return details on the metapopulation
    // For now, it just calls moses_exec
    moses_exec(argc, argv);
}

}
}
