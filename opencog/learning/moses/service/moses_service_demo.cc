#include <iostream>
#include "../main/moses_exec.h"
#include "moses_service.h"
using namespace std;
using namespace opencog::moses;

int main(int argc, char** argv)
{
    cout << "MOSES 'xor' example program.\n";

    // opencog::moses::
    moses_service moses;

    moses.run();

    return 1;
}
