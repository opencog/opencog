#include <iostream>
#include "moses_service.h"

using namespace std;
using namespace opencog::moses;

int main(int argc, char** argv)
{
    moses_service moses;

    moses.run();

    return 1;
}
