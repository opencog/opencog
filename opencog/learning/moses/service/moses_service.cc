#include <iostream>
#include "../main/moses_exec.h"
//#include "../main/moses_exec_def.h"
#include "moses_service.h"
//#include <vector>

using namespace std;

namespace opencog {
namespace moses {

moses_service::moses_service()
{
    cout << "MOSES Service instanciated";
}

moses_service::~moses_service()
{
}

void moses_service::run()
{
    const std::vector<std::string>& args = {
        "moses",
        "-i",
        //"/home/cosmo/opencog/src/qtbin/opencog/learning/moses/main/test/xor.txt",
        "./xor.txt",
        "-c",
        "1",
        "-o",
        //"/home/cosmo/opencog/src/qtbin/opencog/learning/moses/main/test/out.txt"
        "./out.txt"
    };

    moses_exec(args);

}

}
}
