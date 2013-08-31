#include "headers.h"
#include "../main/moses_exec.h"

int main(int argc, char** argv)
{
    cout << "MOSES 'xor' example program.\n";

    //int spoof_argc;
    //char** spoof_argv;

    //spoof_argc = 3;
    //spoof_argv = "-i test/xor.txt";

    const vector<string>& args = {"moses", "-i",  "~/opencog/src/qtbin/opencog/learning/moses/example-progs/test/xor.txt"};

    opencog::moses::moses_exec(args);
}
