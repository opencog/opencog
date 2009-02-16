#include <stdio.h>
#include <map>
#include <string>

namespace test
{
    FILE *logfile=NULL;
    int attachs=0;
}

//namespace haxx
//{
//    unsigned int maxDepth = 250;
//}

namespace reasoning
{
    int varcount=0;
    
    // TODO This doesn't seem to get initialised anywhere in PLN
    std::map<int, std::string> type2name;
}

