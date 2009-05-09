#include <stdio.h>
#include <map>
#include <string>

namespace test
{
    FILE *logfile=NULL;
    //! @todo This variable is modified but not used anywhere
    int attachs=0;
}

//namespace haxx
//{
//    unsigned int maxDepth = 250;
//}

namespace opencog {
namespace pln
{
    //! @todo This variable is modified but not used anywhere
    int varcount=0;
    
    //! @todo This doesn't seem to get initialised anywhere in PLN
    std::map<int, std::string> type2name;
}}

