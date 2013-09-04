
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{

// This type is returned from the agent_finder_api when loading a python module
// containing agents or requests
struct requests_and_agents_t {
    std::vector<std::string> agents;
    std::vector<std::string> requests;
    std::vector<std::string> req_summary;
    std::vector<std::string> req_description;
    std::vector<bool> req_is_shell;
    std::string err_string;
};

}
