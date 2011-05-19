namespace opencog
{

// This type is returned from the agent_finder_api when loading a python module
// containing agents or requests
struct requests_and_agents_t {
    std::vector<std::string> agents;
    std::vector<std::string> requests;
    std::string err_string;
};

}
