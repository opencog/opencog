#include "node_list.hpp"

namespace pleasure
{
/*std::istream& stream_to_node_list(std::istream& in, node_list& list)
{
    std::string str;
    std::getline(in, str);
    if (str != "")
        list.insert(combo::str_to_vertex<combo::ant_builtin_action, combo::ant_perception, combo::ant_action_symbol, combo::ant_indefinite_object>(str));
}*/

void stream_to_node_list(std::istream& in, node_list& list)
{
    std::string str;
    std::getline(in, str);
    if (str != "")
        list.insert(combo::str_to_vertex<combo::ant_builtin_action, combo::ant_perception, combo::ant_action_symbol, combo::ant_indefinite_object>(str));
}
}
