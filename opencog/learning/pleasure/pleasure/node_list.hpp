#include <set>
#include <istream>

#include "comboreduct/combo/vertex.h"
#include "comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h"
#include "comboreduct/combo/builtin_action.h"
#include "comboreduct/combo/perception.h"
#include "comboreduct/combo/action_symbol.h"
#include "comboreduct/combo/indefinite_object.h"

namespace pleasure {
    typedef std::set<combo::vertex> node_list;

    //std::istream& stream_to_node_list(std::istream& is, node_list& list);
    void stream_to_node_list(std::istream& is, node_list& list);
}

