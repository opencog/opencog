#include <set>
#include <istream>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>
#include <opencog/comboreduct/combo/builtin_action.h>
#include <opencog/comboreduct/combo/perception.h>
#include <opencog/comboreduct/combo/action_symbol.h>
#include <opencog/comboreduct/combo/indefinite_object.h>

namespace pleasure {
    typedef std::set<opencog::combo::vertex> node_list;

    //std::istream& stream_to_node_list(std::istream& is, node_list& list);
    void stream_to_node_list(std::istream& is, node_list& list);
}

