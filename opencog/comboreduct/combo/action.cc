#include "comboreduct/combo/action.h"
#include "comboreduct/combo/descriptions.h"


namespace combo
{

    arity_t get_arity(action a)
    {
        using namespace action_properties;
        return actions_properties::instance().action_arity(a);
    }
}



std::ostream& operator<<(std::ostream& out, const combo::action& a)
{
    using namespace combo;
    switch (a) {
    case id::sequential_and:
        return out << "and_seq";
    case id::sequential_or:
        return out << "or_seq";
    case id::sequential_exec:
        return out << "exec_seq";
    case id::action_not:
        return out << "action_not";
    case id::action_if:
        return out << "action_boolean_if";
    case id::boolean_action_if:
        return out << "boolean_action_if";
    case id::contin_action_if:
        return out << "contin_action_if";
    case id::action_action_if:
        return out << "action_action_if";
    case id::action_failure:
        return out << "action_failure";
    case id::action_success:
        return out << "action_success";
    case id::action_while:
        return out << "action_while";
    case id::boolean_while:
        return out << "boolean_while";
    case id::return_success:
        return out << "return_success";
    case id::repeat_n:
        return out << "repeat_n";
    default:
        return out << "ACTION : UNKNOWN_HANDLE";
    }
}
