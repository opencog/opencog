#include "comboreduct/combo/builtin_action.h"

std::ostream& operator<<(std::ostream& out, combo::builtin_action a) {
  opencog::cassert(TRACE_INFO, a);
  return out << a->get_name();
}
