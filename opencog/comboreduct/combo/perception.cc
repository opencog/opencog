#include "comboreduct/combo/perception.h"
#include "comboreduct/combo/descriptions.h"

std::ostream& operator<<(std::ostream& out, combo::perception p) {
  opencog::cassert(TRACE_INFO, p);
  return out << p->get_name();
}
