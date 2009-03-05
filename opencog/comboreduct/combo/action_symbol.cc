#include "ComboReduct/combo/action_symbol.h"

std::ostream& operator<<(std::ostream& out, combo::action_symbol as) {
  LADSUtil::cassert(TRACE_INFO, as);
  return out << as->get_name();
}
