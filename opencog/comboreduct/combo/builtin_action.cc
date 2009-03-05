#include "ComboReduct/combo/builtin_action.h"

std::ostream& operator<<(std::ostream& out, combo::builtin_action a) {
  LADSUtil::cassert(TRACE_INFO, a);
  return out << a->get_name();
}
