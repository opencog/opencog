#include "ComboReduct/combo/perception.h"
#include "ComboReduct/combo/descriptions.h"

std::ostream& operator<<(std::ostream& out, combo::perception p) {
  LADSUtil::cassert(TRACE_INFO, p);
  return out << p->get_name();
}
