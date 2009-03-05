#include "ComboReduct/combo/message.h"

std::ostream& operator<<(std::ostream& out,const combo::message& m) {
  return out << combo::message::prefix() << '\"' << m.getContent() << '\"';
}

