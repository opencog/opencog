#include "comboreduct/combo/perception.h"
#include "comboreduct/combo/descriptions.h"

std::ostream& operator<<(std::ostream& out,combo::indefinite_object i) {
  return out << i->get_name();
}

