#include "ComboReduct/combo/perception.h"
#include "ComboReduct/combo/descriptions.h"

std::ostream& operator<<(std::ostream& out,combo::indefinite_object i) {
  return out << i->get_name();
}

