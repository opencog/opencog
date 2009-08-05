#include "simple_nn.h"
bool compare_connection::operator() (ann_connection* lhs, ann_connection* rhs) {
  return lhs->source->sort_val < rhs->source->sort_val; 
}
