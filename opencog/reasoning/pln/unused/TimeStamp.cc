#include "TimeStamp.h"

TimeStamp::TimeStamp(bool normal, unsigned long a, unsigned long b) : Temporal(normal, a, b) {
}

TimeStamp::TimeStamp(bool normal, unsigned long value) : Temporal(normal, value, normal?0:value) {
}

TimeStamp::~TimeStamp() {
}

unsigned long TimeStamp::getValue() {
    return getA();
}

