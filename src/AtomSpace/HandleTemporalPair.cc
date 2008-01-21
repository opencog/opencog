#include "HandleTemporalPair.h"
#include "Atom.h"
#include "TLB.h"

HandleTemporalPair::HandleTemporalPair() {
}

HandleTemporalPair::HandleTemporalPair(Handle handle, Temporal* time) {
    this->handle = handle;
    this->time = time;
}

HandleTemporalPair::~HandleTemporalPair() {
}

Handle HandleTemporalPair::getHandle() const{
    return handle;
}

Temporal* HandleTemporalPair::getTemporal() const{
    return time;
}

std::string HandleTemporalPair::toString() const{
    Atom* atom = TLB::getAtom(handle);
    std::string  answer;
    answer += "(" + atom->toShortString() + "," + time->toString() + ")";
    return answer;
}

HandleTemporalPair HandleTemporalPair::clone() {
    return HandleTemporalPair(handle, time);
}
