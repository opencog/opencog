#include "RunningProcedureId.h"

using namespace Procedure;

RunningProcedureId::RunningProcedureId(){
    this->type = COMBO;
}

RunningProcedureId::RunningProcedureId(unsigned long id, ProcedureType type){
    this->id = id;
    this->type = type;
}

void RunningProcedureId::setType(ProcedureType type){
    this->type = type;
}

const ProcedureType RunningProcedureId::getType() const {
    return this->type;
}

void RunningProcedureId::setId(unsigned long id){
    this->id = id;
}

const unsigned long RunningProcedureId::getId() const {
    return this->id;
}

bool RunningProcedureId::operator== (const RunningProcedureId& rpId) const {
    return (this->id == rpId.getId() &&
            this->type == rpId.getType());
}

bool RunningProcedureId::operator< (const RunningProcedureId& rpId) const {
    return (this->id < rpId.getId());
}

