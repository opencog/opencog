#include "ComboSelectProcedure.h"

using namespace Procedure;

ComboSelectProcedure::ComboSelectProcedure(){
    this->name = "";
    this->firstScriptName = "";
    this->secondScriptName = "";
}

//ComboSelectProcedure::ComboSelectProcedure(const std::string& _n, const std::string& _f, const std::string& _s) :
//                                           name(_n), firstScriptName(_f), secondScriptName(_s){
//}

ComboSelectProcedure::ComboSelectProcedure(const std::string& _n,
                                           const ComboProcedure& _f,
                                           const ComboProcedure& _s)
    : name(_n), firstScript(_f), secondScript(_s){

    this->firstScriptName = firstScript.getName();       
    this->secondScriptName = secondScript.getName();       
}

ComboSelectProcedure::~ComboSelectProcedure(){
}


ProcedureType ComboSelectProcedure::getType() const {
    return Procedure::COMBO_SELECT;
}

unsigned int ComboSelectProcedure::getArity() const {
    // for now only ComboSelectProcedure without parameters
    return 0;
}

const std::string& ComboSelectProcedure::getName() const{
    return this->name;
}

const ComboProcedure& ComboSelectProcedure::getFirstScript() const {
    return this->firstScript;
}

const ComboProcedure& ComboSelectProcedure::getSecondScript() const {
    return this->secondScript;
}

const std::string& ComboSelectProcedure::getFirstScriptName() const{
    return this->firstScriptName;
}

const std::string& ComboSelectProcedure::getSecondScriptName() const{
    return this->secondScriptName;
}

