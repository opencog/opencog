#ifndef BUILTIN_PROCEDURE_REPOSITORY_H
#define BUILTIN_PROCEDURE_REPOSITORY_H

#include <string>
#include <map>

#include "BuiltInProcedure.h"
#include "PAI.h"

namespace Procedure {
    
class BuiltInProcedureRepository {

private:
    typedef std::map<std::string, BuiltInProcedure*> Name2ProcedureMap; 
    Name2ProcedureMap _map;

    void add(BuiltInProcedure* proc);

public:
    BuiltInProcedureRepository(PerceptionActionInterface::PAI&);
    ~BuiltInProcedureRepository();
    
    bool contains(const std::string& name) const; 
    const BuiltInProcedure& get(const std::string& name) const; 

    /// Update the variable contents of the schemata
    bool update(AtomSpace& atomspace); 
};

}

#endif
