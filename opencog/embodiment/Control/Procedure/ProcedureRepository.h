#ifndef PROCEDUREREPOSITORY_H_
#define PROCEDUREREPOSITORY_H_

#include <opencog/atomspace/SavableRepository.h>
#include "ComboProcedureRepository.h"
#include "ComboSelectProcedureRepository.h"
#include "BuiltInProcedureRepository.h"

#include <string>
#include <map>

using namespace opencog;

namespace Procedure {
    
class ProcedureRepository : public SavableRepository {

private:
    ComboProcedureRepository comboRepository;
    BuiltInProcedureRepository& builtInRepository;
    ComboSelectProcedureRepository * comboSelectRepository; 

public:
    ProcedureRepository(PerceptionActionInterface::PAI&);
    virtual ~ProcedureRepository();
    bool contains(const std::string& name) const;
    void remove(const std::string& name); //remove a combo procedure_call
    const GeneralProcedure& get(const std::string& name) const; 
    void add(const ComboProcedure& cp);
    void add(const ComboSelectProcedure& cp);

    const ComboProcedureRepository& getComboRepository() const; 
    const ComboSelectProcedureRepository& getComboSelectRepository() const; 
    const BuiltInProcedureRepository& getBuiltInRepository() const;
    
    // Methods from SavableRepository interface
    const char* getId() const;
    void saveRepository(FILE*) const;
    
    int loadComboFromStream(istream& in);
    int loadComboSelectFromStream(istream& in);
    
    void loadRepository(FILE*, HandleMap<Atom *>*);
    void clear();
};

};


#endif /*PROCEDUREREPOSITORY_H_*/
