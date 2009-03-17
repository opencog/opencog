#ifndef COMBO_PROCEDURE_REPOSITORY_H
#define COMBO_PROCEDURE_REPOSITORY_H

#include "ComboProcedure.h"
#include <opencog/atomspace/SavableRepository.h>
#include "comboreduct/combo/vertex.h"
#include "comboreduct/combo/procedure_repository.h"

#include <string>
#include <map>
#include <boost/noncopyable.hpp>
#include <opencog/atomspace/HandleMap.h>

using namespace opencog;

namespace Procedure {
  //noncopyable because combo_trees may point at procedures in the repository
class ComboProcedureRepository : public combo::procedure_repository, public SavableRepository,public boost::noncopyable {

  //private:
  //typedef std::map<std::string, ComboProcedure> Name2ProcedureMap; 
  //Name2ProcedureMap _map;

public:
    //the location of the standard library
    static const std::string stdlibPath;

    ComboProcedureRepository();

    //parse and load procedures from a stream - returns # of procedures loaded
    unsigned int loadFromStream(std::istream& in);

    bool contains(const std::string& name) const; 
    const ComboProcedure& get(const std::string& name) const; 
    void add(const ComboProcedure& cp);

    //set procedure calls to point to the procedures stored in this repository
    void instantiateProcedureCalls(combo::combo_tree& tr,bool warnOnDefiniteObj=false) const;
    
    // Methods from SavableRepository interface
    const char* getId() const;
    void saveRepository(FILE*) const;
    void loadRepository(FILE*, HandleMap<Atom *>*);
    void clear();
};

}

#endif
