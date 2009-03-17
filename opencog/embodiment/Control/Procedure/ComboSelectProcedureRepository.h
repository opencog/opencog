#ifndef COMBO_SELECT_PROCEDURE_REPOSITORY_H
#define COMBO_SELECT_PROCEDURE_REPOSITORY_H

#include "ComboSelectProcedure.h"
#include "ComboProcedureRepository.h"

#include <opencog/atomspace/SavableRepository.h>

#include <string>
#include <map>
#include <boost/noncopyable.hpp>

namespace Procedure {
    
typedef std::map<std::string, ComboSelectProcedure> Name2ProcedureMap;
typedef std::map<std::string, ComboSelectProcedure>::const_iterator Name2ProcedureMapIterator;

//noncopyable because combo_trees may point at procedures in the repository
class ComboSelectProcedureRepository : public SavableRepository , public boost::noncopyable {

    public:

        // creates a ComboSelectProcedureRepository. The comboRepository used as
        // argument MUST be the same global comboRepository. It is used mostly
        // to save and load first and second scripts
        ComboSelectProcedureRepository(ComboProcedureRepository& );

        // inform if the repository contains a ComboSelectProcedure with the
        // given name
        bool contains(const std::string& name) const;
    
        // get the ComboSelectProcedure 
        const ComboSelectProcedure& get(const std::string& name) ;

        // add a ComboSelectProcedure to repository. 
        void add(const ComboSelectProcedure& procedure);

        // remove a ComboSelectProcedure from repository. The first and second
        // ComboProcedures stored within the ComboRepository are not removed
        // since they can be reused.
        void remove(const std::string& name);

        // return a reference to the ComboRepository
        ComboProcedureRepository& getComboRepository();

        unsigned int loadFromStream(std::istream& in);

        // Methods from SavableRepository interface
        const char* getId() const;
        void saveRepository(FILE*) const;
        void loadRepository(FILE*, HandleMap<Atom *>*);
        void clear();

    private:

        Name2ProcedureMap procedureMap;
        ComboProcedureRepository& comboRepository;

}; // class
}  // namespace

#endif
