#ifndef _RUNNING_PROCEDURE_ID_H
#define _RUNNING_PROCEDURE_ID_H

#include "GeneralProcedure.h"

namespace Procedure{

class RunningProcedureId {

    public:
        RunningProcedureId();
        RunningProcedureId(unsigned long id, ProcedureType type);

        // getters and setters
        void setType(ProcedureType type);
        const ProcedureType getType() const;

        void setId(unsigned long id);
        const unsigned long getId() const;

        // operator overload
        bool operator== (const RunningProcedureId& rpId) const;
        bool operator< (const RunningProcedureId& rpId) const;

    private:
        unsigned long id;
        ProcedureType type;

}; // class
}  // namespace

#endif

